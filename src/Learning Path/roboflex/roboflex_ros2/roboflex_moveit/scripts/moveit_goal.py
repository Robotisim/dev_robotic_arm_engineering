#!/usr/bin/env python3
"""Plan small raised-arm goals; --execute sends the validated path through MoveIt."""
import argparse
import copy
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (AllowedCollisionEntry, CollisionObject, Constraints,
                             JointConstraint, PlanningScene, PlanningSceneComponents,
                             PositionConstraint, RobotState)
from moveit_msgs.srv import (ApplyPlanningScene, GetMotionPlan, GetPlanningScene,
                             GetPositionFK, GetPositionIK, GetStateValidity)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
import xacro

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
ALL_JOINTS = JOINTS + ['joint_gripper']
TOOL = 'gripper_drive_link'
FLOOR_ID = 'roboflex_table_clearance'


def servo_degrees(q):
    u = [180 * (v + 1.57) / 3.14 for v in q]
    return [u[0] - 90, 120 - u[1], u[2], u[3], u[4]]


def trajectory_samples(trajectory):
    """Sample the controller's position/velocity/acceleration quintic spline."""
    points = trajectory.points
    times = [p.time_from_start.sec + p.time_from_start.nanosec / 1e9 for p in points]
    if len(points) < 2 or times[0] != 0 or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('Trajectory must start at zero and have increasing timestamps')
    if times[-1] > 30:
        raise ValueError('Trajectory exceeds the 30-second trial limit')
    for p in points:
        if not all(len(v) == 4 for v in [p.positions, p.velocities, p.accelerations]):
            raise ValueError('Expected four positions, velocities and accelerations per point')
        if not all(math.isfinite(v) for row in [p.positions, p.velocities, p.accelerations] for v in row):
            raise ValueError('Non-finite trajectory value')
    samples = []
    for i, (begin, finish) in enumerate(zip(times, times[1:])):
        duration = finish - begin
        left, right = points[i:i+2]
        coefficients = []
        for j in range(4):
            a0, a1, a2 = left.positions[j], left.velocities[j], left.accelerations[j] / 2
            b0 = right.positions[j] - (a0 + a1*duration + a2*duration**2)
            b1 = right.velocities[j] - (a1 + 2*a2*duration)
            b2 = right.accelerations[j] - 2*a2
            coefficients.append([a0, a1, a2,
                10*b0/duration**3 - 4*b1/duration**2 + b2/(2*duration),
                -15*b0/duration**4 + 7*b1/duration**3 - b2/duration**2,
                6*b0/duration**5 - 3*b1/duration**4 + b2/(2*duration**3)])
        steps = max(1, math.ceil(duration / 0.04))
        for step in range(steps + 1):
            t = duration * step / steps
            positions = [sum(a*t**k for k,a in enumerate(c)) for c in coefficients]
            velocities = [sum(k*c[k]*t**(k-1) for k in range(1,6)) for c in coefficients]
            accelerations = [sum(k*(k-1)*c[k]*t**(k-2) for k in range(2,6)) for c in coefficients]
            samples.append((begin+t, positions, velocities, accelerations))
    return times[-1], samples


class GoalClient(Node):
    def __init__(self):
        super().__init__('roboflex_guarded_moveit_goal')
        self.joints = None
        self.joints_at = 0.0
        self.firmware = None
        self.firmware_at = 0.0
        self.service_clients = {}
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_subscription(String, '/roboflex/firmware_status', self.on_firmware, 10)
        self.action = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.goal_handle = None

    def on_joints(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values for n in ALL_JOINTS):
            self.joints = [values[n] for n in ALL_JOINTS]
            self.joints_at = time.monotonic()

    def on_firmware(self, msg):
        self.firmware = json.loads(msg.data)
        self.firmware_at = time.monotonic()

    def wait(self, future, timeout=15):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('ROS operation timed out')
        return future.result()

    def call(self, cls, name, request, timeout=15):
        client = self.service_clients.get(name)
        if client is None:
            client = self.service_clients[name] = self.create_client(cls, name)
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(name + ' is unavailable')
        return self.wait(client.call_async(request), timeout)

    def current(self):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.joints and time.monotonic() - self.joints_at < 0.5:
                return self.joints.copy()
        raise RuntimeError('No fresh joint state')

    @staticmethod
    def state(q):
        return RobotState(joint_state=JointState(name=ALL_JOINTS, position=list(q)), is_diff=True)

    def fk(self, q):
        req = GetPositionFK.Request(robot_state=self.state(q), fk_link_names=[TOOL])
        req.header.frame_id = 'base_link'
        result = self.call(GetPositionFK, '/compute_fk', req)
        if result.error_code.val != 1:
            raise RuntimeError('Forward kinematics failed')
        return result.pose_stamped[0]

    def floor(self, table_z):
        # Only the fixed base may overlap its support; every moving link is checked.
        req = GetPlanningScene.Request(components=PlanningSceneComponents(components=128))
        acm = self.call(GetPlanningScene, '/get_planning_scene', req).scene.allowed_collision_matrix
        for name in ['base_link', FLOOR_ID]:
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        base = acm.entry_names.index('base_link')
        floor = acm.entry_names.index(FLOOR_ID)
        # Never retain an old permission for a moving link to cross this floor.
        for i in range(len(acm.entry_names)):
            acm.entry_values[i].enabled[floor] = i == base
            acm.entry_values[floor].enabled[i] = i == base
        obj = CollisionObject(id=FLOOR_ID, operation=CollisionObject.ADD)
        obj.header.frame_id = 'base_link'
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 1.0])]
        pose = Pose(); pose.orientation.w = 1.0
        pose.position.z = table_z + 0.02 - 0.5
        obj.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obj]
        if not self.call(ApplyPlanningScene, '/apply_planning_scene', ApplyPlanningScene.Request(scene=scene)).success:
            raise RuntimeError('Table collision object was not accepted')

    def valid(self, q, constraints=None):
        req = GetStateValidity.Request(robot_state=self.state(q), group_name='')
        if constraints is not None:
            req.constraints = constraints
        result = self.call(GetStateValidity, '/check_state_validity', req)
        if not result.valid:
            contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
            raise RuntimeError('State violates collision/path constraints: ' + str(contacts))

    def firmware_healthy(self, q=None):
        f = self.firmware
        if (not f or time.monotonic()-self.firmware_at > 0.5 or f.get('status_age_s') is None
                or f['status_age_s'] > 3.0):
            raise RuntimeError('Firmware status missing or stale')
        if f.get('armed') != 1 or f.get('hold') != 0 or f.get('driver_fault') != 0 or f.get('wifi') != 'connected':
            raise RuntimeError('Firmware is not armed and healthy')
        if f.get('rx_count', 0) == 0 or f.get('last_rx_ms_ago', 99999) > 800:
            raise RuntimeError('Firmware is not receiving regular UDP commands')
        if q is not None:
            expected = servo_degrees(q)
            for key in ['current_deg', 'target_deg']:
                if len(f.get(key, [])) != 5 or max(abs(a-b) for a,b in zip(expected, f[key])) > 0.7:
                    raise RuntimeError('Host and firmware command positions disagree')
        return f

    def hold(self):
        return self.call(Trigger, '/roboflex/hold', Trigger.Request(), timeout=3)

    def execute(self, trajectory, start, end, duration):
        actual = self.current()
        if max(abs(a-b) for a,b in zip(actual, start)) > 0.002:
            raise RuntimeError('Start state changed during planning; replan')
        before = self.firmware_healthy(start)['rx_count']
        if not self.action.wait_for_server(timeout_sec=5):
            raise RuntimeError('MoveIt execution action unavailable')
        goal = ExecuteTrajectory.Goal(trajectory=trajectory, controller_names=['arm_controller'])
        self.goal_handle = self.wait(self.action.send_goal_async(goal))
        if not self.goal_handle.accepted:
            raise RuntimeError('MoveIt rejected execution')
        result = self.goal_handle.get_result_async()
        deadline = time.monotonic()+duration+8
        try:
            while not result.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                self.firmware_healthy()
                if time.monotonic()>deadline:
                    raise RuntimeError('Execution timeout')
            if result.result().result.error_code.val != 1:
                raise RuntimeError('MoveIt execution failed: '+str(result.result().result.error_code.val))
            # Wait for a post-execution serial status, not merely mirrored ROS state.
            deadline = time.monotonic()+5
            last_error = None
            while time.monotonic()<deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                try:
                    f = self.firmware_healthy(end)
                    if f['rx_count'] > before:
                        self.goal_handle = None
                        return f
                except RuntimeError as exc:
                    last_error = exc
            raise RuntimeError('Firmware did not confirm final command: '+str(last_error))
        except BaseException:
            try:
                self.hold()
            finally:
                self.wait(self.goal_handle.cancel_goal_async(), timeout=3)
                self.goal_handle = None
            raise


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--base-offset-deg', type=float, help='Relative base turn, keeping other arm joints fixed')
    mode.add_argument('--offset', nargs=3, type=float, metavar=('DX','DY','DZ'), help='Tool offset in base_link metres')
    mode.add_argument('--named-pose', choices=['Arm Up'], help='Return to the confirmed reference if within the small-step envelope')
    parser.add_argument('--table-z', type=float, required=True, help='Tabletop z in base_link metres; use 0 only when table is at/below mounting level')
    parser.add_argument('--execute', action='store_true', help='Send the checked plan; requires an already armed firmware monitor')
    parser.add_argument('--report', type=Path, help='Save the plan/check result as JSON')
    args = parser.parse_args()
    values = [args.table_z] + ([args.base_offset_deg] if args.base_offset_deg is not None else []) + (args.offset or [])
    if not all(math.isfinite(v) for v in values):
        parser.error('All inputs must be finite')
    if not -0.5 <= args.table_z <= 0.05:
        parser.error('table-z must be between -0.5 and 0.05 m')
    if args.base_offset_deg is not None and not 0 < abs(args.base_offset_deg) <= 10:
        parser.error('Base trials are limited to nonzero offsets of at most 10 degrees')
    if args.offset and (math.sqrt(sum(v*v for v in args.offset)) > 0.02 or args.offset[2] < 0):
        parser.error('Tool trials allow at most 2 cm and no downward goal')
    return args


def run(node, args):
    description = Path(get_package_share_directory('roboflex_description'))/'urdf/real_hardware.urdf.xacro'
    robot = ET.fromstring(xacro.process_file(str(description)).toxml())
    bounds = {j.attrib['name']: (float(j.find('limit').attrib['lower']), float(j.find('limit').attrib['upper']))
              for j in robot.findall('joint') if j.attrib['name'] in ALL_JOINTS}
    start = node.current()
    # Gripper geometry/direction needs separate calibration; keep it closed for these trials.
    if abs(start[4]) > 0.002:
        raise RuntimeError('Raised-arm trials require the confirmed closed gripper reference')
    node.floor(args.table_z)
    node.valid(start)
    initial_pose = node.fk(start)
    minimum_z = max(initial_pose.pose.position.z - 0.001, args.table_z + 0.08)
    goal = start.copy()
    if args.base_offset_deg is not None:
        goal[0] += math.radians(args.base_offset_deg)
    elif args.named_pose:
        srdf = ET.parse(Path(get_package_share_directory('roboflex_moveit'))/'config/roboflex_description.srdf')
        pose = next(p for p in srdf.findall('group_state') if p.attrib.get('group')=='arm' and p.attrib['name']==args.named_pose)
        named = {j.attrib['name']:float(j.attrib['value']) for j in pose.findall('joint')}
        goal[:4] = [named[n] for n in JOINTS]
    else:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name='arm'; ik.robot_state=node.state(start); ik.ik_link_name=TOOL
        ik.pose_stamped=copy.deepcopy(initial_pose); ik.avoid_collisions=True; ik.timeout.sec=2
        for axis, delta in zip(['x','y','z'], args.offset):
            setattr(ik.pose_stamped.pose.position, axis, getattr(ik.pose_stamped.pose.position, axis)+delta)
        result=node.call(GetPositionIK,'/compute_ik',request)
        if result.error_code.val != 1:
            raise RuntimeError('No collision-free IK solution: '+str(result.error_code.val))
        values=dict(zip(result.solution.joint_state.name,result.solution.joint_state.position))
        goal[:4]=[values[n] for n in JOINTS]
    for name,a,b in zip(ALL_JOINTS,start,goal):
        if not bounds[name][0] <= b <= bounds[name][1] or abs(b-a) > math.radians(10):
            raise RuntimeError(name+': goal exceeds joint bounds or the 10-degree trial envelope')
    if node.fk(goal).pose.position.z < minimum_z:
        raise RuntimeError('Goal would lower the tool below the starting-height floor')
    # Position constraint preserves raised tool height along the entire planned path.
    constraints=Constraints()
    pc=PositionConstraint(link_name=TOOL,weight=1.0)
    pc.header.frame_id='base_link'
    pc.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[1.6,1.6,1.0])]
    region=Pose();region.orientation.w=1.;region.position.z=minimum_z+.5
    pc.constraint_region.primitive_poses=[region]
    constraints.position_constraints=[pc]
    for i,name in enumerate(JOINTS):
        # The first test rotates only the vertical base axis.
        margin=0.0001 if args.base_offset_deg is not None and i>0 else math.radians(1)
        low=min(start[i],goal[i])-margin;high=max(start[i],goal[i])+margin
        constraints.joint_constraints.append(JointConstraint(joint_name=name,position=(low+high)/2,
            tolerance_above=(high-low)/2,tolerance_below=(high-low)/2,weight=1.0))
    node.valid(start,constraints);node.valid(goal,constraints)
    request=GetMotionPlan.Request(); req=request.motion_plan_request
    req.group_name='arm';req.pipeline_id='ompl';req.planner_id='RRTConnectkConfigDefault'
    req.start_state=node.state(start);req.allowed_planning_time=5.;req.num_planning_attempts=3
    req.max_velocity_scaling_factor=.25;req.max_acceleration_scaling_factor=.25
    req.path_constraints=constraints
    req.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=n,position=q,
        tolerance_above=.00005,tolerance_below=.00005,weight=1.) for n,q in zip(JOINTS,goal)])]
    result=node.call(GetMotionPlan,'/plan_kinematic_path',request,timeout=20).motion_plan_response
    if result.error_code.val != 1:
        raise RuntimeError('MoveIt planning failed: '+str(result.error_code.val))
    trajectory=result.trajectory.joint_trajectory
    if trajectory.joint_names != JOINTS:
        raise RuntimeError('Unexpected trajectory joint order')
    # Leave headroom for extrema between time-parameterized waypoints. The
    # controller uses their derivatives to interpolate a quintic spline.
    for point in trajectory.points:
        seconds=point.time_from_start.sec+point.time_from_start.nanosec/1e9
        nanoseconds=round(seconds*1.1*1e9)
        point.time_from_start.sec,point.time_from_start.nanosec=divmod(nanoseconds,10**9)
        point.velocities=[v/1.1 for v in point.velocities]
        point.accelerations=[a/(1.1**2) for a in point.accelerations]
    if max(abs(a-b) for a,b in zip(trajectory.points[0].positions,start[:4])) > .002:
        raise RuntimeError('Planner changed the requested start state')
    duration,samples=trajectory_samples(trajectory)
    minimum_actual_z=float('inf');peak_v=0.;peak_a=0.
    for _,positions,velocities,accelerations in samples:
        q=list(map(float,positions))+[start[4]]
        if any(not bounds[n][0]-1e-6 <= v <= bounds[n][1]+1e-6 for n,v in zip(JOINTS,q)):
            raise RuntimeError('Interpolated trajectory exceeds a joint position bound')
        if args.base_offset_deg is not None and max(abs(q[i]-start[i]) for i in [1,2,3]) > .0002:
            raise RuntimeError('Base trial unexpectedly moves shoulder/elbow/wrist')
        peak_v=max(peak_v,max(map(abs,velocities)));peak_a=max(peak_a,max(map(abs,accelerations)))
        if peak_v>.050001 or peak_a>.100001:
            raise RuntimeError('Interpolated trajectory exceeds reduced speed/acceleration limits')
        node.valid(q,constraints)
        minimum_actual_z=min(minimum_actual_z,node.fk(q).pose.position.z)
    end=list(trajectory.points[-1].positions)+[start[4]]
    if max(abs(a-b) for a,b in zip(end,goal)) > .001:
        raise RuntimeError('Trajectory endpoint does not match the goal')
    report=dict(start_rad=start,goal_rad=end,table_z_m=args.table_z,collision_floor_top_m=args.table_z+.02,
                start_tool_z_m=initial_pose.pose.position.z,minimum_tool_z_m=minimum_actual_z,
                duration_s=duration,checked_spline_samples=len(samples),peak_velocity_rad_s=peak_v,
                peak_acceleration_rad_s2=peak_a,executed=False,physical_position_verified=False)
    print(json.dumps(report,indent=2),flush=True)
    if args.execute:
        node.get_logger().info('Sending checked trajectory through MoveIt ExecuteTrajectory')
        report['firmware_final']=node.execute(result.trajectory,start,end,duration)
        report['executed']=True
        print('MoveIt execution succeeded; firmware confirmed command angles. Physical observation is still required.',flush=True)
    if args.report:
        args.report.write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    args=arguments();rclpy.init();node=GoalClient()
    try:
        run(node,args)
    except (RuntimeError,ValueError) as exc:
        print('REFUSED/FAILED: '+str(exc),file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0


if __name__=='__main__':
    sys.exit(main())
