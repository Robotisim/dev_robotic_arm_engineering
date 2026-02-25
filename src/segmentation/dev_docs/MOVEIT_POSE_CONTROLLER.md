# MoveIt2 Pose Controller Node

## Overview

The `moveit_pose_controller_node` is a C++ ROS2 node that provides an interface to control the robot's end effector pose using MoveIt2's move group action server.

## Features

- **Service Interface**: Call the `execute_pose_goal` service to send poses to MoveIt2
- **Configurable Parameters**: Set goal poses, planning parameters, and MoveIt2 settings via ROS2 parameters  
- **Action Feedback**: Real-time feedback on planning and execution status
- **Error Handling**: Comprehensive error reporting for debugging

## Building

```bash
cd /home/darth-yoseph/ros2_ws
colcon build --packages-select cube_segmentation
source install/setup.bash
```

## Usage

### 1. Basic Launch (with default pose)

```bash
ros2 launch cube_segmentation moveit_pose_controller.launch.py
```

### 2. Launch with Custom Goal Pose

```bash
ros2 launch cube_segmentation moveit_pose_controller.launch.py \
    goal_x:=0.5 \
    goal_y:=0.2 \
    goal_z:=0.3 \
    goal_qw:=1.0
```

### 3. Trigger Goal Execution via Service

After launching the node, trigger the goal execution:

```bash
ros2 service call /execute_pose_goal std_srvs/srv/Trigger
```

### 4. Update Goal Pose at Runtime

You can update the goal pose parameters while the node is running:

```bash
ros2 param set /moveit_pose_controller goal_x 0.6
ros2 param set /moveit_pose_controller goal_y -0.1
ros2 param set /moveit_pose_controller goal_z 0.4
```

Then trigger execution again:

```bash
ros2 service call /execute_pose_goal std_srvs/srv/Trigger
```

## Parameters

### MoveIt Configuration
- `move_action_name` (string, default: "/move_action"): MoveIt move group action server name
- `planning_group` (string, default: "panda_arm"): MoveIt planning group name
- `planning_frame` (string, default: "panda_link0"): Planning reference frame
- `end_effector_link` (string, default: "panda_link8"): End effector link name

### Goal Pose
- `goal_x`, `goal_y`, `goal_z` (double): Position in meters
- `goal_qx`, `goal_qy`, `goal_qz`, `goal_qw` (double): Orientation quaternion

### Planning Parameters
- `allowed_planning_time` (double, default: 5.0): Maximum planning time in seconds
- `num_planning_attempts` (int, default: 5): Number of planning attempts
- `max_velocity_scaling` (double, default: 0.2): Velocity scaling factor (0.0-1.0)
- `max_acceleration_scaling` (double, default: 0.2): Acceleration scaling factor (0.0-1.0)
- `planner_id` (string, default: "RRTConnectkConfigDefault"): MoveIt planner to use
- `plan_only` (bool, default: false): If true, only plan without executing

## Services

### `/execute_pose_goal` (std_srvs/srv/Trigger)
Triggers execution of the goal pose specified in the parameters.

**Request**: Empty  
**Response**:
- `success` (bool): True if goal was sent successfully
- `message` (string): Status message

## Example: Complete Workflow

1. Launch MoveIt2 with your robot (e.g., Panda simulation):
```bash
# In terminal 1
ros2 launch panda panda_sim_moveit.launch.py
```

2. Launch the pose controller:
```bash
# In terminal 2
ros2 launch cube_segmentation moveit_pose_controller.launch.py \
    goal_x:=0.4 goal_y:=-0.1 goal_z:=0.5
```

3. Execute the goal:
```bash
# In terminal 3
ros2 service call /execute_pose_goal std_srvs/srv/Trigger
```

4. Monitor the node output in terminal 2 for planning and execution feedback.

## Integration with Other Nodes

You can call the service from other nodes in your code:

### Python Example
```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.client = self.create_client(Trigger, '/execute_pose_goal')
        
    def send_goal(self):
        # Set parameters first via param client if needed
        # Then call service
        request = Trigger.Request()
        future = self.client.call_async(request)
        return future
```

### C++ Example
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

auto client = node->create_client<std_srvs::srv::Trigger>("/execute_pose_goal");
auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
auto future = client->async_send_request(request);
```

## Troubleshooting

### "MoveIt action server not available"
- Make sure MoveIt2 is running with the correct configuration
- Check that the action name parameter matches your MoveIt setup

### "Goal was rejected by server"
- The goal pose may be unreachable or in collision
- Try adjusting planning parameters or the goal pose
- Check MoveIt logs for more details

### "Planning failed"
- Increase `allowed_planning_time` or `num_planning_attempts`
- Try a different planner via `planner_id` parameter
- Verify the goal pose is within the robot's workspace

## Node Architecture

```
moveit_pose_controller_node
    ├── Service: /execute_pose_goal
    │   └── Triggers goal execution with current parameters
    │
    ├── Action Client: /move_action
    │   └── Sends MoveGroup goals to MoveIt2
    │
    └── Parameters
        ├── Robot configuration (frame names, planning group)
        ├── Goal pose (position + orientation)
        └── Planning settings (time, attempts, scaling)
```

## Future Enhancements

Potential improvements you could add:
- Add a topic subscriber to receive goal poses from other nodes
- Add an action server interface for more complex goal specifications
- Support for multiple waypoint trajectories
- Visualization of goal poses in RViz
- Path constraint specifications
