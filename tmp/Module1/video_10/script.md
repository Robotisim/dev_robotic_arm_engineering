# M1_V10_Gazebo_Bringup_From_Model_to_Simulation

## RViz vs Gazebo

**RViz:**
- A visualiser. Subscribes to topics and draws the robot as specified by the URDF.
- No physics.
- Can show sensor data.

**Gazebo:**
- Physics simulator.
- Applies gravity, collision dynamics, friction and joint actuation.

## Gazebo

### Simulation

The environment we will be simulating are in the form of SDF files. It is
similar to a URDF file in the sense that each object is defined with its visual
and physical properties. For instance a ground plane that would have friction so
objects don't slip on it, or the sun as a directional light source.

### Spawn Entity

#### Via topic (stick_arm)

`stick_arm_gz_control.launch.py:77-94`:

```python
spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    name='spawn_stick_arm',
    arguments=[
        '-name', 'stick_arm',
        '-topic', 'robot_description',   # <-- reads from topic
        '-x', '0.0', '-y', '0.0', '-z', '0.15',
    ],
)
```

Sequence:

1. `robot_state_publisher` publishes the xacro-processed URDF XML to the
   `/robot_description` topic.
2. The `ros_gz_sim create` node subscribes to that topic, retrieves the XML,
   and tells Gazebo: "create a model named `stick_arm` using this URDF at pose
   (0, 0, 0.15)."

#### Via inline string (panda)

`panda_sim.launch.py:54-58` and `panda_sim_control.launch.py:63-68`:

```python
doc = xacro.parse(open(xacro_file))
xacro.process_doc(doc, mappings={'controllers_file': controllers_file})

spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=['-string', doc.toxml(), '-name', 'Panda', '-allow_renaming', 'true'],
)
```

Sequence:

1. The launch file itself parses the xacro URDF using the `xacro` Python
   library (lines 50-52 of `panda_sim.launch.py`).
2. The processed XML string is passed inline via `-string` — no topic involved.
3. Gazebo receives the complete model definition at startup.

#### The key difference

- **Stick arm**: `robot_description` is published first by
  `robot_state_publisher`, then `create` picks it up from the topic. This is
  the more standard ROS 2 pattern — decouples the model source from the
  spawner.
- **Panda**: the launch file owns the parsing and hands the string directly.
  No topic dependency. Simpler, but means the model XML is duplicated in the
  launch file's memory rather than flowing through a topic.

## Gazebo Resource Paths

Gazebo needs to know where to find model files, mesh files, and world files.
It uses an environment variable called `GZ_SIM_RESOURCE_PATH` to search for
them.

### Automatic: environment hook (passive)

`robotic_arm_sim/cmake/environment_hooks/gz_sim_resource_path.bash` runs
automatically when you `source install/setup.bash`. It does:

```bash
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "${AMENT_CURRENT_PREFIX}/share/robotic_arm_sim"
ament_prepend_unique_value GZ_SIM_RESOURCE_PATH "${AMENT_CURRENT_PREFIX}/share/robotic_arm_sim/models"
```

This means anytime Gazebo looks for a `model://table` or `model://ground_plane`,
it searches:

1. `<workspace>/install/robotic_arm_sim/share/robotic_arm_sim/models/table/`
   — finds `model.sdf`
2. `<workspace>/install/robotic_arm_sim/share/robotic_arm_sim/`
   — finds `worlds/empty.world`

The `IGN_GAZEBO_RESOURCE_PATH` variant is a backward-compatibility duplicate
for older Gazebo tooling (pre-harmonic naming).

### Manual: launch file SetEnvironmentVariable (active)

The Panda launch files (`panda_sim.launch.py`, `panda_sim_control.launch.py`)
also set these variables explicitly inside the launch process:

```python
resource_paths = [
    sim_share_path,                            # install/robotic_arm_sim/share/robotic_arm_sim
    os.path.join(sim_share_path, 'models'),    # install/.../models
    existing_gz_resource_path,                 # preserve whatever was already set
]
set_gz_resource_path = SetEnvironmentVariable(
    name='GZ_SIM_RESOURCE_PATH',
    value=os.pathsep.join(dict.fromkeys(resource_paths)),  # deduplicated
)
```

This ensures the sub-processes (gz_server, gz_gui) inherit the correct paths
even if the hook wasn't sourced.

### What Gazebo uses the paths for

| Operation | Example | How it resolves |
|---|---|---|
| `<include><uri>model://table</uri></include>` in world SDF | `pick_and_place_cubes_base.sdf` | Gazebo searches each `GZ_SIM_RESOURCE_PATH` entry for a `table/` directory containing `model.sdf` |
| `model://ground_plane` | `empty.world` | Same — finds `models/ground_plane/model.sdf` |
| Mesh references in URDF/SDF | Panda's `.stl` or `.dae` files | Gazebo searches `GZ_SIM_RESOURCE_PATH` + `SDF_PATH` for relative mesh paths |
| World files | `empty.sdf` as `gz_args` | Gazebo checks `GZ_SIM_RESOURCE_PATH` for `worlds/empty.sdf` |

### Stick arm vs Panda

- **Stick arm** doesn't need the explicit `SetEnvironmentVariable` in its
  launch file because it uses primitive geometry (boxes, cylinders) defined
  inline in the URDF — no external meshes to resolve.
- **Panda** explicitly sets the path because its URDF references mesh files
  (`.stl`/`.dae`) for visual and collision geometry that live in
  `robotic_arm_description/meshes/panda/`.

### Takeaway

Gazebo resource paths tell the simulator where to find model files. The
environment hook ensures every terminal gets them automatically; the launch
files re-assert them explicitly so spawned child processes never lose them.
Without these paths, `model://table` fails, the world loads as a grey void,
and the Panda appears invisible.

## Simulation Time

ROS 2 has two time sources:

- **Wall clock** — real system time (`/use_sim_time := false`). Used when
  working with a real robot.
- **Simulation time** (`/use_sim_time := true`). All nodes subscribe to the
  `/clock` topic and use its timestamp instead of the system clock.

In this repo, `sim_robot.launch.py` sets `use_sim_time:=true` by default.

### How the clock flows

```
Gazebo physics engine
    │
    │ publishes /clock at each simulation step
    │
    ▼
clock_bridge (ros_gz_bridge parameter_bridge)
    │
    │ bridges /clock from Gazebo format to ROS 2 format
    │ argument: '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    │                                    direction ──┘
    │                                    [ = Gazebo → ROS
    ▼
robot_state_publisher, RViz, controllers, etc.
    │ each node reads /clock instead of wall clock
    │ parameter: use_sim_time = true
    ▼
All timestamps (tf, joint_states, sensor_msgs) are in simulation time
```

### Three parts in the launch files

1. **The `-r` flag** in `gz_args`: `'-r -v 3 empty.sdf'`
   - `-r` = start simulation running immediately (don't pause on launch)
   - `-v 3` = verbosity level

2. **The clock bridge** (`stick_arm_gz_control.launch.py:69-75`):

   ```python
   clock_bridge = Node(
       package='ros_gz_bridge',
       executable='parameter_bridge',
       arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
   )
   ```

   The `[` direction arrow means the bridge subscribes to the Gazebo-side
   `/clock` topic and republishes it as a ROS 2 `rosgraph_msgs/msg/Clock`
   message. Without this bridge, `/clock` never appears on the ROS 2 side
   and `use_sim_time=true` nodes would hang waiting for time.

3. **`use_sim_time` parameter** on every node:
   - `stick_arm_gz_control.launch.py:64` — hardcoded `True`
   - The Panda launch files — passed through from `LaunchConfiguration`

### Why it matters

If `use_sim_time` is `false` but Gazebo is running:

- TF timestamps disagree with wall clock
- Controllers' PID loops drift
- Sensor timestamps are meaningless

If `use_sim_time` is `true` but the clock bridge is missing:

- `/clock` never publishes
- Every node blocks waiting for time — nothing moves

### How the stick arm and Panda differ

- **Stick arm**: has a dedicated `clock_bridge` node, and `use_sim_time` is
  hardcoded to `True` in the launch file.
- **Panda**: the clock bridge is bundled in the `wrist_eye_bridge` node
  (`'/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'` is the first argument),
  and `use_sim_time` is a `LaunchConfiguration` parameter (defaults `true`
  from `sim_robot.launch.py`).

### Takeaway

Simulation time keeps every ROS 2 node synchronized with Gazebo's physics
clock. The bridge converts Gazebo's internal time into a ROS 2 `/clock` topic,
and the `use_sim_time` parameter tells each node to use it. Without this chain,
simulation either hangs or runs asynchronously and breaks.
