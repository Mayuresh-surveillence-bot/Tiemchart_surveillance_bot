# Surveillance Robot

A mobile surveillance robot capable of autonomous navigation and patrol inside an
environment. This repository contains the **simulation and ROS 2 software stack**.

## Current Status

**Chunks 1–4 — IMPLEMENTED; Chunk 4 NOT RUNTIME VERIFIED**

The workspace includes the modular robot description, structural TF, RViz,
Gazebo Harmonic simulation, differential-drive ros2_control, and simulated
LiDAR, camera, and IMU integration. Chunk 4 completes the existing partial sensor
configuration without replacing the robot, mounting frames, or drive stack.
SLAM, Nav2, mapping, localization, patrol, obstacle avoidance, and computer
vision are not implemented or launched by this work.

## Architecture

The stack is organized into functional-module packages:

| Package | Build type | Responsibility |
| --- | --- | --- |
| `surveillance_robot_description` | `ament_cmake` | Robot model: URDF/Xacro, meshes, RViz, frames |
| `surveillance_robot_gazebo` | `ament_cmake` | Gazebo simulation: worlds, models, plugins |
| `surveillance_robot_bringup` | `ament_cmake` | System-level launch and configuration |
| `surveillance_robot_navigation` | `ament_cmake` | Mapping, localization, Nav2 config, maps |
| `surveillance_robot_patrol` | `ament_python` | Custom patrol / mission / waypoint behavior |
| `surveillance_robot_interfaces` | `ament_cmake` | Custom msg/srv/action definitions |
| `surveillance_robot_tools` | `ament_python` | Diagnostic / telemetry utilities |

`ament_cmake` is used for packages that only install resources (URDF, worlds, launch,
config) and for `interfaces` because ROS 2 message generation (`rosidl`) requires
CMake. `ament_python` is used for the two packages that will contain `rclpy` nodes.

## Requirements

- Ubuntu 24.04
- ROS 2 **Jazzy Jalisco**
- **Gazebo Harmonic**, Jazzy `ros_gz`, and `gz_ros2_control`
- An Ogre2-compatible graphics environment for GPU LiDAR and camera rendering
- Python 3.12
- `colcon` (`sudo apt install python3-colcon-common-extensions`)

## Workspace Setup

```bash
# From the existing botx-agent checkout; do not create a nested workspace.
cd surveillance_robot_ws

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Install dependencies (from surveillance_robot_ws/)
rosdep install --from-paths src --ignore-src -r -y
```

## Build

From `surveillance_robot_ws/`:

```bash
colcon build
source install/setup.bash
```

## Package List

- **surveillance_robot_description** — modular URDF/Xacro model, visual/collision/
  inertial properties, sensor frames, and RViz robot/TF/LiDAR/image displays.
- **surveillance_robot_gazebo** — test world, control integration, sensor Xacro,
  sensor parameters/bridges, simulation launch, and static sensor contract tests.
- **surveillance_robot_bringup** — existing `simulation_bringup.launch.py` composes
  the simulation without duplicating nodes or starting navigation/patrol.
- **surveillance_robot_navigation** — will hold SLAM Toolbox, localization, and
  Nav2 parameters, maps, and navigation launch files.
- **surveillance_robot_patrol** — will hold patrol behavior nodes such as
  `patrol_manager`, `mission_manager`, and `waypoint_manager`.
- **surveillance_robot_interfaces** — will hold custom ROS 2 interfaces
  (e.g. `PatrolStatus.msg`, `SurveillanceEvent.msg`, `PatrolMission.action`).
- **surveillance_robot_tools** — will hold development/diagnostic utilities such
  as diagnostic and telemetry nodes.

## Development Roadmap

- Chunk 1 — Repository (implemented)
- Chunk 2 — Robot Description (implemented)
- Chunk 3 — Gazebo + Drive (existing implementation retained)
- **Chunk 4 — Sensors** (implemented; NOT RUNTIME VERIFIED)
- Chunk 5 — Environment
- Chunk 6 — SLAM
- Chunk 7 — Navigation
- Chunk 8 — Patrol
- Chunk 9 — Surveillance Features
- Chunk 10 — Integration

## Chunk 4 — Sensor Integration

### Existing simulation entry point

After building and sourcing the workspace on Ubuntu 24.04 + ROS 2 Jazzy:

```bash
ros2 launch surveillance_robot_bringup simulation_bringup.launch.py use_rviz:=true
```

The existing Gazebo launch owns the world, robot spawn, robot_state_publisher,
clock bridge, controller spawners, and one additional `sensor_bridge` node.
The bridge loads `surveillance_robot_gazebo/config/sensor_bridges.yaml`; all four
entries are Gazebo-to-ROS only, non-lazy, and use `SENSOR_DATA` QoS (best effort,
volatile). Gazebo supplies simulation timestamps; there is no wall-time or frame
ID override in the bridge. Gazebo's Ogre2 Sensors system runs the GPU LiDAR and
camera, and its separate Imu system runs the IMU. No Gazebo Classic plugins are used.

### Implemented interfaces

| Sensor | ROS topic | Message type | `header.frame_id` |
| --- | --- | --- | --- |
| LiDAR | `/scan` | `sensor_msgs/msg/LaserScan` | `laser_link` |
| Camera | `/camera/image_raw` | `sensor_msgs/msg/Image` | `camera_optical_frame` |
| Camera | `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_optical_frame` |
| IMU | `/imu/data` | `sensor_msgs/msg/Imu` | `imu_link` |

Gazebo Transport uses the same topic names, with `gz.msgs.LaserScan`,
`gz.msgs.Image`, `gz.msgs.CameraInfo`, and `gz.msgs.IMU`, respectively.
These are configured interfaces, not a claim that message publication was
observed in this environment.

Parameters live in `surveillance_robot_gazebo/config/sensors.yaml`:

- **LiDAR:** 10 Hz, 720 planar samples, 0.5-degree increments from -180 to
  +179.5 degrees (one full revolution without a duplicate endpoint), 0.12–12 m
  range, 0.01 m range resolution and Gaussian standard deviation. The scan lies
  in `laser_link` XY, with zero angle along robot-forward +X and positive angles
  toward +Y (left).
- **Camera:** RGB 640×480 at 15 Hz, 80-degree horizontal FOV, 0.05–20 m clipping.
  Images are noise-free for initial transport/frame inspection.
- **IMU:** 50 Hz; orientation uses an ENU reference, with three-axis angular
  velocity and linear acceleration. Gaussian standard deviations are 0.002 rad/s
  and 0.02 m/s². The already-existing `imu_link` justifies this sensor; no new
  mounting design was added. Orientation is simulated, not a localization output.

### TF and visualization

The existing mounting positions in `dimensions.xacro` are unchanged:

```text
odom                            (existing diff_drive_controller TF)
└── base_footprint
    └── base_link
        ├── laser_link          (+X forward, +Y left, +Z up)
        ├── camera_link         (+X forward, +Y left, +Z up)
        │   └── camera_optical_frame  (+Z forward, +X right, +Y down)
        └── imu_link            (+X forward, +Y left, +Z up)
```

The optical joint rotates by roll=-π/2, pitch=0, yaw=-π/2. The Gazebo camera stays
unrotated on `camera_link` because Gazebo renders along +X; both camera messages
identify the optical child frame. Robot State Publisher remains the sole sensor
TF publisher. No duplicate frames, static TF nodes, or `map → odom` were added.
The existing sensor mounting joints are preserved during URDF-to-SDF conversion.

RViz's existing configuration now includes `/scan` and `/camera/image_raw`, both
with best-effort subscriptions. Its fixed frame remains `base_footprint` for the
existing robot-centric view; it does not compensate for sensor rotations. The
LiDAR display uses zero decay to avoid trails in that moving fixed frame.
Description-only launch still creates no sensors; its sensor displays will have
no data and can be disabled. No extra simulation launch is required.

### Validation status

**IMPLEMENTED — NOT RUNTIME VERIFIED.** The static test suite checks XML/SDF
well-formedness, launch Python syntax, YAML, real Xacro expansion in both
simulation and description-only modes, unique connected sensor TF, sensor
parameters, topic/message/frame agreement, bridge launch wiring, RViz QoS,
world plugins, declared dependencies, and installation resource paths.
The non-ROS check substitutes source-directory lookup for Xacro's package-share
resolver only; it does not validate the installed ament index or SDF conversion.

The test registered by the existing `ament_cmake_pytest` rule can run after a ROS
workspace build:

```bash
colcon test --packages-select surveillance_robot_gazebo
colcon test-result --verbose
```

ROS 2, Gazebo, colcon, and RViz are unavailable in v0. Consequently no ROS build,
SDF conversion, sensor publication/rate/content measurement, graphics rendering,
TF runtime lookup, or drive regression was performed here. Source checks cannot
establish that a sensor produces physically meaningful measurements.

### Required Ubuntu/Jazzy runtime checks

With the simulation running, verify publisher types, QoS, and real frame IDs:

```bash
ros2 topic info /scan --verbose
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /camera/image_raw --once --field header --qos-reliability best_effort
ros2 topic echo /camera/camera_info --once --qos-reliability best_effort
ros2 topic echo /imu/data --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
ros2 run tf2_ros tf2_echo base_link imu_link
```

Check approximate simulation-time rates of 10/15/50 Hz (wall-time rates fall when
Gazebo runs below real time). Inspect forward-wall LiDAR returns, camera contents
and matching intrinsics, and stationary IMU gravity/orientation before moving.
For an upright robot at its initial XY/yaw, the front wall's near face is roughly
2.75 m ahead of the LiDAR; out-of-range directions may correctly return infinity.
Check that scan/camera poses follow the robot, yaw-rate changes follow turning,
and that the existing `/cmd_vel`, wheel joints, odometry, and TF still work.

**Pre-existing Chunk 3 issue to check, not changed here:** the controller launch
remaps `~/cmd_vel` to `/cmd_vel` but does not remap `~/odom`. Jazzy's default
odometry topic is therefore expected to be `/diff_drive_controller/odom`, not
`/odom`, despite the older controller comments. Confirm with `ros2 topic list`;
this sensor-only change deliberately does not alter controller remappings.
The existing `OnProcessExit` controller chain also does not check exit codes, so
inspect spawn/controller logs rather than treating process exit as success.

## Maintainer

Replace the placeholder maintainer in every `package.xml` (and in the `setup.py`
of the Python packages) with your real name and email:

```xml
<maintainer email="your_email@example.com">Your Name</maintainer>
```
