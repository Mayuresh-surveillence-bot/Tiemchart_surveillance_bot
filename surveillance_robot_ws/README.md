# Surveillance Robot

A mobile surveillance robot capable of autonomous navigation and patrol inside an
environment. This repository contains the **simulation and ROS 2 software stack**.

## Current Status

**Chunk 1 — Repository Initialization**

Only the workspace and package skeleton exists. No robot description, simulation,
SLAM, navigation, or patrol logic is implemented yet.

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
- Python 3.12
- `colcon` (`sudo apt install python3-colcon-common-extensions`)

## Workspace Setup

```bash
# Clone the repository into the workspace src/ directory
mkdir -p surveillance_robot_ws/src
cd surveillance_robot_ws/src
git clone <your-repo-url> surveillance_robot
cd ..

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

- **surveillance_robot_description** — will hold the robot's URDF/Xacro model,
  meshes, visual/collision/inertial properties, sensor frames, and RViz config.
- **surveillance_robot_gazebo** — will hold the surveillance world, Gazebo models,
  plugins, and simulation configuration.
- **surveillance_robot_bringup** — will provide top-level launch files
  (`bringup`, `simulation`, `navigation`, `patrol`) that compose the system.
- **surveillance_robot_navigation** — will hold SLAM Toolbox, localization, and
  Nav2 parameters, maps, and navigation launch files.
- **surveillance_robot_patrol** — will hold patrol behavior nodes such as
  `patrol_manager`, `mission_manager`, and `waypoint_manager`.
- **surveillance_robot_interfaces** — will hold custom ROS 2 interfaces
  (e.g. `PatrolStatus.msg`, `SurveillanceEvent.msg`, `PatrolMission.action`).
- **surveillance_robot_tools** — will hold development/diagnostic utilities such
  as diagnostic and telemetry nodes.

## Development Roadmap

- **Chunk 1 — Repository** (current)
- Chunk 2 — Robot Description
- Chunk 3 — Gazebo + Drive
- Chunk 4 — Sensors
- Chunk 5 — Environment
- Chunk 6 — SLAM
- Chunk 7 — Navigation
- Chunk 8 — Patrol
- Chunk 9 — Surveillance Features
- Chunk 10 — Integration

## Maintainer

Replace the placeholder maintainer in every `package.xml` (and in the `setup.py`
of the Python packages) with your real name and email:

```xml
<maintainer email="your_email@example.com">Your Name</maintainer>
```
