# Chunk 6 — Online SLAM Mapping

## Scope

Chunk 6 adds online 2D mapping only. It starts the existing simulation and
SLAM Toolbox, publishes `/map` and the sole `map -> odom` transform, and opens a
mapping RViz view. It does **not** launch AMCL, Nav2 planners/controllers,
costmaps, patrol, waypoint logic, or mission logic.

## Interfaces

| Role | Interface |
| --- | --- |
| Simulation clock | `/clock` |
| Command input | `/cmd_vel` (`geometry_msgs/msg/TwistStamped`) |
| Wheel odometry | `/odom` |
| LiDAR | `/scan` (`sensor_msgs/msg/LaserScan`) |
| Joint states | `/joint_states` |
| Existing TF | `odom -> base_footprint -> base_link -> laser_link` |
| SLAM output | `/map`, `map -> odom` |

The differential-drive controller remains the sole publisher of `odom ->
base_footprint`; robot_state_publisher remains the publisher of robot and sensor
fixed transforms. SLAM Toolbox is the sole publisher of `map -> odom`.

## Dependencies

On Ubuntu 24.04 with ROS 2 Jazzy, install the mapping dependencies:

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server
```

`nav2_map_server` is used only for `map_saver_cli`; this chunk does not launch
Nav2.

## Build

```bash
cd surveillance_robot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch

Simulation only remains:

```bash
ros2 launch surveillance_robot_bringup simulation_bringup.launch.py \
  world:=construction_site_chunk5 site_visual:=auto use_rviz:=true
```

Simulation, online SLAM, and mapping RViz:

```bash
ros2 launch surveillance_robot_navigation mapping.launch.py \
  world:=construction_site_chunk5 site_visual:=true use_rviz:=true
```

`site_visual:=true` requires the externally distributed visual asset at:

```text
src/surveillance_robot_gazebo/models/construction_site_chunk5/meshes/construction_site_chunk5_visual.glb
```

The source branch intentionally does not contain that large GLB. Do not commit a
replacement or alter Chunk 5 collision assets. The provided visual must be
available locally and the workspace must be rebuilt before Gazebo can install it.

## RViz and live checks

Mapping RViz uses `map` as its fixed frame and displays Map, TF, RobotModel, and
LaserScan. After launch, verify the live interfaces before driving:

```bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /odom
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic info /map
ros2 topic echo /map --once
```

Check the TF chain `map -> odom -> base_footprint -> base_link -> laser_link`.
Do not add a second odometry source or a second publisher for `map -> odom`.

Drive slowly in an open area first, then traverse a feasible loop, a narrow
passage, and a static structure. A useful map should develop recognizable walls
and stable structure alignment while the robot moves. Diagnose failures in this
order: `/clock`, `/scan`, `/odom`, LiDAR TF, odometry TF, timestamps, SLAM
configuration, then tuning.

## Save and validate a map

After the live map is useful, save it under this package's installed source map
directory:

```bash
ros2 run nav2_map_server map_saver_cli -f \
  src/surveillance_robot_navigation/maps/construction_site
```

This creates `construction_site.yaml` and `construction_site.pgm`. Validate both
files exist and can be read by `map_saver_cli`/a map server on the ROS host.
A live SLAM map and a saved map are separate artifacts.

## Verification status

- **Static implementation:** configured in this repository.
- **Build/runtime/RViz/mapping:** require a ROS 2 Jazzy + Gazebo Harmonic host.
- **Construction-site mapping:** additionally requires the real visual GLB,
  because the existing GPU LiDAR perceives rendered geometry rather than the
  collision-only world representation.
