# Chunk 7 — Saved-Map Localization and Nav2

## Scope and modes

Mapping remains **Mode A**: Gazebo + sensors + SLAM Toolbox. It is separate from
this mode and must not run at the same time as AMCL.

Chunk 7 is **Mode B**: Gazebo + sensors + saved map + Map Server + AMCL + Nav2.
AMCL is the sole `map -> odom` publisher. The existing differential drive remains
the sole `odom -> base_footprint` publisher.

## Required map

No saved construction-site map is committed in this repository. Do not fabricate
one. Before launching localization/navigation, create a real map in Chunk 6 on a
ROS/Gazebo host and save it as, for example:

```text
src/surveillance_robot_navigation/maps/construction_site.yaml
src/surveillance_robot_navigation/maps/construction_site.pgm
```

The map path is always configurable with `map:=/absolute/or/workspace/path.yaml`.
The default is the intended package path above and launch fails clearly if it is
missing.

## Interfaces

| Role | Value |
| --- | --- |
| Map / global frame | `map` |
| Odometry frame | `odom` |
| Navigation base frame | `base_footprint` |
| LiDAR frame / topic | `laser_link` / `/scan` |
| Odometry | `/odom` |
| Command input | `/cmd_vel` (`geometry_msgs/msg/TwistStamped`) |

The footprint is `[[-0.28,-0.27],[-0.28,0.27],[0.28,0.27],[0.28,-0.27]]`.
It covers the 0.50 × 0.40 m chassis, 0.50 m wheel envelope, camera projection,
and a small clearance margin.

## Dependencies

On Ubuntu 24.04 / ROS 2 Jazzy:

```bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

The configuration selects the standard Jazzy-compatible `nav2_navfn_planner::NavfnPlanner`
and `dwb_core::DWBLocalPlanner`. Their presence is not verified in the current
sandbox; verify with `ros2 pkg prefix nav2_navfn_planner` and
`ros2 pkg prefix nav2_dwb_controller` on the ROS host. Nav2 is explicitly set to
publish `TwistStamped` commands to match the existing controller.

## Build and launch

```bash
cd surveillance_robot_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

ros2 launch surveillance_robot_navigation navigation.launch.py \
  map:=$(pwd)/src/surveillance_robot_navigation/maps/construction_site.yaml \
  world:=construction_site_chunk5 site_visual:=true use_rviz:=true
```

`site_visual:=true` requires the separately distributed Chunk 5 visual GLB at its
existing expected path. Do not commit or replace that asset.

## Operator workflow

1. Confirm `/clock`, `/scan`, `/odom`, and TF are live.
2. Confirm Map Server, AMCL, planner, controller, behavior server, and BT
   navigator reach the active lifecycle state.
3. In RViz, Fixed Frame is `map`. Use **2D Pose Estimate** to set the robot's
   initial x/y/yaw estimate.
4. Confirm `/amcl_pose`, `map -> odom`, scan alignment, and costmaps.
5. Use the Nav2 goal tool to send one reachable manual `NavigateToPose` goal.

## Costmaps and recovery

The global costmap uses static, obstacle, and inflation layers in `map`. The
rolling local costmap uses obstacle and inflation layers in `odom`. Both consume
`/scan` for marking and clearing. Inflation radius is 0.45 m.

NavFn is used as one grid planner. DWB is used as one differential-drive
controller with conservative 0.45 m/s linear and 1.0 rad/s angular maxima. The
standard behavior server is configured with Spin, BackUp, DriveOnHeading, and
Wait. No custom obstacle or mission logic is added.

## Runtime verification status

**IMPLEMENTED — RUNTIME NOT VERIFIED.** The current sandbox has no ROS 2,
Gazebo, Nav2, saved map, or running simulation. Map Server, AMCL, costmaps,
planner, controller, RViz, localization, static/dynamic obstacle, recovery, and
regression tests are blocked until a real map and ROS/Gazebo host are available.
