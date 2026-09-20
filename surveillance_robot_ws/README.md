# Surveillance Robot

A mobile surveillance robot capable of autonomous navigation and patrol inside an
environment. This repository contains the **simulation and ROS 2 software stack**.

## Current Status

**Chunks 2–4 robot stack + Chunk 5 environment integration (runtime pending).**

The existing Xacro robot, ros2_control differential drive, sensor definitions,
and simulation launch are retained. The default world is now the supplied Chunk 5
construction site. The original two-wall `test_world` remains selectable.
SLAM/localization, Nav2, and patrol are not implemented in this checkout.

**BUILD: NOT VERIFIED. RUNTIME: NOT VERIFIED.** ROS 2, colcon, Gazebo, and xacro
are not installed in the integration sandbox. Eleven offline contract tests pass;
that is not a ROS build, SDF engine validation, or physical simulation result.
The 145 MB high-detail visual GLB was not supplied.

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
# On a ROS 2 Jazzy / Gazebo Harmonic host, from this repository root:
cd surveillance_robot_ws

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Install dependencies (from surveillance_robot_ws/)
rosdep install --from-paths src --ignore-src -r -y
```

## Build

From `surveillance_robot_ws/`:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Package List

- **surveillance_robot_description** — contains the existing modular URDF/Xacro,
  wheel/chassis geometry, inertias, sensor frames, and description-only RViz launch.
- **surveillance_robot_gazebo** — owns the single Gazebo instance, robot spawn,
  controllers, clock/sensor bridges, and selectable simulation worlds.
- **surveillance_robot_bringup** — composes the simulation launch and forwards
  world, visual, spawn, and RViz options without starting duplicate nodes.
- **surveillance_robot_navigation** — will hold SLAM Toolbox, localization, and
  Nav2 parameters, maps, and navigation launch files.
- **surveillance_robot_patrol** — will hold patrol behavior nodes such as
  `patrol_manager`, `mission_manager`, and `waypoint_manager`.
- **surveillance_robot_interfaces** — will hold custom ROS 2 interfaces
  (e.g. `PatrolStatus.msg`, `SurveillanceEvent.msg`, `PatrolMission.action`).
- **surveillance_robot_tools** — will hold development/diagnostic utilities such
  as diagnostic and telemetry nodes.

## Development Roadmap

- Chunk 1 — Repository
- Chunk 2 — Robot Description
- Chunk 3 — Gazebo + Drive
- Chunk 4 — Sensors (existing definitions; bridge wiring repaired here)
- **Chunk 5 — Environment integration** (offline checks passed; runtime pending)
- Chunk 6 — SLAM
- Chunk 7 — Navigation
- Chunk 8 — Patrol
- Chunk 9 — Surveillance Features
- Chunk 10 — Integration

## Chunk 4 inspection and integration decisions

| Area | Existing implementation and changes |
| --- | --- |
| Robot | `description/urdf/robot.urdf.xacro` includes dimensions, chassis, wheels, sensor frames, and simulation-only control/sensor macros. All robot geometry, dimensions, inertias, friction, and sensor parameters are unchanged. |
| Spawn | One `ros_gz_sim create` consumes `/robot_description`. Now targets the selected world explicitly. Chunk 5 defaults to `(-12, -12, 0.15)` with yaw 0; `test_world` retains `(0, 0, 0.15)`. |
| Gazebo | One `ros_gz_sim/gz_sim.launch.py` include in `gazebo/launch/simulation.launch.py`; bringup only composes it. The bundle's independent launch is not installed or invoked. |
| Differential drive | Existing `gz_ros2_control/GazeboSimSystem` hardware and Gazebo plugin start the controller manager. Joint-state broadcaster then diff-drive controller are spawned in sequence. Track 0.45 m; wheel radius 0.10 m. No Gazebo DiffDrive plugin was added. |
| `/cmd_vel` | Existing Jazzy controller input is `geometry_msgs/msg/TwistStamped`, remapped from `~/cmd_vel`. No control bridge or competing publisher was added. |
| `/odom` | Existing controller published its private `~/odom`; added the missing remap to `/odom`. Odometry remains encoder-based (`open_loop: false`), not a Gazebo pose bridge. |
| TF | Controller alone owns `odom -> base_footprint`. Robot state publisher owns `base_footprint -> base_link`, wheel and sensor transforms, including `camera_link -> camera_optical_frame`. No `map -> odom` publisher exists. |
| LiDAR | Existing `gpu_lidar`, `/scan`, `laser_link`, 720 samples, 10 Hz, 12 m range retained. |
| IMU | Existing `/imu/data`, `imu_link`, 50 Hz retained. |
| Camera | Existing `/camera/image_raw` and `/camera/camera_info`, `camera_optical_frame`, 640×480 at 15 Hz retained. |
| Bridges | Existing clock bridge retained. Previously unreferenced `sensor_bridges.yaml` is now loaded once by a sensor bridge. Its four GZ_TO_ROS mappings do not overlap the clock or bridge TF/control/odom. Two bridge processes, five unique mappings; not duplicate bridges. |
| SLAM/localization | No implementation, launch, map, AMCL, EKF, or SLAM Toolbox configuration exists in this branch. Nothing was removed or replaced. |
| Nav2 | Navigation package is a skeleton. No existing Nav2 execution to re-test; future Nav2 must match the stamped command interface. |
| Launch files | Description-only launch remains separate and unchanged; do not launch it alongside simulation because simulation already owns robot state publisher. |
| Build references | Removed CMake installation of absent `scripts/check_sensors.py`; replaced the absent `test/test_sensor_contracts.py` registration with the supplied offline integration tests. No existing script/test implementation was deleted. |

### Uploaded bundle inspection

Every file was inspected, including both GLB binary headers, embedded JSON,
vertex/index buffers, bounds, and checksums. There are no external buffers/textures
in the two collision GLBs.

- `collision/ground_collision.glb`, `collision/site_static_collision.glb`: copied
  byte-for-byte into the existing Gazebo package.
- `config/chunk5_metadata.json`: copied byte-for-byte; recorded scale/offset are
  provenance, **not** transformations to apply again to the baked mesh vertices.
- `worlds/construction_site_chunk5.sdf`: adapted into the existing world directory.
  Fixed the invalid model `static="true"` attribute to `<static>true</static>`,
  rebased mesh paths, and carried over the six existing Harmonic world systems
  (Physics, UserCommands, SceneBroadcaster, Contact, Sensors/ogre2, Imu).
- `model.sdf`: contains the same site geometry and invalid static attribute;
  deliberately not installed as a second model source. The integrated world owns
  the site once and does not need `model.config` or model search-path overrides.
- `launch/chunk5_sim.launch.py`: starts a standalone Gazebo instance; deliberately
  not imported. The existing robot launch is the only simulation entry point.
- `package.xml`, `setup.py`, `setup.cfg`, empty
  `resource/surveillance_robot_chunk5`: inspected, not installed as another ROS
  package. Existing `ament_cmake` resource installation covers world, metadata,
  collision assets and the future visual. The bundle's setup omitted metadata.
- `README.md`: inspected for environment specifications and validation criteria;
  its standalone launch instructions are superseded by those below.

The supplied assets are marked **Proprietary** in the bundle manifest. A local
license notice preserves that distinction from the surrounding Apache-2.0 code;
confirm redistribution rights with the asset owner.

### Environment and spawn invariants

SDF **1.9**, **50 × 50 m** terrain centered on the origin, **0.001 s** physics
step, and ground/static friction **mu=0.75, mu2=0.75** are preserved. Visual and
collision geometry stay separate; neither a flat replacement floor nor a
replacement visual mesh was introduced. The supplied ODE settings remain in the
SDF, but Harmonic's existing Physics system normally uses DART; this is not a
claim that it implements every legacy ODE solver tuning parameter.

Binary inspection found terrain bounds `x,y=[-25,25]`,
`z=[0.06815285,0.08765371]` m (10,201 vertices; 20,000 triangles).
The site collision bounds are approximately `x=[-8.92172,6.36230]`,
`y=[-6.24427,9.03975]`, `z=[-0.02841,2.16382]` m (14,684 triangles).
The old `(0,0)` spawn overlaps site triangles up to about 1.22 m high.
The new `(-12,-12)` spawn is outside the entire static-mesh bounds and inside
terrain bounds. With the unchanged base-footprint/wheel geometry, wheel bottoms
start at z=0.15 m, above the terrain, allowing settling rather than penetration.
These are geometric checks, **not** proof of wheel contact, upright stability,
traction, or collision response. The existing two-wheel chassis has no added
caster or balancing controller; its pitch/settling must be observed at runtime.

## Launch and missing visual asset

After a build and sourcing the workspace on a ROS/Gazebo host, the exact command
for the supplied small bundle is:

```bash
ros2 launch surveillance_robot_bringup simulation_bringup.launch.py world:=construction_site_chunk5 site_visual:=false use_rviz:=true
```

This is **collision-only** mode. The site has no rendered visual. Camera and
GPU LiDAR consume the rendered scene, not collision triangles; they cannot
observe this site's obstacles in this mode. Publishing sensor messages would
not establish meaningful construction-site perception, mapping, or navigation.
Use Gazebo's collision visualization for inspection only, not as a substitute
for the final sensor-visible mesh.

Place the actual 145 MB GLB at this exact repository-relative location:

```text
surveillance_robot_ws/src/surveillance_robot_gazebo/models/construction_site_chunk5/meshes/construction_site_chunk5_visual.glb
```

The supplied metadata calls the original visual `construction_site_ground_50x50.glb`;
use the filename above when placing that actual asset. Do not substitute a proxy.
Rebuild after adding it so resource installation discovers the new file. Its
installed location is
`$(ros2 pkg prefix surveillance_robot_gazebo)/share/surveillance_robot_gazebo/models/construction_site_chunk5/meshes/construction_site_chunk5_visual.glb`.
A 145 MB asset exceeds GitHub's ordinary file-size limit; distribute it separately
or use Git LFS rather than a normal Git blob. No visual file or LFS pointer was
fabricated by this change.

To require the real visual (fails before starting Gazebo when it is missing):

```bash
ros2 launch surveillance_robot_bringup simulation_bringup.launch.py world:=construction_site_chunk5 site_visual:=true use_rviz:=true
```

`site_visual:=auto` is the default: loads the real GLB if present, otherwise emits
an explicit collision-only warning and omits only that visual. Missing collision
meshes always fail before starting Gazebo. The launch prepares a temporary SDF
with absolute installed mesh paths and cleans it up on launch-process exit;
source assets remain unchanged. Use the ROS launch rather than launching the
unprepared source SDF directly when the visual is missing.

The same launch arguments are supported by the low-level
`surveillance_robot_gazebo simulation.launch.py` entry point. Run **one** entry
point, not both. `spawn_x`, `spawn_y`, `spawn_z`, and `spawn_yaw` override the pose.
The original regression world remains available:

```bash
ros2 launch surveillance_robot_bringup simulation_bringup.launch.py world:=test_world use_rviz:=true
```

## Chunk 4 → Chunk 5 verification report

**BUILD: NOT VERIFIED** — no ROS 2/ament/colcon installation in this sandbox.
`colcon build --symlink-install` could not be executed.

**RUNTIME: NOT VERIFIED** — no `gz`, `ros2`, or xacro executable in this sandbox.
Gazebo was not launched. No final high-detail visual is available.

| Item | Status | Evidence / outstanding verification |
| --- | --- | --- |
| Robot | NOT VERIFIED | Original description retained; Xacro expansion and spawning require ROS. |
| Differential drive | NOT VERIFIED | Existing hardware/controller configuration retained; actual drive/turn tests pending. |
| Gazebo | NOT VERIFIED | One launch include statically confirmed; process/plugin startup not observed. |
| World | NOT VERIFIED | SDF XML, systems, asset layout and missing-visual handling checked; engine loading pending. |
| Collision | NOT VERIFIED | Both original GLBs checked; ground support, obstacle contact, rut/bump crossing and settling not simulated. |
| CMD_VEL | NOT VERIFIED | Existing stamped command path preserved; motion not measured. |
| ODOM | NOT VERIFIED | Controller remap added; messages, timestamps and motion not measured. |
| TF | NOT VERIFIED | Ownership and frame names retained; live TF availability/uniqueness not measured. |
| LiDAR | NOT VERIFIED | Sensor definition and bridge checked; scan stream/frames/timing not observed; site returns blocked by missing visual. |
| IMU | NOT VERIFIED | Sensor/system/bridge present; messages, orientation and timestamps not observed. |
| Camera | NOT VERIFIED | Sensor/system/bridges present; image/info streams not observed; site visuals unavailable. |
| SLAM | NOT VERIFIED | Not implemented in this checkout; no map-to-odom source or mapping runtime to test. |
| Nav2 | NOT VERIFIED | Not implemented in this checkout; no navigation runtime to test. |

**Offline checks: PASS (11 tests).** Executed with Python's standard library:

```bash
python3 surveillance_robot_ws/src/surveillance_robot_gazebo/test/test_chunk5_integration.py -v
```

Tests cover byte-identical asset checksums, actual GLB buffers/bounds, geometric
spawn clearance, preserved world parameters, six unique system plugins,
collision-only path resolution, missing-asset errors, optional/full visual path
handling using disposable fixtures, symlink-install path semantics, the old
world, unique bridge mappings, sensor/frame contracts, Python syntax, and CMake
resource/test references. XML/path preparation is exercised without loading or
mocking ROS. These tests do **not** validate ROS launch execution or Gazebo's SDF,
Assimp GLB importer, DART collision behavior, or rendering.

### Runtime acceptance still required

On the ROS/Gazebo host, build with `colcon build --symlink-install`, source the
workspace, and run the single launch above. Then:

1. Confirm one world/server, one robot, one robot state publisher, both active
   controllers, and one publisher per bridged sensor/clock topic.
2. Observe wheel settling onto the terrain, no falling/tunneling or trapped
   spawn, and stable chassis attitude. Test straight driving, rotation, terrain
   bumps/ruts, and physical contact against a known static obstacle.
3. Use stamped, simulation-clock `/cmd_vel` commands at a rate above the existing
   0.5 s timeout; verify `/odom` and wheel motion agree. Stop commands after tests.
4. Check live `odom -> base_footprint -> base_link -> sensor` TF paths, unique
   authorities, sensor header frames, and simulation timestamps. Do not add a
   second odometry/TF bridge to repair a missing controller.
5. Check `/scan`, `/imu/data`, `/camera/image_raw`, and `/camera/camera_info`
   rates and data. With the real visual installed, verify visual/collision
   alignment and meaningful camera/LiDAR observations against site geometry.
6. Re-test SLAM/localization and Nav2 only after those stacks exist and the real
   visual is available; no compatibility/runtime success is claimed here.

Unresolved: actual ROS build and launch, Harmonic/Assimp GLB loading and axis
alignment, DART ground/static mesh contact, wheel/chassis stability, controller
and sensor runtime behavior, final visual delivery, downstream mapping/navigation,
and asset redistribution permission. No unrelated robot redesign was performed.

### Files changed

Paths below are relative to `surveillance_robot_ws/`.

Modified:
- `README.md`
- `src/surveillance_robot_bringup/launch/simulation_bringup.launch.py`
- `src/surveillance_robot_gazebo/CMakeLists.txt`
- `src/surveillance_robot_gazebo/launch/simulation.launch.py`

Added:
- `src/surveillance_robot_gazebo/worlds/construction_site_chunk5.sdf`
- `src/surveillance_robot_gazebo/config/chunk5_metadata.json`
- `src/surveillance_robot_gazebo/models/construction_site_chunk5/collision/ground_collision.glb`
- `src/surveillance_robot_gazebo/models/construction_site_chunk5/collision/site_static_collision.glb`
- `src/surveillance_robot_gazebo/models/construction_site_chunk5/meshes/.gitkeep`
- `src/surveillance_robot_gazebo/models/construction_site_chunk5/LICENSE`
- `src/surveillance_robot_gazebo/test/test_chunk5_integration.py`

## Maintainer

Replace the placeholder maintainer in every `package.xml` (and in the `setup.py`
of the Python packages) with your real name and email:

```xml
<maintainer email="your_email@example.com">Your Name</maintainer>
```
