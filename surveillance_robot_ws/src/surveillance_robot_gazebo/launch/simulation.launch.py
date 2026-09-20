"""Bring up the surveillance robot in Gazebo Harmonic with ros2_control.

Startup sequence (spec section 12):
    1. Start one gz-sim instance with the selected world (Chunk 5 by default).
    2. Publish robot_description (xacro sim_mode:=true) via robot_state_publisher.
    3. Bridge /clock (gz -> ROS) so use_sim_time works everywhere.
    4. Spawn the robot into Gazebo from the /robot_description topic.
    5. After the spawn succeeds, load joint_state_broadcaster.
    6. After the broadcaster is active, load diff_drive_controller.

The controllers are chained with event handlers so they load in the correct
order once the entity exists. The controller manager itself is started by the
gz_ros2_control plugin embedded in the URDF, not here.

This launch starts ONLY simulation + control. No SLAM, Nav2, localization, or
patrol (those belong to later chunks).
"""

import atexit
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _prepare_world_tree(world_file, visual_mode):
    if visual_mode not in ("auto", "true", "false"):
        raise ValueError("site_visual must be auto, true, or false")
    tree = ET.parse(world_file)
    world = tree.getroot().find("world")
    messages = []
    if world is None:
        raise ValueError(f"No world in {world_file}")

    for link in world.findall(".//link"):
        visual = link.find("visual[@name='high_detail_visual']")
        if visual is None:
            continue
        uri = visual.find("geometry/mesh/uri")
        visual_path = (world_file.parent / uri.text.strip()).absolute()
        if visual_mode == "true" and not visual_path.is_file():
            raise FileNotFoundError(f"Required Chunk 5 visual GLB missing: {visual_path}")
        if visual_mode == "false" or not visual_path.is_file():
            link.remove(visual)
            messages.append(
                "WARNING: Chunk 5 COLLISION-ONLY mode. No replacement visual is used. "
                "The site is invisible to the camera and GPU LiDAR; site perception, "
                f"SLAM and Nav2 cannot be validated. Visual GLB location: {visual_path}"
            )

    # Absolute paths remain valid when Gazebo reads the generated SDF in /tmp.
    # Do not resolve symlinks: --symlink-install may link the world file alone.
    for uri in world.findall(".//mesh/uri"):
        mesh_path = (world_file.parent / uri.text.strip()).absolute()
        if not mesh_path.is_file():
            raise FileNotFoundError(f"Required world mesh missing: {mesh_path}")
        uri.text = str(mesh_path)
    return tree, world.attrib["name"], messages


def _prepare_world(context):
    package = Path(FindPackageShare("surveillance_robot_gazebo").perform(context))
    selection = LaunchConfiguration("world").perform(context)
    world_file = package / "worlds" / f"{selection}.sdf"
    tree, world_name, messages = _prepare_world_tree(
        world_file, LaunchConfiguration("site_visual").perform(context)
    )
    temporary = tempfile.TemporaryDirectory(prefix="surveillance-world-")
    atexit.register(temporary.cleanup)
    generated_world = Path(temporary.name) / world_file.name
    tree.write(generated_world, encoding="utf-8", xml_declaration=True)

    actions = [
        SetLaunchConfiguration("resolved_world", str(generated_world)),
        SetLaunchConfiguration("world_name", world_name),
    ]
    for argument in ("spawn_x", "spawn_y", "spawn_z", "spawn_yaw"):
        value = LaunchConfiguration(argument).perform(context)
        if value == "auto" and argument in ("spawn_x", "spawn_y"):
            value = "-12.0" if selection == "construction_site_chunk5" else "0.0"
        if not math.isfinite(float(value)):
            raise ValueError(f"{argument} must be finite")
        actions.append(SetLaunchConfiguration(argument, value))
    actions.extend(LogInfo(msg=message) for message in messages)
    return actions


def generate_launch_description() -> LaunchDescription:
    pkg_gazebo = FindPackageShare("surveillance_robot_gazebo")
    pkg_description = FindPackageShare("surveillance_robot_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    xacro_file = PathJoinSubstitution([pkg_description, "urdf", "robot.urdf.xacro"])
    controllers_file = PathJoinSubstitution([pkg_gazebo, "config", "controllers.yaml"])
    world_file = LaunchConfiguration("resolved_world")
    sensor_bridges_file = PathJoinSubstitution([pkg_gazebo, "config", "sensor_bridges.yaml"])
    world_arguments = [
        DeclareLaunchArgument(
            "world", default_value="construction_site_chunk5",
            choices=["construction_site_chunk5", "test_world"],
            description="World for the existing robot simulation.",
        ),
        DeclareLaunchArgument(
            "site_visual", default_value="auto", choices=["auto", "true", "false"],
            description="auto: use the site GLB if present; true: require it; false: collision-only.",
        ),
        DeclareLaunchArgument("spawn_x", default_value="auto"),
        DeclareLaunchArgument("spawn_y", default_value="auto"),
        DeclareLaunchArgument("spawn_z", default_value="0.15"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
    ]

    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Start RViz2 alongside the simulation.",
    )

    # Robot description built for simulation: sim_mode:=true pulls in the
    # ros2_control block and the gz_ros2_control plugin (with the YAML path).
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    'xacro "',
                    xacro_file,
                    '" sim_mode:=true controllers_file:="',
                    controllers_file,
                    '"',
                ]
            ),
            value_type=str,
        ),
        "use_sim_time": True,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Start Gazebo Harmonic with the selected world (-r = run immediately).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r -v4 ", world_file]}.items(),
    )

    # Bridge the simulation clock so all ROS nodes share sim time.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sensor_bridge",
        parameters=[{"config_file": sensor_bridges_file, "use_sim_time": True}],
        output="screen",
    )

    # Spawn the robot from the published /robot_description topic.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_surveillance_robot",
        arguments=[
            "-topic", "robot_description",
            "-name", "surveillance_robot",
            "-world", LaunchConfiguration("world_name"),
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
        output="screen",
    )

    # Controllers, loaded in order via the controller_manager spawner.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager",
            # Expose the controller's cmd_vel as the conventional /cmd_vel.
            "--controller-ros-args", "-r ~/cmd_vel:=/cmd_vel -r ~/odom:=/odom",
        ],
        output="screen",
    )

    # Chain: spawn robot -> joint_state_broadcaster -> diff_drive_controller.
    load_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    load_diff_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_description, "rviz", "robot_description.rviz"]),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            declare_use_rviz,
            *world_arguments,
            OpaqueFunction(function=_prepare_world),
            load_jsb_after_spawn,
            load_diff_after_jsb,
            gz_sim,
            robot_state_publisher,
            clock_bridge,
            sensor_bridge,
            spawn_robot,
            rviz,
        ]
    )
