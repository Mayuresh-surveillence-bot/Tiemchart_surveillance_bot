"""Bring up the surveillance robot in Gazebo Harmonic with ros2_control.

Startup sequence (spec section 12):
    1. Start gz-sim with the test world.
    2. Publish robot_description (xacro sim_mode:=true) via robot_state_publisher.
    3. Bridge /clock (gz -> ROS) so use_sim_time works everywhere.
    4. Spawn the robot into Gazebo from the /robot_description topic.
    5. After the spawn succeeds, load joint_state_broadcaster.
    6. After the broadcaster is active, load diff_drive_controller.

The controllers are chained with event handlers so they load in the correct
order once the entity exists. The controller manager itself is started by the
gz_ros2_control plugin embedded in the URDF, not here.

The sensor bridge starts alongside the clock bridge and forwards LiDAR, camera,
and IMU messages once the spawned robot's Gazebo sensors produce data.
This launch starts ONLY simulation + control + sensors. No SLAM, Nav2,
localization, or patrol (those belong to later chunks).
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_gazebo = FindPackageShare("surveillance_robot_gazebo")
    pkg_description = FindPackageShare("surveillance_robot_description")
    pkg_ros_gz_sim = FindPackageShare("ros_gz_sim")

    xacro_file = PathJoinSubstitution([pkg_description, "urdf", "robot.urdf.xacro"])
    controllers_file = PathJoinSubstitution([pkg_gazebo, "config", "controllers.yaml"])
    world_file = PathJoinSubstitution([pkg_gazebo, "worlds", "test_world.sdf"])
    sensor_bridges_file = PathJoinSubstitution(
        [pkg_gazebo, "config", "sensor_bridges.yaml"]
    )

    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Start RViz2 alongside the simulation.",
    )

    # Robot description built for simulation: sim_mode:=true pulls in the
    # ros2_control block and the gz_ros2_control plugin (with the YAML path).
    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_file,
                " sim_mode:=true",
                " controllers_file:=",
                controllers_file,
            ]
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

    # Start Gazebo Harmonic with the test world (-r = run immediately).
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
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.15",
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
            "--controller-ros-args", "-r ~/cmd_vel:=/cmd_vel",
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
            gz_sim,
            robot_state_publisher,
            clock_bridge,
            sensor_bridge,
            spawn_robot,
            load_jsb_after_spawn,
            load_diff_after_jsb,
            rviz,
        ]
    )
