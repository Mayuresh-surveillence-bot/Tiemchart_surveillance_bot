"""Launch the existing Gazebo simulation with online SLAM Toolbox mapping.

This Chunk 6 entry point composes the established simulation bringup, then adds
only online 2D SLAM and mapping RViz. It does not launch AMCL, Nav2, patrol, or
mission nodes. The Gazebo launch remains the sole owner of robot spawn,
controllers, /clock, /odom, /scan, and robot_state_publisher.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = FindPackageShare("surveillance_robot_bringup")
    pkg_navigation = FindPackageShare("surveillance_robot_navigation")

    world = LaunchConfiguration("world")
    site_visual = LaunchConfiguration("site_visual")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, "launch", "simulation_bringup.launch.py"])
        ),
        launch_arguments={
            "use_rviz": "false", "world": world, "site_visual": site_visual,
            "spawn_x": spawn_x, "spawn_y": spawn_y, "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    slam_toolbox = Node(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox", output="screen",
        parameters=[PathJoinSubstitution([pkg_navigation, "config", "slam_toolbox.yaml"])],
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="mapping_rviz", output="screen",
        arguments=["-d", PathJoinSubstitution([pkg_navigation, "rviz", "mapping.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="construction_site_chunk5",
                              choices=["construction_site_chunk5", "test_world"]),
        DeclareLaunchArgument("site_visual", default_value="auto",
                              choices=["auto", "true", "false"]),
        DeclareLaunchArgument("spawn_x", default_value="auto"),
        DeclareLaunchArgument("spawn_y", default_value="auto"),
        DeclareLaunchArgument("spawn_z", default_value="0.15"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        simulation, slam_toolbox, rviz,
    ])
