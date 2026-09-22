"""Complete Chunk 7 simulation, localization, and NavigateToPose launch.

SLAM Toolbox is deliberately not included. AMCL is the sole map->odom authority.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_bringup = FindPackageShare("surveillance_robot_bringup")
    pkg_nav = FindPackageShare("surveillance_robot_navigation")
    pkg_nav2 = FindPackageShare("nav2_bringup")
    world = LaunchConfiguration("world")
    site_visual = LaunchConfiguration("site_visual")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    params = PathJoinSubstitution([pkg_nav, "config", "nav2_params.yaml"])
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_bringup, "launch", "simulation_bringup.launch.py"])),
        launch_arguments={"use_rviz": "false", "world": world, "site_visual": site_visual,
                          "spawn_x": spawn_x, "spawn_y": spawn_y, "spawn_z": spawn_z, "spawn_yaw": spawn_yaw}.items(),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_nav, "launch", "localization.launch.py"])),
        launch_arguments={"map": LaunchConfiguration("map"), "autostart": "true"}.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_nav2, "launch", "navigation_launch.py"])),
        launch_arguments={"use_sim_time": "true", "autostart": "true", "params_file": params}.items(),
    )
    rviz = Node(package="rviz2", executable="rviz2", name="navigation_rviz", output="screen",
                arguments=["-d", PathJoinSubstitution([pkg_nav, "rviz", "navigation.rviz"])],
                parameters=[{"use_sim_time": True}], condition=IfCondition(LaunchConfiguration("use_rviz")))
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=PathJoinSubstitution([pkg_nav, "maps", "construction_site.yaml"])),
        DeclareLaunchArgument("world", default_value="construction_site_chunk5", choices=["construction_site_chunk5", "test_world"]),
        DeclareLaunchArgument("site_visual", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("spawn_x", default_value="auto"), DeclareLaunchArgument("spawn_y", default_value="auto"),
        DeclareLaunchArgument("spawn_z", default_value="0.15"), DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"), simulation, localization, navigation, rviz,
    ])
