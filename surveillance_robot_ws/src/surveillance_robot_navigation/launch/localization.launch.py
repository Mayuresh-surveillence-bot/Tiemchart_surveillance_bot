"""Saved-map localization mode. This launch intentionally never starts SLAM."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _require_map(context):
    map_file = Path(LaunchConfiguration("map").perform(context))
    if not map_file.is_file():
        raise RuntimeError(
            f"Saved map is required but was not found: {map_file}. "
            "Generate a real Chunk 6 map, then pass map:=/path/to/map.yaml."
        )
    return []


def generate_launch_description():
    pkg_nav = FindPackageShare("surveillance_robot_navigation")
    pkg_nav2 = FindPackageShare("nav2_bringup")
    params = PathJoinSubstitution([pkg_nav, "config", "nav2_params.yaml"])
    default_map = PathJoinSubstitution([pkg_nav, "maps", "construction_site.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("autostart", default_value="true"),
        OpaqueFunction(function=_require_map),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2, "launch", "localization_launch.py"])
            ),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "use_sim_time": "true",
                "autostart": LaunchConfiguration("autostart"),
                "params_file": params,
            }.items(),
        ),
    ])
