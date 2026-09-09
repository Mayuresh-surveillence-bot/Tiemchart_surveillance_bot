"""System-level entry point for the simulated surveillance robot.

Architecture (spec sections 22-23)
----------------------------------
    Gazebo-specific launch (surveillance_robot_gazebo/simulation.launch.py)
        -> owns the low-level simulation: gz-sim, robot spawn, controllers,
           robot_state_publisher, clock bridge.

    This bringup launch (surveillance_robot_bringup)
        -> owns system-level orchestration. Right now it simply composes the
           Gazebo launch and optionally RViz, giving one stable command to run
           the whole simulated robot. Later chunks (SLAM, Nav2, patrol) are
           added here rather than inside the Gazebo launch, keeping the
           simulation layer reusable and the orchestration layer thin.

There is intentionally no duplicated node here: everything low-level is
delegated to the Gazebo launch to avoid redundant launch files.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_gazebo = FindPackageShare("surveillance_robot_gazebo")

    use_rviz = LaunchConfiguration("use_rviz")

    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 with the robot/TF/odometry display.",
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, "launch", "simulation.launch.py"])
        ),
        launch_arguments={"use_rviz": use_rviz}.items(),
    )

    return LaunchDescription(
        [
            declare_use_rviz,
            simulation,
        ]
    )
