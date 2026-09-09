"""Launch robot_state_publisher (and optionally RViz2 and a joint-state GUI).

Processes robot.urdf.xacro at launch time and feeds the resulting URDF to
robot_state_publisher, which publishes the static TF tree plus TF for any
movable joints reported on /joint_states.

Launch arguments
----------------
    use_sim_time    : forward sim time to the nodes (default: false)
    use_rviz        : also start RViz2 with the description config (default: true)
    use_jsp_gui     : start joint_state_publisher_gui so the wheel joints can be
                      inspected/driven in RViz (default: true)

This is a DESCRIPTION-ONLY launch. It publishes no map/odom transforms and
starts no Gazebo, controllers, or sensors.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("surveillance_robot_description")

    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "robot.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "robot_description.rviz"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")

    # Expand xacro -> URDF as a string parameter for robot_state_publisher.
    robot_description = {
        "robot_description": Command(["xacro ", xacro_file]),
        "use_sim_time": use_sim_time,
    }

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2 with the robot description config.",
    )
    declare_use_jsp_gui = DeclareLaunchArgument(
        "use_jsp_gui",
        default_value="true",
        description="Start joint_state_publisher_gui to inspect wheel joints.",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Publishes /joint_states for the continuous wheel joints so RViz can show
    # them. This is a visualization aid only, NOT a wheel controller.
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(use_jsp_gui),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_use_rviz,
            declare_use_jsp_gui,
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )
