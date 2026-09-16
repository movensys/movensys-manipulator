import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Quest right-controller -> MoveIt Servo POSE teleop (relative clutch).

    Assumes moveit.launch.py (move_group + servo_node + RViz) and the sim bridge
    (sim_bridge.launch.py) are already running, and that the
    quest_pose_publisher node is streaming on the other DDS peer.
    """
    manipulator_model = os.environ.get("MANIPULATOR_MODEL", "dobot_cr3a")
    params_file = os.path.join(
        get_package_share_directory("movensys_manipulator_moveit_config"),
        "config",
        manipulator_model,
        "quest_servo_teleop.yaml",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time (true for Gazebo/Isaac Sim)",
        ),
        Node(
            package="movensys_manipulator_moveit_config",
            executable="quest_servo_teleop",
            name="quest_servo_teleop",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
    ])
