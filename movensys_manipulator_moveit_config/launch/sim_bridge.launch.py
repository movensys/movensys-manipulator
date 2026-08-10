import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    manipulator_model = os.environ.get("MANIPULATOR_MODEL", "dobot_cr3a")

    bridge_params = os.path.join(
        get_package_share_directory("movensys_manipulator_moveit_config"),
        "config", manipulator_model, "sim_bridge.yaml")

    bridge_node = Node(
        package="movensys_manipulator_moveit_config",
        executable="sim_bridge",
        name="sim_bridge",
        output="screen",
        parameters=[bridge_params, {"use_sim_time": use_sim_time}],
    )

    return [bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        ),
        OpaqueFunction(function=launch_setup),
    ])
