import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    manipulator_model = os.environ.get("MANIPULATOR_MODEL", "dobot_cr3a")
    moveit_config = (
        MoveItConfigsBuilder("movensys_manipulator")
        .robot_description(
            file_path=f"config/{manipulator_model}/movensys_manipulator.urdf.xacro"
        )
        .robot_description_semantic(
            file_path=f"config/{manipulator_model}/movensys_manipulator.srdf"
        )
        .robot_description_kinematics(
            file_path=f"config/{manipulator_model}/kinematics.yaml"
        )
        .joint_limits(file_path=f"config/{manipulator_model}/joint_limits.yaml")
        .trajectory_execution(
            file_path=f"config/{manipulator_model}/moveit_controllers.yaml"
        )
        .pilz_cartesian_limits(
            file_path=f"config/{manipulator_model}/pilz_cartesian_limits.yaml"
        )
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    client_config = os.path.join(
        get_package_share_directory("movensys_manipulator_moveit_config"),
        "config", manipulator_model, "moveit2_client.yaml",
    )

    # moveit.launch.py is already running; launch only the demo client here.
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulation time"
        ),
        Node(
            package="movensys_manipulator_moveit_config",
            executable="joint_demo",
            name="joint_demo",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                client_config,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
