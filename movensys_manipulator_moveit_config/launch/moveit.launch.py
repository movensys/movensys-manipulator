import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    declared_arguments = []

    # RViz config argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="movensys_manipulator_moveit.rviz",
            description="RViz configuration file",
        )
    )

    # ⚠ Add use_sim_time argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        )
    )

    # Publish /robot_description here. Set false when a backend launch (Gazebo
    # sim or wmx_ros2_control) already publishes it, so there is a single
    # publisher and the CR3A controller_manager reads the right description.
    declared_arguments.append(
        DeclareLaunchArgument(
            "rsp",
            default_value="true",
            description="Start robot_state_publisher here (set false to defer to a "
                        "backend launch that publishes /robot_description)",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):

    # Load the argument
    use_sim_time = LaunchConfiguration("use_sim_time")

    manipulator_model = os.environ.get("MANIPULATOR_MODEL", "dobot_cr3a")

    # Build MoveIt config
    # *.srdf -> *.urdf ??
    # ompl, chomp, pilz_industrial_motion_planner.
    moveit_config = (
        MoveItConfigsBuilder("movensys_manipulator")
        .robot_description_semantic(
            file_path=f"config/{manipulator_model}/movensys_manipulator.srdf")
        .robot_description(file_path=f"config/{manipulator_model}/movensys_manipulator.urdf.xacro")
        .robot_description_kinematics(file_path=f"config/{manipulator_model}/kinematics.yaml")
        .joint_limits(file_path=f"config/{manipulator_model}/joint_limits.yaml")
        .trajectory_execution(file_path=f"config/{manipulator_model}/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .pilz_cartesian_limits(file_path=f"config/{manipulator_model}/pilz_cartesian_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # move_group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            # Load the Pilz sequence capabilities so clients can plan/execute
            # blended LIN sequences via the /sequence_move_group action.
            {
                "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction "
                                "pilz_industrial_motion_planner/MoveGroupSequenceService",
            },
        ],
    )

    # RViz config
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("movensys_manipulator_moveit_config"), "rviz", rviz_base]
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        condition=IfCondition(LaunchConfiguration("rsp")),
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("movensys_manipulator_moveit_config"),
                "launch",
                "trajectory_service.launch.py",
            ])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return [
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        api_launch,
    ]
