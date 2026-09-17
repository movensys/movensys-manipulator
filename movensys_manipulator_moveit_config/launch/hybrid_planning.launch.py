#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def load_yaml(package_name, file_path):
    absolute_file_path = os.path.join(
        get_package_share_directory(package_name), file_path)
    with open(absolute_file_path, "r") as file:
        return yaml.safe_load(file)


def namespaced_topic(ros_namespace, topic):
    if not ros_namespace:
        return topic
    return "/{}/{}".format(ros_namespace, topic.lstrip("/"))


def launch_setup(context, *args, **kwargs):
    moveit_config_dump = LaunchConfiguration("moveit_config_dump").perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader)

    global_planner_param = load_yaml(
        "movensys_manipulator_moveit_config",
        "config/hybrid_planning/global_planner.yaml",
    )
    local_planner_param = load_yaml(
        "movensys_manipulator_moveit_config",
        "config/hybrid_planning/local_planner.yaml",
    )
    hybrid_planning_manager_param = load_yaml(
        "movensys_manipulator_moveit_config",
        "config/hybrid_planning/hybrid_planning_manager.yaml",
    )

    group_name = LaunchConfiguration("group_name").perform(context)
    global_planning_pipeline = LaunchConfiguration(
        "global_planning_pipeline").perform(context)
    local_solution_topic = LaunchConfiguration("local_solution_topic").perform(context)
    local_solution_topic_type = LaunchConfiguration(
        "local_solution_topic_type").perform(context)
    ros_namespace = LaunchConfiguration("ros_namespace").perform(context).strip("/")
    use_sim_time = LaunchConfiguration("use_sim_time")

    local_planner_param["group_name"] = group_name
    local_planner_param["local_solution_topic"] = local_solution_topic
    local_planner_param["local_solution_topic_type"] = local_solution_topic_type
    local_planner_param["joint_states_topic"] = namespaced_topic(
        ros_namespace, local_planner_param["joint_states_topic"])
    local_planner_param["monitored_planning_scene"] = namespaced_topic(
        ros_namespace, local_planner_param["monitored_planning_scene"])
    local_planner_param["collision_object_topic"] = namespaced_topic(
        ros_namespace, local_planner_param["collision_object_topic"])

    global_planner_param["planning_pipelines"]["pipeline_names"] = [
        global_planning_pipeline]
    global_planner_param["plan_request_params"]["planning_pipeline"] = (
        global_planning_pipeline)

    robot_description = {
        "robot_description": moveit_config_dict["robot_description"],
    }
    robot_description_semantic = {
        "robot_description_semantic": moveit_config_dict[
            "robot_description_semantic"],
    }
    kinematics_yaml = {
        "robot_description_kinematics": moveit_config_dict[
            "robot_description_kinematics"],
    }
    robot_description_planning = {
        "robot_description_planning": moveit_config_dict[
            "robot_description_planning"],
    }
    planning_pipeline_config = {
        "planning_pipelines": moveit_config_dict["planning_pipelines"],
    }
    if global_planning_pipeline in moveit_config_dict:
        planning_pipeline_config[global_planning_pipeline] = moveit_config_dict[
            global_planning_pipeline]

    return [
        ComposableNodeContainer(
            name="hybrid_planning_container",
            namespace=ros_namespace,
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="moveit_hybrid_planning",
                    plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                    name="global_planner",
                    parameters=[
                        global_planner_param,
                        robot_description,
                        robot_description_semantic,
                        kinematics_yaml,
                        robot_description_planning,
                        planning_pipeline_config,
                        {"use_sim_time": use_sim_time},
                    ],
                ),
                ComposableNode(
                    package="moveit_hybrid_planning",
                    plugin="moveit::hybrid_planning::LocalPlannerComponent",
                    name="local_planner",
                    parameters=[
                        local_planner_param,
                        robot_description,
                        robot_description_semantic,
                        kinematics_yaml,
                        robot_description_planning,
                        {"use_sim_time": use_sim_time},
                    ],
                ),
                ComposableNode(
                    package="moveit_hybrid_planning",
                    plugin="moveit::hybrid_planning::HybridPlanningManager",
                    name="hybrid_planning_manager",
                    parameters=[
                        hybrid_planning_manager_param,
                        {"use_sim_time": use_sim_time},
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("moveit_config_dump"),
        DeclareLaunchArgument("ros_namespace", default_value=""),
        DeclareLaunchArgument(
            "group_name", default_value="movensys_manipulator_arm"),
        DeclareLaunchArgument("global_planning_pipeline", default_value="ompl"),
        DeclareLaunchArgument(
            "local_solution_topic", default_value="/joint_trajectory"),
        DeclareLaunchArgument(
            "local_solution_topic_type",
            default_value="trajectory_msgs/JointTrajectory",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
