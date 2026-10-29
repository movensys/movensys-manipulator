# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

"""
Jazzy-compatible Movensys Manipulator launch file with cuMotion integration.

This file adapts the Jazzy launch architecture to include cuMotion planning.
"""

import os
import tempfile
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
import yaml


def load_yaml(package_name, file_path):
    """Load YAML file from package."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def cumotion_params():
    """Load cuMotion planning parameters."""
    config_file_path = os.path.join(
        get_package_share_directory('isaac_ros_cumotion_moveit'),
        'config',
        'isaac_ros_cumotion_planning.yaml'
    )
    with open(config_file_path) as config_file:
        config = yaml.safe_load(config_file)

    return config


def launch_setup(context: LaunchContext, *args, **kwargs) -> List[Node]:
    """Launch setup function that properly resolves launch configurations."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    read_esdf_world = LaunchConfiguration('read_esdf_world').perform(context) == 'true'
    rviz_config_file = LaunchConfiguration('rviz_config').perform(context)
    manipulator_model = os.environ.get('MANIPULATOR_MODEL', 'dobot_cr3a')

    desc_share = get_package_share_directory('movensys_manipulator_description')
    config_share = get_package_share_directory('movensys_manipulator_moveit_config')

    robot_xrdf = os.path.join(
        desc_share, 'urdf', manipulator_model, 'movensys_manipulator.xrdf')
    xacro_path = os.path.join(
        desc_share, 'urdf', manipulator_model, 'movensys_manipulator.xacro')

    # Process xacro to generate URDF with correct paths at runtime
    robot_description_content = xacro.process_file(xacro_path).toxml()
    urdf_path = os.path.join(tempfile.gettempdir(), 'movensys_manipulator.urdf')
    with open(urdf_path, 'w') as urdf_file:
        urdf_file.write(robot_description_content)

    # Build MoveIt configuration (use xacro for runtime path resolution)
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='movensys_manipulator',
            package_name='movensys_manipulator_moveit_config',
        )
        .robot_description(file_path=os.path.join(
            desc_share, 'urdf', manipulator_model, 'movensys_manipulator.xacro'))
        .robot_description_semantic(file_path=os.path.join(
            config_share, 'config', manipulator_model, 'movensys_manipulator.srdf'))
        .robot_description_kinematics(file_path=os.path.join(
            config_share, 'config', manipulator_model, 'kinematics.yaml'))
        .joint_limits(file_path=os.path.join(
            config_share, 'config', manipulator_model, 'joint_limits.yaml'))
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .trajectory_execution(file_path=os.path.join(
            config_share, 'config', manipulator_model, 'moveit_controllers.yaml'))
        .to_moveit_configs()
    )

    # Add cuMotion to planning pipelines
    cumotion_config = cumotion_params()
    moveit_config.planning_pipelines['planning_pipelines'].insert(0, 'isaac_ros_cumotion')
    moveit_config.planning_pipelines['isaac_ros_cumotion'] = cumotion_config
    moveit_config.planning_pipelines['default_planning_pipeline'] = 'isaac_ros_cumotion'

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rsp')),
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Move group node with cuMotion integration
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': use_sim_time},
        ],
    )

    # Static planning scene server (provides /publish_static_planning_scene service)
    # Only available in Jazzy
    ros_distro = os.environ.get('ROS_DISTRO', '')
    static_planning_scene_server = None
    if ros_distro == 'jazzy':
        static_planning_scene_server = Node(
            package='isaac_ros_cumotion',
            executable='static_planning_scene',
            name='static_planning_scene_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )

    cumotion_planner_node = Node(
        package='isaac_ros_cumotion',
        executable='cumotion_planner_node',
        name='cumotion_planner_node',
        output='screen',
        parameters=[
            {
                'robot': robot_xrdf,
                'urdf_path': urdf_path,
                'use_sim_time': use_sim_time,
                'read_esdf_world': read_esdf_world,
                'esdf_service_name': '/nvblox_node/get_esdf_and_gradient',
                'voxel_size': 0.01,
                'time_dilation_factor': 1.0,
                'override_moveit_scaling_factors': False,
                'max_attempts': 10,
                'num_graph_seeds': 10,
                'num_trajopt_seeds': 10,
                'num_trajopt_time_steps': 50,
                'collision_cache_cuboid': 30,
                'collision_cache_mesh': 30,
                'joint_states_topic': '/joint_states',
                'add_ground_plane': True,
                'publish_curobo_world_as_voxels': False,
                'publish_voxel_size': 0.02,
                'max_publish_voxels': 50000,
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    moveit2_api_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('movensys_manipulator_moveit_config'),
                'launch',
                'moveit2_api.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    servo_yaml = load_yaml(
        'movensys_manipulator_moveit_config',
        os.path.join('config', manipulator_model, 'servo.yaml'),
    )
    servo_container = ComposableNodeContainer(
        name='servo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='moveit_servo',
                plugin='moveit_servo::ServoNode',
                name='servo_node',
                parameters=[
                    {'moveit_servo': servo_yaml},
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    {'use_sim_time': use_sim_time},
                ],
            ),
        ],
    )

    nodes = [
        robot_state_publisher,
        move_group_node,
        cumotion_planner_node,
        rviz_node,
        moveit2_api_launch,
        servo_container,
    ]

    if static_planning_scene_server is not None:
        nodes.insert(2, static_planning_scene_server)

    return nodes


def generate_launch_description():
    """Generate launch description with cuMotion integration."""
    launch_args = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock (/clock)'
        ),
        DeclareLaunchArgument(
            'read_esdf_world',
            default_value='false',
            description='Enable ESDF world reading from nvblox for obstacle avoidance'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                get_package_share_directory('movensys_manipulator_moveit_config'),
                'rviz', 'movensys_manipulator_moveit.rviz'
            ),
            description='Path to the RViz config file'
        ),
        DeclareLaunchArgument(
            'rsp',
            default_value='true',
            description='Start robot_state_publisher here (set false to defer to a '
                        'backend launch that publishes /robot_description)'
        ),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])
