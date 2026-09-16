"""Launch file for camera transform tuning with RViz visualization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('movensys_manipulator_perception')
    manipulator_model = os.environ.get('MANIPULATOR_MODEL', 'dobot_cr3a')
    camera_tf_arg_defaults = {
        'camera_0_x': '-0.402',
        'camera_0_y': '-0.149',
        'camera_0_z': '0.947',
        'camera_0_roll': '-0.076',
        'camera_0_pitch': '1.017',
        'camera_0_yaw': '0.696',
        'camera_1_x': '0.417',
        'camera_1_y': '-0.255',
        'camera_1_z': '0.879',
        'camera_1_roll': '0.074',
        'camera_1_pitch': '0.867',
        'camera_1_yaw': '2.402',
    }

    xacro_file = os.path.join(
        get_package_share_directory('movensys_manipulator_description'),
        'urdf', manipulator_model, 'movensys_manipulator.xacro')

    rviz_config = os.path.join(pkg_share, 'rviz', 'camera_transform_tuning.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (/clock)'
    )
    rsp = LaunchConfiguration('rsp')
    declare_rsp = DeclareLaunchArgument(
        'rsp',
        default_value='true',
        description='Publish robot_state_publisher for standalone camera TF tuning'
    )
    rviz = LaunchConfiguration('rviz')
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz for standalone camera TF tuning'
    )

    parent_frame = LaunchConfiguration('parent_frame')
    declare_parent_frame = DeclareLaunchArgument(
        'parent_frame',
        default_value='world_manipulator',
        description='Parent frame for camera transform'
    )
    tf_time_offset = LaunchConfiguration('tf_time_offset')
    declare_tf_time_offset = DeclareLaunchArgument(
        'tf_time_offset',
        default_value='0.0',
        description='Seconds added to GUI-published TF timestamps'
    )

    declare_camera_tf_args = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in camera_tf_arg_defaults.items()
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(rsp),
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file]),
            'use_sim_time': use_sim_time
        }]
    )

    camera_transform_tuning_node = Node(
        package='movensys_manipulator_perception',
        executable='camera_transform_tuning.py',
        name='camera_transform_tuner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'parent_frame': parent_frame,
            'tf_time_offset': tf_time_offset,
            'camera_count': 2,
            'camera_0_name': 'camera 0',
            'camera_0_frame': 'camera_nvblox_0_link',
            'camera_0_x': LaunchConfiguration('camera_0_x'),
            'camera_0_y': LaunchConfiguration('camera_0_y'),
            'camera_0_z': LaunchConfiguration('camera_0_z'),
            'camera_0_roll': LaunchConfiguration('camera_0_roll'),
            'camera_0_pitch': LaunchConfiguration('camera_0_pitch'),
            'camera_0_yaw': LaunchConfiguration('camera_0_yaw'),
            'camera_1_name': 'camera 1',
            'camera_1_frame': 'camera_nvblox_1_link',
            'camera_1_x': LaunchConfiguration('camera_1_x'),
            'camera_1_y': LaunchConfiguration('camera_1_y'),
            'camera_1_z': LaunchConfiguration('camera_1_z'),
            'camera_1_roll': LaunchConfiguration('camera_1_roll'),
            'camera_1_pitch': LaunchConfiguration('camera_1_pitch'),
            'camera_1_yaw': LaunchConfiguration('camera_1_yaw'),
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rsp,
        declare_rviz,
        declare_parent_frame,
        declare_tf_time_offset,
        *declare_camera_tf_args,
        robot_state_publisher,
        camera_transform_tuning_node,
        rviz_node,
    ])
