import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    manipulator_model = os.environ.get('MANIPULATOR_MODEL', 'dobot_cr3a')
    config_file = os.path.join(
        get_package_share_directory('movensys_manipulator_isaac_ros_config'),
        'config', manipulator_model, 'curobo_eef_client.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='movensys_manipulator_isaac_ros_config',
            executable='curobo_eef_client.py',
            name='curobo_eef_client',
            output='screen',
            parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
