"""Launcher for the YOLO OBB cube detector node.

Loads defaults from config/$MANIPULATOR_MODEL/yolo_cube_detector.yaml. The `model_path`
argument defaults to the cubes_obb.pt weights shipped inside this
package's models/ directory; override to point at a different .pt
file or an OpenVINO export.

The hand camera is brought up alongside the detector so this file runs
standalone. Pass `launch_camera:=false` when the camera is already
running — that is what yolo_dice_and_cube_detector.launch.py does, since
it starts one shared camera for both detectors.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('movensys_manipulator_perception')
    manipulator_model = os.environ.get('MANIPULATOR_MODEL', 'dobot_cr3a')
    default_params = os.path.join(pkg_share, 'config', manipulator_model, 'yolo_cube_detector.yaml')
    default_model = os.path.join(pkg_share, 'models', 'cubes_obb-monoplyboard_background.pt')

    declared_arguments = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML parameter file for the YOLO cube detector',
        ),
        DeclareLaunchArgument(
            'model_path', default_value=default_model,
            description='Path to OpenVINO model dir or .pt weights file',
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'launch_camera', default_value='true',
            description='Also start the hand camera. Set false when the '
                        'camera is already running, or it will be started twice.',
        ),
    ]

    # Skipped under use_sim_time — the simulator supplies the image stream,
    # so there is no physical RealSense to open.
    camera_hand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'camera_hand.launch.py')
        ),
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('launch_camera'), "' == 'true' and '",
            LaunchConfiguration('use_sim_time'), "' != 'true'",
        ])),
    )

    detector_node = Node(
        package='movensys_manipulator_perception',
        executable='yolo_cube_detector.py',
        name='yolo_cube_detector',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'model_path': LaunchConfiguration('model_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription(declared_arguments + [camera_hand_launch, detector_node])
