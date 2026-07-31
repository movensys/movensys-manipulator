import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    PopLaunchConfigurations,
    PushLaunchConfigurations,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('movensys_manipulator_perception')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock (/clock)',
        ),
    ]

    # One camera shared by both detectors, so the cube launcher's own
    # camera_hand include is suppressed below via launch_camera:=false.
    camera_hand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'camera_hand.launch.py')
        ),
        condition=UnlessCondition(use_sim_time),
    )

    yolo_cube_launch = GroupAction([
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'yolo_cube_detector.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'launch_camera': 'false',
            }.items(),
        ),
        PopLaunchConfigurations(),
    ])

    yolo_dice_launch = GroupAction([
        PushLaunchConfigurations(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'yolo_dice_detector.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        PopLaunchConfigurations(),
    ])

    return LaunchDescription(declared_arguments + [
        camera_hand_launch,
        yolo_cube_launch,
        yolo_dice_launch,
    ])
