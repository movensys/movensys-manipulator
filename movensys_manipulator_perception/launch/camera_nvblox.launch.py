import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock (/clock)"
    )
    publish_camera_tf_arg = DeclareLaunchArgument(
        "publish_camera_tf", default_value="true",
        description="Publish static camera TF. Set false while tuning camera TF with GUI."
    )
    camera_tf_args = [
        DeclareLaunchArgument("camera_0_x", default_value="-0.402"),
        DeclareLaunchArgument("camera_0_y", default_value="-0.149"),
        DeclareLaunchArgument("camera_0_z", default_value="0.947"),
        DeclareLaunchArgument("camera_0_roll", default_value="-0.076"),
        DeclareLaunchArgument("camera_0_pitch", default_value="1.017"),
        DeclareLaunchArgument("camera_0_yaw", default_value="0.696"),
        DeclareLaunchArgument("camera_1_x", default_value="0.417"),
        DeclareLaunchArgument("camera_1_y", default_value="-0.255"),
        DeclareLaunchArgument("camera_1_z", default_value="0.879"),
        DeclareLaunchArgument("camera_1_roll", default_value="0.074"),
        DeclareLaunchArgument("camera_1_pitch", default_value="0.867"),
        DeclareLaunchArgument("camera_1_yaw", default_value="2.402"),
    ]

    pkg_share = get_package_share_directory('movensys_manipulator_perception')
    manipulator_model = os.environ.get('MANIPULATOR_MODEL', 'dobot_cr3a')
    realsense_config_0 = os.path.join(pkg_share, 'config', manipulator_model, 'realsense_nvblox_0.yaml')
    realsense_config_1 = os.path.join(pkg_share, 'config', manipulator_model, 'realsense_nvblox_1.yaml')

    camera_nvblox_0_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_0',
        namespace='camera_nvblox_0',
        parameters=[realsense_config_0],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        remappings=[
            ('realsense2_camera_0/color/image_raw', '/image_nvblox_0/rgb'),
            ('realsense2_camera_0/color/camera_info', '/image_nvblox_0/camera_info'),
            ('realsense2_camera_0/aligned_depth_to_color/image_raw', '/image_nvblox_0/depth'),
            ('realsense2_camera_0/aligned_depth_to_color/camera_info',
             '/image_nvblox_0/depth/camera_info'),
        ],
    )

    camera_nvblox_1_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera_1',
        namespace='camera_nvblox_1',
        parameters=[realsense_config_1],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        remappings=[
            ('realsense2_camera_1/color/image_raw', '/image_nvblox_1/rgb'),
            ('realsense2_camera_1/color/camera_info', '/image_nvblox_1/camera_info'),
            ('realsense2_camera_1/aligned_depth_to_color/image_raw', '/image_nvblox_1/depth'),
            ('realsense2_camera_1/aligned_depth_to_color/camera_info',
             '/image_nvblox_1/depth/camera_info'),
        ],
    )

    start_camera_0_nvblox_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "--frame-id", "world_manipulator",
            "--child-frame-id", "camera_nvblox_0_link",
            "--x", LaunchConfiguration("camera_0_x"),
            "--y", LaunchConfiguration("camera_0_y"),
            "--z", LaunchConfiguration("camera_0_z"),
            "--roll", LaunchConfiguration("camera_0_roll"),
            "--pitch", LaunchConfiguration("camera_0_pitch"),
            "--yaw", LaunchConfiguration("camera_0_yaw"),
        ],
    )

    start_camera_1_nvblox_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        condition=IfCondition(LaunchConfiguration("publish_camera_tf")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=[
            "--frame-id", "world_manipulator",
            "--child-frame-id", "camera_nvblox_1_link",
            "--x", LaunchConfiguration("camera_1_x"),
            "--y", LaunchConfiguration("camera_1_y"),
            "--z", LaunchConfiguration("camera_1_z"),
            "--roll", LaunchConfiguration("camera_1_roll"),
            "--pitch", LaunchConfiguration("camera_1_pitch"),
            "--yaw", LaunchConfiguration("camera_1_yaw"),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        publish_camera_tf_arg,
        *camera_tf_args,
        camera_nvblox_0_node,
        camera_nvblox_1_node,
        start_camera_0_nvblox_transform,
        start_camera_1_nvblox_transform,
    ])
