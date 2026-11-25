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

    pkg_share = get_package_share_directory('movensys_manipulator_perception')
    manipulator_model = os.environ.get('MANIPULATOR_MODEL', 'dobot_cr3a')
    realsense_config = os.path.join(pkg_share, 'config', manipulator_model, 'realsense_top.yaml')

    camera_top_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='camera_top',
        parameters=[realsense_config],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        remappings=[
            ('realsense2_camera/color/image_raw', '/image_top/rgb'),
            ('realsense2_camera/color/camera_info', '/image_top/camera_info'),
        ],
    )

    start_camera_top_transform_real = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
        parameters=[{"use_sim_time": False}],
        arguments=[
            "--frame-id", "world_manipulator",
            "--child-frame-id", "camera_top_link",
            "--x", "0.27",
            "--y", "0.0",
            "--z", "0.7",
            "--roll", "0.0",
            "--pitch", "1.5708",
            "--yaw", "0.0",
        ],
    )

    start_camera_top_transform_simulation = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        condition=IfCondition(LaunchConfiguration("use_sim_time")),
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--frame-id", "world_manipulator",
            "--child-frame-id", "camera_top_color_optical_frame",
            "--x", "0.27",
            "--y", "0.0",
            "--z", "0.7",
            "--roll", "3.1416",
            "--pitch", "0.0",
            "--yaw", "-1.5708",
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        camera_top_node,
        start_camera_top_transform_simulation,
        start_camera_top_transform_real,
    ])
