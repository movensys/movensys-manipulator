import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    num_cameras = int(LaunchConfiguration('num_cameras').perform(context))

    pkg_movensys_isaac_ros_config = get_package_share_directory(
        'movensys_manipulator_isaac_ros_config')
    pkg_movensys_description = get_package_share_directory('movensys_manipulator_description')

    manipulator_model = os.environ.get('MANIPULATOR_MODEL', 'dobot_cr3a')

    robot_xrdf = os.path.join(
        pkg_movensys_description, 'urdf', manipulator_model, 'movensys_manipulator.xrdf')
    urdf_path = os.path.join(
        pkg_movensys_description, 'urdf', manipulator_model, 'movensys_manipulator.urdf')

    nvblox_base_config = os.path.join(
        pkg_movensys_isaac_ros_config, 'config', manipulator_model, 'nvblox_movensys_base.yaml')
    workspace_config = os.path.join(
        pkg_movensys_isaac_ros_config, 'config', manipulator_model, 'movensys_sim.yaml')

    manipulation_container = ComposableNodeContainer(
        name='manipulation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=[
            remapping
            for i in range(num_cameras)
            for remapping in [
                (f'/camera_{i}/color/image', f'/image_nvblox_{i}/rgb'),
                (f'/camera_{i}/color/camera_info', f'/image_nvblox_{i}/camera_info'),
                (f'/camera_{i}/depth/image', f'/robot_segmenter/world_depth_{i}'),
                (f'/camera_{i}/depth/camera_info', f'/image_nvblox_{i}/camera_info'),
            ]
        ],
        parameters=[
            nvblox_base_config,
            workspace_config,
            {'num_cameras': num_cameras},
            {'use_sim_time': use_sim_time},
        ]
    )

    load_nvblox = LoadComposableNodes(
        target_container='manipulation_container',
        composable_node_descriptions=[nvblox_node],
    )

    robot_segmenter_config = os.path.join(
        pkg_movensys_isaac_ros_config, 'config', manipulator_model, 'robot_segmenter_movensys.yaml'
    )
    robot_segmenter = Node(
        package='isaac_ros_cumotion',
        executable='robot_segmenter_node',
        name='robot_segmenter_node',
        output='screen',
        parameters=[
            robot_segmenter_config,
            {
                'robot': robot_xrdf,
                'urdf_path': urdf_path,
                'use_sim_time': use_sim_time,
                'num_cameras': num_cameras,
                'depth_image_topics': [
                    f'/image_nvblox_{i}/depth' for i in range(num_cameras)
                ],
                'depth_camera_infos': [
                    f'/image_nvblox_{i}/camera_info' for i in range(num_cameras)
                ],
                'robot_mask_publish_topics': [
                    f'/robot_segmenter/robot_mask_{i}' for i in range(num_cameras)
                ],
                'world_depth_publish_topics': [
                    f'/robot_segmenter/world_depth_{i}' for i in range(num_cameras)
                ],
            }
        ]
    )

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')

    nodes = [
        manipulation_container,
        robot_segmenter,
        load_nvblox,
    ]

    if ros_distro == 'jazzy':
        static_planning_scene = Node(
            package='isaac_ros_cumotion',
            executable='static_planning_scene',
            name='static_planning_scene',
            output='screen',
            parameters=[
                {
                    'robot': robot_xrdf,
                    'urdf_path': urdf_path,
                    'use_sim_time': use_sim_time,
                }
            ]
        )
        nodes.append(static_planning_scene)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock (/clock)'
        ),
        DeclareLaunchArgument(
            'num_cameras', default_value='2', choices=['1', '2'],
            description='Use camera_0 only (1), or camera_0 and camera_1 (2).'
        ),
        OpaqueFunction(function=launch_setup),
    ])
