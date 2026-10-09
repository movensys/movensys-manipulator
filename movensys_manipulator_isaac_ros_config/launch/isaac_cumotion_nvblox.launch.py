import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    camera_tf_arg_defaults = {
        "camera_0_x": "-1.194",
        "camera_0_y": "-0.264",
        "camera_0_z": "0.863",
        "camera_0_roll": "0.127",
        "camera_0_pitch": "0.460",
        "camera_0_yaw": "0.487",
        "camera_1_x": "-0.117",
        "camera_1_y": "-1.462",
        "camera_1_z": "0.791",
        "camera_1_roll": "0.024",
        "camera_1_pitch": "0.344",
        "camera_1_yaw": "1.753",
    }

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock (/clock)"
    )
    declare_num_cameras = DeclareLaunchArgument(
        'num_cameras', default_value='2', choices=['1', '2'],
        description='Use camera_0 only (1), or camera_0 and camera_1 (2).'
    )

    declare_rsp = DeclareLaunchArgument(
        "rsp",
        default_value="true",
        description="Publish /robot_description via isaac_cumotion.launch.py. Set false "
                    "when a backend launch (Gazebo sim or wmx_r2_control) already publishes it."
    )
    declare_publish_camera_tf = DeclareLaunchArgument(
        "publish_camera_tf",
        default_value="true",
        description="Publish static camera TF. Set false while tuning camera TF with GUI."
    )
    declare_enable_hybrid_planning = DeclareLaunchArgument(
        "enable_hybrid_planning",
        default_value="false",
        description="Start MoveIt Hybrid Planning components"
    )
    declare_hybrid_local_solution_topic = DeclareLaunchArgument(
        "hybrid_local_solution_topic",
        default_value="/joint_trajectory",
        description="JointTrajectory topic published by the hybrid local planner"
    )
    declare_enable_nvblox_planning_scene = DeclareLaunchArgument(
        "enable_nvblox_planning_scene",
        default_value="true",
        description="Convert nvblox ESDF to PlanningScene OctoMap"
    )
    declare_read_esdf_world = DeclareLaunchArgument(
        "read_esdf_world",
        default_value="true",
        description="Let cuMotion read nvblox ESDF directly for global planning."
    )
    camera_tf_args = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in camera_tf_arg_defaults.items()
    ]

    pkg_movensys_isaac_ros_config = get_package_share_directory(
        'movensys_manipulator_isaac_ros_config')
    pkg_movensys_manipulator_perception = get_package_share_directory(
        'movensys_manipulator_perception')

    nvblox_rviz = os.path.join(pkg_movensys_isaac_ros_config, 'rviz', 'nvblox.rviz')

    camera_nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_movensys_manipulator_perception, 'launch', 'camera_nvblox.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_camera_tf': LaunchConfiguration('publish_camera_tf'),
            'num_cameras': LaunchConfiguration('num_cameras'),
            **{
                name: LaunchConfiguration(name)
                for name in camera_tf_arg_defaults
            },
        }.items(),
    )

    cumotion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_movensys_isaac_ros_config, 'launch', 'isaac_cumotion.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'read_esdf_world': LaunchConfiguration('read_esdf_world'),
            'rviz_config': nvblox_rviz,
            'rsp': LaunchConfiguration('rsp'),
            'enable_hybrid_planning': LaunchConfiguration('enable_hybrid_planning'),
            'hybrid_local_solution_topic': LaunchConfiguration('hybrid_local_solution_topic'),
        }.items()
    )

    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_movensys_isaac_ros_config, 'launch', 'isaac_nvblox.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'num_cameras': LaunchConfiguration('num_cameras'),
        }.items()
    )

    nvblox_planning_scene_bridge = Node(
        package='movensys_manipulator_moveit_config',
        executable='nvblox_esdf_to_planning_scene',
        name='nvblox_esdf_to_planning_scene',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_nvblox_planning_scene')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'esdf_service_name': '/nvblox_node/get_esdf_and_gradient',
            'planning_scene_topic': '/planning_scene',
            'frame_id': 'world_manipulator',
            'publish_rate': 5.0,
            'occupied_distance': 0.0,
            'octomap_resolution': 0.03,
            'update_esdf': True,
            'visualize_esdf': False,
            'max_occupied_voxels': 20000,
            'aabb_min_m': [-1.26, -1.26, -0.1],
            'aabb_size_m': [2.52, 2.52, 1.5],
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_num_cameras,
        declare_rsp,
        declare_publish_camera_tf,
        declare_enable_hybrid_planning,
        declare_hybrid_local_solution_topic,
        declare_enable_nvblox_planning_scene,
        declare_read_esdf_world,
        *camera_tf_args,
        camera_nvblox,
        cumotion_launch,
        nvblox_launch,
        nvblox_planning_scene_bridge,
    ])
