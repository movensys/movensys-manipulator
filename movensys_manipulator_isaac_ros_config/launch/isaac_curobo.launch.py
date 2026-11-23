import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    manipulator_model = os.environ.get('MANIPULATOR_MODEL', 'dobot_cr3a')
    curobo_v2_path = os.environ.get('CUROBO_V2_PATH', '/opt/curobo_v2')

    pkg_movensys_isaac_ros_config = get_package_share_directory(
        'movensys_manipulator_isaac_ros_config')
    pkg_movensys_manipulator_description = get_package_share_directory(
        'movensys_manipulator_description')

    config_file = os.path.join(
        pkg_movensys_isaac_ros_config, 'config', manipulator_model,
        'curobo.yaml')
    rviz_file = os.path.join(
        pkg_movensys_isaac_ros_config, 'rviz', 'curobo.rviz')
    xacro_file = os.path.join(
        pkg_movensys_manipulator_description, 'urdf', manipulator_model,
        'movensys_manipulator.xacro')
    xrdf_file = os.path.join(
        pkg_movensys_manipulator_description, 'urdf', manipulator_model,
        'movensys_manipulator.xrdf')

    robot_description = xacro.process_file(xacro_file).toxml()
    urdf_file = os.path.join(tempfile.gettempdir(), 'movensys_manipulator.urdf')
    with open(urdf_file, 'w') as file:
        file.write(robot_description)

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (/clock)'
    )
    with open(config_file) as f:
        params = yaml.safe_load(f)['curobo']['ros__parameters']

    camera_nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('movensys_manipulator_perception'), 
            'launch', 'camera_nvblox.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
        ],
    )

    curobo = Node(
        package='movensys_manipulator_isaac_ros_config',
        executable='curobo_node.py',
        name='curobo',
        output='screen',
        parameters=[
            config_file,
            {'urdf_path': urdf_file, 'xrdf_path': xrdf_file, 'use_sim_time': use_sim_time},
        ],
        additional_env={'PYTHONPATH': curobo_v2_path + ':' + os.environ.get('PYTHONPATH', '')},
    )

    goal_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='goal_state_publisher',
        output='log',
        parameters=[
            {'robot_description': robot_description, 'frame_prefix': 'goal/'},
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('joint_states', params['goal_joint_state_topic']),
                    ('robot_description', '/curobo/goal_robot_description')],
    )
    goal_root_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='goal_root_tf',
        output='log',
        arguments=['--frame-id', params['base_frame'],
                   '--child-frame-id', 'goal/' + params['base_frame']],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        camera_nvblox,
        robot_state_publisher,
        goal_state_publisher,
        goal_root_tf,
        curobo,
        rviz,
    ])
