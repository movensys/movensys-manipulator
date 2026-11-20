"""[2026-10-26] 실제 로봇 엔코더 읽기 전용 R2 노드 (cr3a_gui/wmx_r2.sh encoders 가 띄움).

R2 일반 노드 (wmx_r2_general_nodes.launch.py) 에서 엔진·축 상태만: wmx_engine_node (엔진 켜기 → CR3A 파라미터 넣기 → 통신 시작),
wmx_core_motion_node (/wmx/axes/status 100 Hz: 실제 위치·서보·알람), 둘을 켜 주는 wmx_lifecycle_manager_node.
빼는 것 (서보를 켜거나 출력을 쓰는 노드): joint_state_broadcaster (켜질 때 알람 해제 → 서보 ON), ros2_control, 제어기, 그리퍼, IO.
core_motion 은 서비스가 불릴 때만 축을 움직임 — 이 launch 를 쓰는 쪽은 서비스를 부르지 않는다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def nodes(context):
    xml = LaunchConfiguration('wmx_param_file').perform(context)
    return [
        Node(package='wmx_r2_package', executable='wmx_engine_node', name='wmx_engine_node', output='screen', emulate_tty=True,
             parameters=[{'core': -1, 'affinity_mask': 0, 'wmx_param_file_path': xml}]),
        Node(package='wmx_r2_package', executable='wmx_lifecycle_manager_node', name='wmx_lifecycle_manager_node', output='screen',
             emulate_tty=True, parameters=[{'managed_nodes': ['wmx_core_motion_node'], 'discovery_period': 1.0}]),
        LifecycleNode(package='wmx_r2_package', executable='wmx_core_motion_node', name='wmx_core_motion_node', namespace='',
                      output='screen', emulate_tty=True,
                      parameters=[{'axes_status_rate': 100, 'motion_controllers': ['']}]),     # '' = 지켜볼 제어기 없음
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wmx_param_file', description='CR3A WMX 파라미터 XML (단위 rad = 기어비)'),
        OpaqueFunction(function=nodes),
    ])
