"""[2026-10-27 사용자 "조그에서는 입력성형 빼고 무브잇만으로 조그하도록 하자. 목표 이동만 입력성형 적용"] GUI 조그용 MoveIt Servo.
동료 MoveIt 설정 (movensys_manipulator_moveit_config, dobot_cr3a) 의 로봇 형상·충돌 설정 (SRDF)·IK (KDL)·속도 한계를 그대로 쓰고 세 가지만 바꿈:
  1) 공구 끝 링크 gui_tip (Link6 + z TIP_M = cr3a_model, 그리퍼 손가락 끝) 을 붙이고 팔 그룹 사슬 끝 Link6 → gui_tip.
     Servo 2.12.4 는 사슬 끝 둘레로 돌고 (공구 끝 점 둘레 회전), 사슬 시작 (받침 Link0) 이 명령 기준 좌표.
  2) 관절 위치 범위 = 계획기 URDF (cr3_robot_tool.urdf: J1·J2·J4~J6 ±360°, J3 ±155° — 사용자 결정 10-27 "계획기 그대로").
     동료 URDF 는 더 좁음 (J2 −90~63°, J3 −146~11.5°, J5 −5.7~172° …) — robot_description_planning 의 min/max_position 으로 덮어씀.
  3) Servo 설정 = 이 폴더 cr3a_servo.yaml.
동료 모델은 우리 모델 (cuRobo·GUI) 과 J1 높이만 6.5 mm 다르고 관절 축·방향은 같음 (urdf_compare 2026-10-27) → 움직이는 방향은 같음.
실행: rt_server 가 R2 를 켠 뒤 `ros2 launch <이 파일>` (같은 컨테이너·ROS 환경), 끌 때 R2 보다 먼저 끔."""
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import cr3a_model as cm  # noqa: E402  (TIP_M — GUI·서버와 같은 공구 끝)

TIP_LINK = 'gui_tip'
OUR_URDF = cm.URDF
# 조그 관절 가속 한계 rad/s² (동료 joint_limits.yaml 2.0 → 1.0): Servo 는 위치만 25 ms 마다 보내 팔 제어기가 점 사이를 곧게 잇는데, 시작·멈춤
#  (가속 한계로 속도를 바꾸는 구간) 에서 점마다 속도가 꺾여 R2 가 1 ms 로 나눈 명령의 엔진 가속이 2.0 에서 9.9~13.0 rad/s² (R2 멈춤 기준
#  15 — 넘으면 MaxAccError → 비상정지 Final). 2026-10-27 WMX 시뮬레이터 1~3 차. 꺾임은 가속 한계에 비례 → 절반으로
SERVO_ACC = 1.0


def generate_launch_description():
    cfg = Path(get_package_share_directory('movensys_manipulator_moveit_config')) / 'config' / 'dobot_cr3a'
    urdf = subprocess.run(['xacro', str(cfg / 'movensys_manipulator.urdf.xacro')], check=True, capture_output=True, text=True).stdout
    tip = (f'<link name="{TIP_LINK}"/><joint name="{TIP_LINK}_joint" type="fixed"><parent link="Link6"/>'
           f'<child link="{TIP_LINK}"/><origin xyz="0 0 {cm.TIP_M}" rpy="0 0 0"/></joint>')
    assert urdf.count('</robot>') == 1
    urdf = urdf.replace('</robot>', tip + '</robot>')
    # 그리퍼 손가락 (직선 관절) 은 R2 /joint_states 에 없음 → 고정으로 (없으면 Servo 장면이 "로봇 상태를 다 모름" 을 1 초마다 경고)
    for j in ('picker_1_joint', 'picker_2_joint'):
        a = f'<joint name="{j}" type="prismatic">'
        assert urdf.count(a) == 1, f'{j} 정의가 바뀜'
        urdf = urdf.replace(a, f'<joint name="{j}" type="fixed">')
    srdf = (cfg / 'movensys_manipulator.srdf').read_text()
    assert srdf.count('tip_link="Link6"') == 1, 'SRDF 팔 그룹 사슬 끝이 Link6 하나가 아님'
    srdf = srdf.replace('tip_link="Link6"', f'tip_link="{TIP_LINK}"')
    limits = yaml.safe_load((cfg / 'joint_limits.yaml').read_text())
    ours = {j.get('name'): j.find('limit') for j in ET.parse(OUR_URDF).getroot().findall('joint') if j.find('limit') is not None}
    for name, d in limits['joint_limits'].items():
        lim = ours[name]
        d.update(has_position_limits=True, min_position=float(lim.get('lower')), max_position=float(lim.get('upper')),
                 has_acceleration_limits=True, max_acceleration=SERVO_ACC)
    kin = yaml.safe_load((cfg / 'kinematics.yaml').read_text())
    servo = yaml.safe_load((HERE / 'cr3a_servo.yaml').read_text())
    # 가속도 제한 필터 (AccelerationLimitedPlugin) 의 파라미터 — 노드 최상위 이름 (online_signal_smoothing 파라미터)
    smooth = {'update_period': servo['publish_period'], 'planning_group_name': servo['move_group_name']}
    return LaunchDescription([
        Node(package='moveit_servo', executable='servo_node', name='servo_node', output='screen',
             parameters=[{'moveit_servo': servo}, smooth, {'robot_description': urdf}, {'robot_description_semantic': srdf},
                         {'robot_description_kinematics': kin}, {'robot_description_planning': limits}]),
    ])
