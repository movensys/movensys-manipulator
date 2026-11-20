#!/usr/bin/env python3
# [만듦 2026-10-13] Dobot CR3A 공식 URDF(cr3_robot.urdf) + 메시 → cuRoboV2 로봇 설정 cr3a_curobo.yml
#   실행 (wmx_r2 컨테이너 안, admin):  cd ~/workspaces/movensys_ws/cr3a_sim && python3 build_model.py
#   하는 일
#     1) 링크 메시마다 충돌 구를 맞춘다 (cuRoboV2 RobotBuilder, MorphIt)
#        - base_link 구는 z >= BASE_CLIP 로 자른다: 바닥 상자의 활성 거리 안에 고정 구가 있으면 지울 수 없는 비용이 생겨
#          계획이 전부 실패한다 (Mirobot 때 겪음: curobo_v2_작업기록.md / step2_make_xrdf.py)
#     2) 자기 충돌을 무시할 링크 짝 계산 (붙어 있는 짝 + 샘플링으로 절대 안 닿는 짝) + Dobot SRDF 의 disable_collisions 추가
#     3) 관절 한계 넣기 (아래 LIMITS 설명)
#     4) yml 저장 + 링크별 구 개수 출력
#   URDF 는 고치지 않는다. 메시 경로 package://dobot_rviz/meshes/cr3/... 는 cuRobo 가 package:// 를 떼고 asset_path 뒤에 붙인다.
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
URDF = HERE / 'dobot_rviz' / 'urdf' / 'cr3_robot.urdf'
ASSET = HERE                      # dobot_rviz/meshes/cr3/*.STL 의 기준 폴더
OUT = HERE / 'cr3a_curobo.yml'
TOOL = 'Link6'
BASE_CLIP = 0.02                  # base_link 구의 아래 끝 (m). 바닥 윗면(scene.py FLOOR_TOP)보다 활성 거리 이상 높게

# LIMITS (2026-10-13 사용자 결정 과정, wmx_r2_작업기록.md 9장)
#   속도: URDF 그대로 (J1,J2 3.14 / J3~J6 3.89 rad/s = Dobot cr3_moveit joint_limits.yaml max_velocity 와 같음)
#   가속: Dobot 공식 ROS2 패키지 cr3_moveit/config/joint_limits.yaml 의 max_acceleration = 속도 × 2 (6.28 / 7.78 rad/s²)
#   저크: Dobot 자료에 없음 → 사용자 제안 "가속 × 10" (62.8 / 77.8 rad/s³)
ACC_PER_VEL = 2.0
JERK_PER_ACC = 10.0
# Dobot cr3_moveit/config/cr3_robot.srdf 의 disable_collisions (Adjacent / Never)
SRDF_DISABLE = [('Link1', 'Link2'), ('Link1', 'Link4'), ('Link1', 'base_link'), ('Link2', 'Link3'),
                ('Link3', 'Link4'), ('Link4', 'Link5'), ('Link4', 'Link6'), ('Link5', 'Link6')]
# Dobot SRDF 의 home 자세 (계획기의 기본 자세로 씀)
HOME = [0.0, 0.6072, -1.7223, -0.2949, 1.6134, 0.0]


def urdf_limits(path):
    root = ET.parse(path).getroot()
    out = {}
    for j in root.findall('joint'):
        lim = j.find('limit')
        if j.get('type') in ('revolute', 'prismatic') and lim is not None:
            out[j.get('name')] = (float(lim.get('lower')), float(lim.get('upper')), float(lim.get('velocity')))
    return out


def main():
    from curobo.robot_builder import RobotBuilder

    lims = urdf_limits(URDF)
    names = [f'joint{i}' for i in range(1, 7)]
    vel = [lims[n][2] for n in names]
    acc = [round(v * ACC_PER_VEL, 4) for v in vel]
    jerk = [round(a * JERK_PER_ACC, 3) for a in acc]
    print('URDF 관절 한계 (아래, 위, 속도):', {n: lims[n] for n in names})
    print('가속:', acc, ' 저크:', jerk)

    b = RobotBuilder(urdf_path=str(URDF), asset_path=str(ASSET), tool_frames=[TOOL])
    spheres = b.fit_collision_spheres(sphere_density=1.0, compute_metrics=True,
                                      clip_links={'base_link': ('z', BASE_CLIP)})
    ignore = b.compute_collision_matrix(num_samples=1000)
    cfg = b.build()
    b.save(cfg, str(OUT))

    d = yaml.safe_load(open(OUT))
    k = d['kinematics']
    cs = k.setdefault('cspace', {})
    order = cs.get('joint_names', names)
    if list(order) != names:
        print('주의: cspace 관절 순서가 joint1..6 이 아님:', order)
    idx = [names.index(n) for n in order]
    cs['max_acceleration'] = [acc[i] for i in idx]
    cs['max_jerk'] = [jerk[i] for i in idx]
    cs['default_joint_position'] = [HOME[i] for i in idx]
    # SRDF 의 무시 짝을 더한다 (양방향)
    sci = k.setdefault('self_collision_ignore', {}) or {}
    added = 0
    for a, c in SRDF_DISABLE:
        for x, y in ((a, c), (c, a)):
            lst = sci.setdefault(x, []) or []
            if y not in lst:
                lst.append(y); added += 1
            sci[x] = lst
    k['self_collision_ignore'] = sci
    yaml.safe_dump(d, open(OUT, 'w'), sort_keys=False)

    sph = k.get('collision_spheres', {})
    print(f'\n저장: {OUT}')
    print('기준 링크:', k.get('base_link'), '/ 도구 프레임:', k.get('tool_frames'))
    print('충돌 링크:', k.get('collision_link_names'))
    print('링크별 구 개수:', {l: len(v) for l, v in sph.items()}, '합계', sum(len(v) for v in sph.values()))
    base_low = min((s['center'][2] - s['radius'] for s in sph.get('base_link', [])), default=float('nan'))
    print(f'base_link 구의 가장 낮은 점 z = {base_low:.4f} m (목표 >= {BASE_CLIP})')
    print('자기 충돌 무시 짝 (SRDF 에서 더한 것', added, '개):')
    for l, v in sci.items():
        print('  ', l, '→', v)
    print('cspace:', {kk: cs[kk] for kk in ('joint_names', 'max_acceleration', 'max_jerk', 'default_joint_position') if kk in cs})


if __name__ == '__main__':
    sys.exit(main())
