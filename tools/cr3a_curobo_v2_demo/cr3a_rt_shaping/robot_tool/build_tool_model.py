#!/usr/bin/env python3
"""[만듦 2026-10-16] 공구 붙은 CR3A (cr3_robot_tool.urdf) 의 cuRoboV2 로봇 설정 → robot_tool/cr3a_tool_curobo.yml

실행 (wmx_r2 컨테이너, admin):
  cd /home/admin/workspaces/movensys_ws/cr3a_rt_shaping/robot_tool && source /opt/wmx_r2_env.sh &&
  PYTHONPATH=/home/admin/workspaces/movensys_ws/curobo_v2_shaping:$PYTHONPATH python3 build_tool_model.py [구 개수 = 8]
하는 일
  1) 팔 (base_link ~ Link6) 은 cr3a_sim/cr3a_curobo.yml 그대로 — 구 60 개·관절 한계·자기 충돌 무시 짝이 지금까지 실행과 같게.
  2) 공구 구 = cover_spheres.py 의 '공구 전체를 빈틈없이 덮는 큰 구' (사용자 결정 2026-10-16 "그냥 그리퍼를 덮은 몇개의 큰 구로").
     [시도 후 뺌] 부품마다 MorphIt 으로 맞추기: 얇은 브래킷·카메라는 cuRobo 초기화 (경계 상자 격자 중 메시 안 점) 가 안쪽 점을 못 찾아
     표면 2 mm 구로 떨어짐 (메시가 구 밖으로 최대 65·36 mm), 그리퍼만 됨 (14 개, 밖으로 최대 8.7 mm).
  3) 자기 충돌 무시 짝 (공구 링크가 들어간 짝만 더함): 붙어 있는 짝 (Link6·공구끼리) + RobotBuilder.compute_collision_matrix
     (기본 자세에서 겹치는 짝 + 1000 자세 샘플링에서 절대 안 닿는 짝). 팔끼리 짝은 옛 설정 그대로.
  4) 구 품질 (메시 표면 점이 구 밖으로 나간 거리 = MPC·안전 검사가 못 보는 두께) 을 팔 링크와 함께 → tool_spheres.json.
"""
import copy
import json
import sys
from pathlib import Path

import numpy as np
import trimesh
import yaml

HERE = Path(__file__).resolve().parent
URDF = HERE / 'cr3_robot_tool.urdf'
OLD_YML = HERE.parent.parent / 'cr3a_sim' / 'cr3a_curobo.yml'
OUT = HERE / 'cr3a_tool_curobo.yml'
TOOL = ['tool_bracket', 'tool_camera', 'tool_gripper']
ARM = ['base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6']
CPATH = '/home/admin/workspaces/movensys_ws/cr3a_rt_shaping/robot_tool'     # 컨테이너 안 경로 (yml 은 컨테이너에서 읽음)


def mesh_of(link):
    import xml.etree.ElementTree as ET
    root = ET.parse(URDF).getroot()
    ln = [x for x in root.findall('link') if x.get('name') == link][0]
    fn = ln.find('visual/geometry/mesh').get('filename').replace('package://', '')
    return trimesh.load(HERE / fn, force='mesh')


def coverage(link, spheres, n=60000, seed=0):
    """메시 표면 표본 점 → 구 합집합 밖으로 나간 거리 (mm): 나간 점 비율, 99 %, 최대"""
    m = mesh_of(link)
    P, _ = trimesh.sample.sample_surface(m, n, seed=seed)
    C = np.array([s['center'] for s in spheres])
    R = np.array([s['radius'] for s in spheres])
    d = np.min(np.linalg.norm(P[:, None, :] - C[None], axis=-1) - R[None], axis=1)
    out = d[d > 0]
    return {'n_spheres': len(spheres), 'outside_frac': round(float((d > 0).mean()), 4),
            'outside_p99_mm': round(float(np.quantile(np.maximum(d, 0), 0.99)) * 1e3, 2),
            'outside_max_mm': round(float(d.max()) * 1e3, 2) if len(out) else 0.0}


def main():
    from curobo.robot_builder import RobotBuilder
    old = yaml.safe_load(open(OLD_YML))
    k_old = old['kinematics']
    K = sys.argv[1] if len(sys.argv) > 1 else '8'
    cover = json.loads((HERE / 'tool_cover_spheres.json').read_text(encoding='utf-8'))[K]
    new = {ln: [] for ln in TOOL}
    for sp in cover['spheres']:                      # 구 → 그 구가 가장 많이 덮는 부품 링크 (모두 Link6 좌표 = 공구 링크 좌표)
        new[sp['link']].append({'center': sp['center'], 'radius': sp['radius']})
    TOOLS = [ln for ln in TOOL if new[ln]]
    print(f'덮개 구 {K} 개:', {ln: len(v) for ln, v in new.items()}, f"(두께 중앙 {cover['thick_med_mm']} / 최대 {cover['thick_max_mm']} mm)")
    b = RobotBuilder(urdf_path=str(URDF), asset_path=str(HERE), tool_frames=['Link6'])
    b._collision_spheres = {ln: copy.deepcopy(k_old['collision_spheres'][ln]) for ln in ARM}   # 팔 = 옛 구 그대로
    b._collision_spheres.update({ln: new[ln] for ln in TOOLS})
    ignore = b.compute_collision_matrix(num_samples=1000)
    d = copy.deepcopy(old)
    k = d['kinematics']
    k['urdf_path'] = f'{CPATH}/cr3_robot_tool.urdf'
    k['asset_root_path'] = CPATH
    k['collision_link_names'] = list(k['collision_link_names']) + TOOLS
    k['mesh_link_names'] = list(k['mesh_link_names']) + TOOL
    for ln in TOOLS:
        k['collision_spheres'][ln] = new[ln]
        k['self_collision_buffer'][ln] = 0.0
    sci = k['self_collision_ignore']
    added = []
    pairs = set()
    for a, lst in ignore.items():
        for c in lst:
            if a in TOOL or c in TOOL:
                pairs.add(tuple(sorted((a, c))))
    for t in TOOLS:                                  # 공구끼리·Link6 은 늘 붙어 있음 (샘플링 결과와 상관없이)
        pairs.add(tuple(sorted((t, 'Link6'))))
        for u in TOOLS:
            if u != t:
                pairs.add(tuple(sorted((t, u))))
    for a, c in sorted(pairs):
        for x, y in ((a, c), (c, a)):
            lst = sci.setdefault(x, [])
            if y not in lst:
                lst.append(y)
        added.append(f'{a}-{c}')
    yaml.safe_dump(d, open(OUT, 'w'), sort_keys=False, allow_unicode=True)
    checked = sorted({tuple(sorted((t, a))) for t in TOOLS for a in ARM + TOOLS if t != a} - pairs)
    print('공구 무시 짝:', added)
    print('공구 자기 충돌 검사 짝:', [f'{a}-{c}' for a, c in checked])
    allsph = [sp for ln in TOOLS for sp in new[ln]]
    rep = {'cover_k': K, 'tool': {ln: coverage(ln, allsph) for ln in TOOL},       # 부품마다: 공구 구 전체로 덮였나
           'arm': {ln: coverage(ln, k_old['collision_spheres'][ln]) for ln in ARM if ln != 'base_link'},
           'ignore_added': added, 'self_check_pairs': [f'{a}-{c}' for a, c in checked]}
    for grp in ('tool', 'arm'):
        for ln, r in rep[grp].items():
            print(f'  {ln:13s} 구 {r["n_spheres"]:3d} 개, 메시 표면이 구 밖: {r["outside_frac"] * 100:5.1f} % 점, 99 % {r["outside_p99_mm"]:5.2f} mm, 최대 {r["outside_max_mm"]:5.2f} mm')
    (HERE / 'tool_spheres.json').write_text(json.dumps(rep, ensure_ascii=False, indent=1), encoding='utf-8')
    print('저장:', OUT)


if __name__ == '__main__':
    sys.exit(main())
