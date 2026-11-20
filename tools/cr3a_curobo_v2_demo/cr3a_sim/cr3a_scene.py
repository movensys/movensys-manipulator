# [만듦 2026-10-13] CR3A 장애물 회피 시뮬레이션 — 공용 설정과 계산 (plan_avoid.py, make_report.py 가 같이 씀)
#   좌표: cuRobo 기준 링크 dummy_link (= Dobot base_link, 로봇 받침 바닥 중심). 단위 m, rad. 쿼터니언은 wxyz.
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROBOT_YML = HERE / 'cr3a_curobo.yml'          # build_model.py 가 만듦
URDF = HERE / 'dobot_rviz' / 'urdf' / 'cr3_robot.urdf'
JOINTS = [f'joint{i}' for i in range(1, 7)]   # R2 는 이름이 아니라 순서로 축 0..5 에 넣는다 → 항상 이 순서
TOOL = 'Link6'

# 장면 (2026-10-13 사용자 결정: "제가 기본 장면 구성")
#   바닥: 윗면 z = 0 (로봇 받침과 같은 높이). base_link 구는 z >= 0.04 라서 활성 거리(ACT)보다 멀다 (build_model.py)
#   상자: A 와 B 사이에 세운 10x10x30 cm 기둥. 관절을 직선으로 보간하면 부딪힌다 (plan_avoid.py 가 확인)
FLOOR = {'dims': [2.0, 2.0, 0.02], 'pose': [0.0, 0.0, -0.01, 1, 0, 0, 0]}
BOX = {'dims': [0.10, 0.10, 0.30], 'pose': [0.42, 0.0, 0.15, 1, 0, 0, 0]}
SCENE = {'cuboid': {'floor': FLOOR, 'box': BOX}}

# 시작 A / 목표 B 의 도구(Link6) 위치. 방향은 Dobot SRDF home 자세의 도구 방향 (아래를 봄) — plan_avoid.py 가 FK 로 구함
#   첫 판 (0.40, ±0.20) 은 B 자세 자체가 상자에서 0.6 mm (CR3A 손목이 옆으로 ~13 cm 비켜 있어 팔뚝이 상자 쪽으로 붙음)
#   → 후보 12개 시험 (2026-10-13): (0.42, ±0.34) 이 끝 자세 거리 A 81.6 / B 58.6 mm, 직선 보간 -72 mm (65/101 점 충돌)
A_POS = [0.42, 0.34, 0.20]
B_POS = [0.42, -0.34, 0.20]
END_CLEARANCE_MIN = 0.03   # A, B 자세가 상자에서 이만큼은 떨어져야 함 (아니면 장면이 잘못된 것 → 중단)
HOME = [0.0, 0.6072, -1.7223, -0.2949, 1.6134, 0.0]   # Dobot cr3_moveit/config/cr3_robot.srdf group_state "home"

# cuRobo 계획 설정 (Mirobot 에서 고른 값을 출발점으로: curobo_v2_작업기록.md 4장)
ACT = 0.02            # 충돌 활성 거리 (m). Mirobot 0.01 은 기둥을 ~15 mm 로 스침 → 회피가 눈에 보이게 2 cm
POS_TOL = 0.0005      # 0.5 mm (라이브러리 기본 5 mm 는 3~4.5 mm 모자란 채 성공 처리됨)
ORI_TOL = 0.005
IK_SEEDS = 32
TRAJOPT_SEEDS = 6


def quat_to_rot(q):
    """wxyz → 3x3 회전 행렬 (list)"""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def box_clearance(centers, radii, box=None):
    """구 중심 (..., 3) 과 반지름 (...) → 상자 표면까지 거리 - 반지름 (m). 음수 = 파고듦.
    [바꿈 2026-10-13] 상자 회전(pose 의 wxyz)도 반영 (랜덤 시험에서 상자를 돌려 놓음)"""
    import torch
    box = box or BOX
    c = torch.tensor(box['pose'][:3], device=centers.device, dtype=centers.dtype)
    R = torch.tensor(quat_to_rot(box['pose'][3:7]), device=centers.device, dtype=centers.dtype)
    h = torch.tensor(box['dims'], device=centers.device, dtype=centers.dtype) / 2
    local = (centers - c) @ R            # 상자 좌표계로 (R 의 열 = 상자 축)
    d = local.abs() - h
    outside = torch.clamp(d, min=0).norm(dim=-1)
    inside = torch.clamp(d.max(dim=-1).values, max=0)
    return outside + inside - radii


def floor_clearance(centers, radii, floor=FLOOR):
    top = floor['pose'][2] + floor['dims'][2] / 2
    return centers[..., 2] - radii - top


def clearances(kin, q, box=None):
    """관절 궤적 q (T, 6, JOINTS 순서) → 단계별 장애물/바닥 최소 거리(m)와 장애물에 가장 가까운 링크.
    kin: curobo Kinematics. box: 상자 dict 하나 또는 [dict, ...] (여러 개면 가장 가까운 것), 없으면 BOX.
    반환 dict of lists (CPU). 바닥은 고정된 base_link 를 뺀다."""
    import torch
    from curobo.types import JointState
    q = torch.as_tensor(q, device='cuda', dtype=torch.float32).view(-1, 6)
    js = JointState.from_position(q, joint_names=JOINTS)
    st = kin.compute_kinematics(js)
    s = st.robot_spheres.reshape(q.shape[0], -1, 4)            # [T, 구 개수, xyz r]
    names, link_of = sphere_links(kin)
    valid = s[..., 3] > 0
    boxes = box if isinstance(box, list) else [box or BOX]
    cb = torch.stack([box_clearance(s[..., :3], s[..., 3], b) for b in boxes]).min(dim=0).values
    cb = cb.masked_fill(~valid, float('inf'))
    not_base = torch.tensor([names[i] != 'base_link' for i in link_of], device=s.device)
    cf = floor_clearance(s[..., :3], s[..., 3]).masked_fill(~(valid & not_base), float('inf'))
    mb, ib = cb.min(dim=-1)
    mf = cf.min(dim=-1).values
    return {'box': mb.cpu().tolist(), 'box_link': [names[link_of[i]] for i in ib.cpu().tolist()],
            'floor': mf.cpu().tolist()}


def sphere_links(kin):
    """구 번호 → 링크 이름. (링크 이름 목록, 구마다 링크 번호)"""
    kc = kin.kinematics_config
    idx2name = {v: k for k, v in kc.link_name_to_idx_map.items()}
    m = kc.link_sphere_idx_map.view(-1).cpu().tolist()
    names = sorted(set(idx2name[i] for i in m), key=lambda n: m.index(kc.link_name_to_idx_map[n]))
    return names, [names.index(idx2name[i]) for i in m]


def spheres_local(kin):
    """링크 좌표계의 충돌 구 (뷰어용): {링크: [[x, y, z, r], ...]}"""
    import yaml
    d = yaml.safe_load(open(ROBOT_YML))['kinematics']['collision_spheres']
    return {ln: [list(sp['center']) + [sp['radius']] for sp in v] for ln, v in d.items()}


def make_kinematics():
    from curobo.kinematics import Kinematics, KinematicsCfg
    return Kinematics(KinematicsCfg.from_robot_yaml_file(str(ROBOT_YML)))


def tool_pose(kin, q):
    """q (6,) → (xyz, wxyz)"""
    import torch
    from curobo.types import JointState
    st = kin.compute_kinematics(JointState.from_position(
        torch.tensor([q], device='cuda', dtype=torch.float32), joint_names=JOINTS))
    p = st.tool_poses.get_link_pose(TOOL)
    return p.position.view(-1).tolist(), p.quaternion.view(-1).tolist()


def deg(x):
    return [round(math.degrees(v), 2) for v in x]
