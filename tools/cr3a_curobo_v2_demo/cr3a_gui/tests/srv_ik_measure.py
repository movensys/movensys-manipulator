"""[2026-10-27 R-1 ④] 공구 끝 IK 오차 재기 (이상적 모드 — R2·WMX 없음, GUI 연결 없이): <폴더>/rt_server.make_controller 로 서버 제어기를
만들고 공구 끝 목표들을 ctl._ik 로 풀어, 고른 해의 정기구학 공구 끝 (ctl._tip) 과 목표의 위치·방향 차를 IK_TOL 과 비교. 같은 목표로 cuRobo
IKSolver 를 직접 불러 '성공' 이라 한 해들의 cuRobo 오차 최대도. [2026-10-28] IK_TOL 은 계획기를 만들 때 넣음 (World(ik_tol=…)) —
만든 뒤 아무도 안 바꾼 설정 값이 IK_TOL 인지 (ik_cfg_is_tol) + 설정 값이 풀 때 쓰이는지 (tolerance_probe) 로 '만들 때 넣은 값이 쓰임' 을 봄.
  python3 tests/srv_ik_measure.py <rt_server 가 있는 폴더> [포트 47875] [결과 json]
목표: 지금 공구 끝 (A 자세) 그대로 · ±3 cm (받침 x y z) · ±10° (받침 x y z 축, 공구 끝 점 둘레) · 공구 아래 rpy (180, 0, rz) · 무작위 닿는 자세 10 개.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

gui = Path(sys.argv[1]).resolve()
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 47875
OUT = Path(sys.argv[3]) if len(sys.argv) > 3 else None
assert PORT != 47861
sys.path.insert(0, str(gui))
import rt_server as rs  # noqa: E402
import cr3a_model as cm  # noqa: E402


def log(*x):
    print(time.strftime('[%H:%M:%S]'), *x, flush=True)


def ang(Ra, Rb):
    """두 회전 사이 각 (rad) — 시험 쪽에서 따로 (scipy)."""
    from scipy.spatial.transform import Rotation
    return float(np.linalg.norm(Rotation.from_matrix(np.asarray(Ra).T @ np.asarray(Rb)).as_rotvec()))


def rot(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


args = argparse.Namespace(robot='ideal', speed=1.0, realtime='off', port=PORT, q0='A', scenario='moving3t', out=None,
                          shaper=('Unshaped', [(True, 8.0), (False, 15.0), (False, 23.0)]), camera='virtual', mask_q_file=None,
                          test_comm_left_s=None, test_rec_max_s=None)
link = rs.Link(PORT)
ctl = rs.make_controller(link, log, args)
q0 = np.asarray(ctl.robot.measured(), float)
T0 = ctl._tip(q0)
log('지금 공구 끝 (mm)', np.round(T0[:3, 3] * 1000, 2).tolist(), 'IK_TOL', rs.IK_TOL_M * 1000, 'mm ·', math.degrees(rs.IK_TOL_RAD), '°')
cfg = ctl.W.planner.ik_solver.config
cfg0 = {'position_tolerance': float(cfg.position_tolerance), 'orientation_tolerance': float(cfg.orientation_tolerance)}
cfg_is_tol = (math.isclose(cfg0['position_tolerance'], rs.IK_TOL_M, rel_tol=1e-6)
              and math.isclose(cfg0['orientation_tolerance'], rs.IK_TOL_RAD, rel_tol=1e-6))
log('IK 설정 (make_controller 뒤 그대로 — 새 서버는 만들 때 넣은 값, 옛 서버는 만든 뒤 바꾼 값)', cfg0, '= IK_TOL' if cfg_is_tol else '≠ IK_TOL')

targets = [('지금 그대로', T0.copy())]
for ax in range(3):
    for s in (1, -1):
        T = T0.copy()
        T[ax, 3] += s * 0.03
        targets.append((f'{"xyz"[ax]} {s * 30:+d} mm', T))
for ax in range(3):
    for s in (1, -1):
        T = T0.copy()
        T[:3, :3] = rot(np.eye(3)[ax], math.radians(10 * s)) @ T0[:3, :3]
        targets.append((f'R{"xyz"[ax]} {s * 10:+d}°', T))
for rz in (-90, -45, 0, 45, 90, 180):
    T = T0.copy()
    T[:3, :3] = cm.rpy_R(math.pi, 0.0, math.radians(rz))
    targets.append((f'공구 아래 rpy (180, 0, {rz})', T))
rng = np.random.default_rng(1027)
lo, hi = ctl.mpc.qlim
for i in range(10):
    q = q0 + rng.uniform(-0.6, 0.6, 6)
    q = np.clip(q, lo + 0.1, hi - 0.1)
    targets.append((f'무작위 {i}', ctl._tip(q)))

import torch  # noqa: E402
from curobo.types import GoalToolPose, JointState, Pose  # noqa: E402

def direct(T):
    """cuRobo IKSolver 를 직접 (_ik 와 같은 목표·씨앗 수) → (성공 [n], 위치 오차 m [n], 방향 오차 [n])."""
    T6 = T @ np.linalg.inv(cm.T_of([0.0, 0.0, cm.TIP_M], [0.0, 0.0, 0.0]))
    pose = Pose(position=torch.tensor(T6[None, :3, 3], device='cuda', dtype=torch.float32),
                quaternion=torch.tensor(rs.R_quat(T6[:3, :3])[None], device='cuda', dtype=torch.float32))
    cur = JointState.from_position(torch.tensor([list(q0)], device='cuda', dtype=torch.float32), joint_names=[f'joint{k}' for k in range(1, 7)])
    r = ctl.W.planner.ik_solver.solve_pose(GoalToolPose.from_poses({'Link6': pose}), current_state=cur, return_seeds=rs.IK_SEEDS)
    return (r.success.reshape(-1).cpu().numpy().astype(bool), r.position_error.reshape(-1).cpu().numpy().astype(float),
            r.rotation_error.reshape(-1).cpu().numpy().astype(float))


rows = []
for name, T in targets:
    t0 = time.monotonic()
    q, why = ctl._ik(T, q0)
    ms = (time.monotonic() - t0) * 1000
    row = {'target': name, 'ok': q is not None, 'why': why, 'ms': round(ms, 1)}
    if q is not None:
        Tq = ctl._tip(q)
        row['pos_mm'] = round(float(np.linalg.norm(Tq[:3, 3] - T[:3, 3])) * 1000, 4)
        row['rot_deg'] = round(math.degrees(ang(T[:3, :3], Tq[:3, :3])), 4)
        row['within_tol'] = row['pos_mm'] <= rs.IK_TOL_M * 1000 and row['rot_deg'] <= math.degrees(rs.IK_TOL_RAD)
    # cuRobo 를 직접 (같은 목표 · 같은 씨앗 수): '성공' 해들의 cuRobo 위치·방향 오차 최대
    ok, pe, re = direct(T)
    row['curobo_n_success'] = int(ok.sum())
    if ok.any():
        row['curobo_success_pos_max_mm'] = round(float(pe[ok].max()) * 1000, 4)
        row['curobo_success_rot_max'] = round(float(re[ok].max()), 6)
    rows.append(row)
    log(json.dumps(row, ensure_ascii=False))

# 설정의 허용 오차가 풀 때 쓰이나: 위치 허용 오차를 float32 로 못 닿는 1e-9 m 로 바꿔 '지금 그대로' 를 풀어 봄 — 그래도 성공이면 안
#  쓰임, 성공이 거의 0 이면 쓰임. 그다음 크게 (1 m) 바꿔 성공이 느는지도. 끝나면 IK_TOL 로 되돌림 (설정 값이 쓰이고 그 값이 처음부터
#  IK_TOL 이면 = 만들 때 넣은 IK_TOL 이 쓰임)
probe = {}
for tol in (1e-9, 1.0):
    cfg.position_tolerance = tol
    ok, pe, re = direct(targets[0][1])
    probe[f'tol_{tol:g}'] = {'n_success': int(ok.sum()), 'pos_err_min_mm': round(float(pe.min()) * 1000, 6)}
cfg.position_tolerance = rs.IK_TOL_M
log('허용 오차 바꾸기 시험', json.dumps(probe, ensure_ascii=False))
okr = [r for r in rows if r['ok']]
summary = {'n': len(rows), 'solved': len(okr), 'within_tol': sum(r.get('within_tol', False) for r in okr),
           'pos_mm_max': max((r['pos_mm'] for r in okr), default=None), 'rot_deg_max': max((r['rot_deg'] for r in okr), default=None),
           'curobo_success_pos_max_mm': max((r.get('curobo_success_pos_max_mm', 0) for r in rows), default=None),
           'curobo_success_rot_max': max((r.get('curobo_success_rot_max', 0) for r in rows), default=None),
           'tolerance_probe': probe, 'ik_cfg_at_start': cfg0, 'ik_cfg_is_tol': cfg_is_tol,
           'ik_tol_mm': rs.IK_TOL_M * 1000, 'ik_tol_deg': math.degrees(rs.IK_TOL_RAD), 'folder': str(gui)}
log('요약', json.dumps(summary, ensure_ascii=False))
if OUT:
    OUT.write_text(json.dumps({'summary': summary, 'rows': rows}, ensure_ascii=False, indent=1))
