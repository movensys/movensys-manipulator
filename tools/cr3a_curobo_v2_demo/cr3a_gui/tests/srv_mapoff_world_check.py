"""[2026-11-02 디버깅 D-7] GuiWorld 카메라 지도 끄기 (map_off) 확인 — GPU (Thor 컨테이너), 서버·ROS 없음.
참 형상 지도 (cr3a_mapper.set_oracle) 에 상자 하나를 목표 자세 B 의 로봇 구 자리에 넣고 (A 에서는 멂):
  ① 지도 켬: B 거리 < 0, A → B 계획 실패
  ② 지도 끔: B 거리 = 바닥 면만 (> 0, 'dyn' = 멂), A → B 계획 성공 · 계획 검증 통과 · 'start' 장면 (공구 끝 IK·다시 계획) 으로도 성공
  ③ 지도 끔에서도 책상 (바닥) 은 봄: 바닥 아래로 내려간 자세의 거리 < 0, 그 자세로 계획 실패
  ④ 다시 켬: ① 과 같음 (쌓인 지도는 그대로)
  python3 tests/srv_mapoff_world_check.py      (cr3a_gui 폴더에서 — 옆의 cr3a_rt_shaping 을 씀)
"""
import sys
from pathlib import Path

import numpy as np

G = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(G), str(G.parent / 'cr3a_rt_shaping')]
import gui_controller as gc  # noqa: E402  (rt_run → cuRobo 실행 설정이 먼저)
from rt_config import Config, TaskCfg  # noqa: E402
from rt_world import FAR_M, robot_cfg  # noqa: E402

BOX = 0.10                  # 상자 한 변 (m)
RES = []


def check(name, ok, detail=''):
    RES.append((name, bool(ok)))
    print(('통과' if ok else '실패'), name, detail, flush=True)


def clr(W, q, grid='raw'):
    return float(W.clearance(np.asarray(q, float)[None], grid=grid)[0])


def plan_ok(W, q0, q1):
    path, wall = W.plan(q0, q1)
    return path is not None, path, wall


c = Config(task=TaskCfg(), style='carrot', model='tool')
W = gc.GuiWorld(robot_cfg(c.model), c, log=print)
qA, qB = (np.asarray(q, float) for q in c.task.goals(c.model))

# 상자 자리: B 의 움직이는 구 가운데 A 의 구들과 가장 먼 것 (A 출발은 안 닿게)
cB, rB = (x[0].cpu().numpy() for x in W.spheres(qB[None]))
cA, rA = (x[0].cpu().numpy() for x in W.spheres(qA[None]))
mov = W.moving_sph.cpu().numpy() & (rB > 0)
useA = rA > 0
gap = np.array([np.min(np.linalg.norm(cA[useA] - p, axis=1) - rA[useA]) for p in cB])
gap[~mov] = -np.inf
k = int(np.argmax(gap))
center = cB[k].tolist()
print(f'상자: B 의 구 {k} 자리 {np.round(center, 3).tolist()} m, 한 변 {BOX} m — A 의 구 겉면에서 {gap[k] * 1000:.0f} mm')
W.cm.set_oracle([], [(center, (BOX, BOX, BOX), [1, 0, 0, 0])])
W.cm.esdf_mpc()

# ① 지도 켬
dA, dB = clr(W, qA), clr(W, qB)
check('① 켬: A 는 안 닿음', dA > 0.0, f'{dA * 1000:.0f} mm')
check('① 켬: B 는 상자 안 (거리 < 0)', dB < 0.0, f'{dB * 1000:.0f} mm')
ok, _, wall = plan_ok(W, qA, qB)
check('① 켬: A → B 계획 실패 (상자)', not ok, f'{wall:.2f} s')

# ② 지도 끔
W.map_off = True
d = W.clearances(np.stack([qA, qB]), ('raw', 'dyn', 'mpc'))
c_all, r_all = W.spheres(np.stack([qA, qB]))
zc, rr = c_all[..., 2].cpu().numpy(), r_all.cpu().numpy()
fl = np.where(W.moving_sph.cpu().numpy() & (rr > 0), zc - rr - W.floor_z, FAR_M).min(1)      # 바닥 면만 (손 계산)
check('② 끔: raw·mpc = 바닥 면 거리만', np.allclose(d['raw'], fl, atol=1e-5) and np.allclose(d['mpc'], fl, atol=1e-5),
      f'raw {np.round(d["raw"] * 1000, 1).tolist()} mm, 바닥 {np.round(fl * 1000, 1).tolist()} mm')
check('② 끔: dyn (새 물체) = 멂', np.all(d['dyn'] >= FAR_M - 1e-6))
check('② 끔: B 거리 > 0', d['raw'][1] > 0.0, f'{d["raw"][1] * 1000:.0f} mm')
ok, path, wall = plan_ok(W, qA, qB)
check('② 끔: A → B 계획 성공 (상자 안 봄)', ok, f'{wall:.2f} s' + (f', 끝 차 {np.degrees(np.abs(path[1][-1] - qB).max()):.3f}°' if ok else ''))
if ok:
    W.map_off = False
    through = clr(W, path[1][-1])
    W.map_off = True
    check('② 그 경로 끝은 켠 지도로는 상자 안', through < 0.0, f'{through * 1000:.0f} mm')
W.plan_prep('start', q0=qA)
check("② 끔: 'start' 장면 (IK·다시 계획) 으로도 계획 성공", W.plan_solve(qA, qB) is not None)

# ③ 끔에서도 바닥: J2 를 숙여 손목이 바닥 아래로 가는 자세 (B 에서 가까운 J2 부터, 양쪽)
qF = None
j2s = np.radians(np.arange(-150.0, 151.0, 5.0))
for j2 in j2s[np.argsort(np.abs(j2s - qB[1]))]:
    q = qB.copy()
    q[1] = j2
    if clr(W, q) < -0.02:
        qF = q
        break
check('③ 끔: 바닥 아래 자세를 찾음', qF is not None, '' if qF is None else f'J2 {np.degrees(qF[1]):.0f}°, 거리 {clr(W, qF) * 1000:.0f} mm')
if qF is not None:
    ok, _, wall = plan_ok(W, qA, qF)
    check('③ 끔: 바닥 아래로는 계획 실패 (책상은 그대로 봄)', not ok, f'{wall:.2f} s')

# ④ 다시 켬
W.map_off = False
dB2 = clr(W, qB)
check('④ 다시 켬: B 거리 < 0 (지도 그대로)', dB2 < 0.0 and abs(dB2 - dB) < 1e-6, f'{dB2 * 1000:.0f} mm')
ok, _, wall = plan_ok(W, qA, qB)
check('④ 다시 켬: A → B 계획 실패', not ok, f'{wall:.2f} s')
W.plan_prep('start', q0=qA)
check("④ 다시 켬: 'start' 장면으로도 실패", W.plan_solve(qA, qB) is None)

n_bad = sum(1 for _, ok in RES if not ok)
print(f'RESULT 통과 {len(RES) - n_bad}/{len(RES)}', flush=True)
sys.exit(1 if n_bad else 0)
