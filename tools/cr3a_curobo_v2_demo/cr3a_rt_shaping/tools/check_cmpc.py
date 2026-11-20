#!/usr/bin/env python3
"""[2026-10-14] 성형 MPC (rt_cmpc, C 방식) 단독 점검 — 물체 없이 A → B 한 다리 (로봇 = 명령 그대로, ROS·WMX 없음).
  PYTHONPATH=<curobo_v2_shaping>:$PYTHONPATH python3 -m tools.check_cmpc [--shaper ZVD --shaper-hz 8] [--horizon 1.0] [--rate 20]
  (cr3a_rt_shaping 에서. [2026-10-27 R-1] tools/ 로 옮김 — C 방식은 다음 단계라 도구로 둠, 사용자 결정)
확인:
  ① MPC 궤적 첫 점 = 시작 상태 (시간표가 맞는가)
  ② cuRobo 안의 y (set_input_shaper 행렬) = 실행 기록을 1 ms 커널로 컨볼루션한 y (다른 계산 경로) — 상태 궤적 점에서
  ③ cuRobo 가 충돌·기구학에 쓴 구 위치 = y 의 기구학 (u 가 아님)
  ④ 움직임: 도착 시간, 속도·가속 한계 대비, 전역 경로에서 벗어난 정도, 풀기 시간
"""
import argparse
import dataclasses
from pathlib import Path

import numpy as np

from rt_cmpc import CMpc, CMpcCfg, _js
from rt_config import Config
from rt_shaping import load_cfg, make_kernel, shape_fir
from rt_world import World, robot_cfg

ROOT = Path(__file__).resolve().parent.parent     # cr3a_rt_shaping (기본 성형 설정 shaping.yaml 이 있는 곳)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--shaping-file', default=str(ROOT / 'shaping.yaml'))
    ap.add_argument('--shaper', nargs='*')
    ap.add_argument('--shaper-hz', nargs='*', type=float)
    ap.add_argument('--shaper-zeta', nargs='*', type=float)
    ap.add_argument('--horizon', type=float, default=CMpcCfg.horizon_s)
    ap.add_argument('--rate', type=float, default=CMpcCfg.rate_hz)
    ap.add_argument('--knot', type=float, default=CMpcCfg.knot_s)       # > 0 = 첫 구간 자유화 (매듭 간격 s, rt_config.CMpcCfg.knot_s)
    ap.add_argument('--cycles', type=int, default=120)
    ap.add_argument('--mpc', nargs='*', default=[], metavar='이름=값')     # CMpcCfg 바꾸기 (예: lookahead_s=0.4 nonterminal=1)
    a = ap.parse_args()
    sc = load_cfg(a.shaping_file, method=a.shaper, freq_hz=a.shaper_hz, zeta=a.shaper_zeta)
    K = make_kernel(sc)
    print(f'성형: {sc.label()}  칸 {K.L}, 길이 {K.duration_s * 1000:.1f} ms, 평균 지연 {K.mean_delay_s * 1000:.1f} ms')
    c = Config()
    rd = robot_cfg(c.model)
    W = World(rd, c, log=print)
    grid = W.cm.esdf_mpc()
    kv = {k: (v if k == 'ls_scales' else (v.lower() in ('1', 'true') if k == 'force_shaper' else float(v))) for k, v in (x.split('=') for x in a.mpc)}
    cc = dataclasses.replace(CMpcCfg(), horizon_s=a.horizon, rate_hz=a.rate, knot_s=a.knot, **kv)
    print('MPC 설정:', cc)
    mpc = CMpc(rd, cc, c.dist.act_new, grid, [W.floor], K)
    print(f'MPC: 주기 {1 / cc.rate_hz * 1000:.0f} ms, 점 간격 {mpc.opt_dt * 1000:.3f} ms, 매듭 {mpc.n_knots}, 상태 궤적 점 {mpc.H}, '
          f'기억 점 {mpc.Hh}')
    qa, qb = (np.array(g) for g in c.task.goals(c.model))
    path, wall = W.plan(qa, qb)
    assert path is not None, '전역 계획 실패'
    print(f'전역 경로: {path[0][-1]:.2f} s, 계획 {wall:.2f} s')
    mpc.start(qa)
    mpc.follow(*path)
    g = 0.001
    n_dt = int(round(1.0 / cc.rate_hz / g))
    log = [np.repeat(qa[None], K.L + 64 * n_dt, 0)]            # 실행된 u (1 ms), 시작 전 = 출발 자세
    log_v, log_a = [], []
    s0 = (qa.copy(), np.zeros(6), np.zeros(6))
    ms, st_err, y_err, sph_err, acc_r, vel_r = [], [], [], [], [], []
    t_arrive = None
    for cyc in range(a.cycles):
        past = np.concatenate(log)[-(mpc.Hh * int(np.ceil(mpc.opt_dt / g)) + 8):]
        hist = mpc.history(past, g)
        info = mpc.step(s0, hist)
        assert info['ok'], f'{cyc}: MPC 해가 수가 아님'
        u1 = info['u1']
        ms.append(info['ms'])
        st_err.append(info['start_err'])
        acc_r.append(info['acc_ratio'])
        vel_r.append(info['vel_ratio'])
        if mpc.shaped:
            # ② cuRobo 안 y (행렬) vs 실행 기록 + 이번 u 를 1 ms 커널로 컨볼루션 → 상태 궤적 점 (0 ~ 2 주기) 시각에서
            P = info['traj']
            nf = info.get('n_f', 0)                                 # 첫 구간 자유화: 계획 앞 nf 점은 지난 시각
            y_in = mpc.shaped_traj(P, hist)[nf:nf + 2 * mpc.M + 1]
            seq = np.concatenate([np.concatenate(log), u1[:, 0]])
            y1 = shape_fir(seq, K.h)[-len(u1):]                     # 지금 (u1[0]) 부터 1 ms 칸
            tj = np.arange(2 * mpc.M + 1) * mpc.opt_dt
            y_out = np.stack([np.interp(tj, np.arange(len(u1)) * g, y1[:, j]) for j in range(6)], 1)
            y_err.append(float(np.abs(y_in - y_out).max()))
            # ③ cuRobo 가 쓴 구 = y 의 기구학인가 (평가 rollout 의 상태)
            rs = mpc.mpc.trajectory_execution_manager.get_robot_state_sequence()
            sph = rs.robot_spheres[0, nf:nf + 2 * mpc.M + 1, :, :3].detach().cpu().numpy()
            ks = mpc.mpc.compute_kinematics(_js(y_in[0], mpc.names, mpc.device))
            k_u = mpc.mpc.compute_kinematics(_js(P[nf], mpc.names, mpc.device))
            s_y = ks.robot_spheres.view(-1, 4)[:, :3].cpu().numpy()
            s_u = k_u.robot_spheres.view(-1, 4)[:, :3].cpu().numpy()
            sph_err.append((float(np.abs(sph[0] - s_y).max()), float(np.abs(sph[0] - s_u).max())))
        log.append(u1[0:n_dt, 0])                           # 이번 주기 [지금, 지금 + 주기) 가 실행됨
        log_v.append(u1[0:n_dt, 1])
        log_a.append(u1[0:n_dt, 2])
        s0 = tuple(np.array(x, float) for x in u1[n_dt])
        if cyc % max(1, int(round(cc.rate_hz / 4))) == 0:
            lg = np.concatenate(log)[-K.L:]
            y_now = (K.h[::-1][:, None] * lg[-K.L:]).sum(0) if len(lg) >= K.L else lg[-1]
            print(f'  {cyc:3d} t {(cyc + 1) / cc.rate_hz:5.2f}  기준 {mpc.ref_t:5.2f}  y→B {np.degrees(np.max(np.abs(y_now - qb))):6.2f}°  '
                  f'목표까지 {np.degrees(np.max(np.abs(s0[0] - mpc.goal))):6.2f}°  '
                  f'B 까지 {np.degrees(np.max(np.abs(s0[0] - qb))):6.2f}°  |v| {np.max(np.abs(s0[1])):.3f}  MPC 끝점−목표 '
                  f'{np.degrees(np.max(np.abs(info["traj"][-1] - mpc.goal))):6.2f}°  {info["ms"]:.0f} ms')
        if t_arrive is None and np.max(np.abs(s0[0] - qb)) < np.radians(0.1) and np.max(np.abs(s0[1])) < 1e-2:
            t_arrive = (cyc + 1) / cc.rate_hz
    U = np.concatenate(log)
    V, A = np.concatenate(log_v), np.concatenate(log_a)
    tp, qp = path
    print(f'풀기 ms 중앙 {np.median(ms):.1f} (최대 {max(ms):.1f}, 첫 cold {ms[0]:.1f})')
    print(f'① 첫 점 − 시작 상태 최대 {max(st_err):.2e} rad')
    if mpc.shaped:
        print(f'② cuRobo 안 y − 1 ms 컨볼루션 y 최대 {max(y_err):.2e} rad (중앙 {np.median(y_err):.2e})')
        print(f'③ 구 위치: cuRobo − FK(y) 최대 {max(e[0] for e in sph_err) * 1000:.3f} mm,  cuRobo − FK(u) 최대 '
              f'{max(e[1] for e in sph_err) * 1000:.3f} mm (움직일 때 y ≠ u 라 커야 함)')
    print(f'④ 도착 {t_arrive} s (경로 {tp[-1]:.2f} s), MPC 속도/한계 최대 {max(vel_r):.2f}, 가속/한계 최대 {max(acc_r):.2f}')
    print(f'   실행된 u: 속도/한계 {np.max(np.abs(V) / mpc.vmax):.2f}, 가속/한계 {np.max(np.abs(A) / mpc.amax):.2f}, '
          f'끝 − 목표 {np.degrees(np.max(np.abs(U[-1] - qb))):.3f}°')
    Y = shape_fir(U, K.h)
    print(f'   성형 전 u 와 성형된 y 의 최대 차이 {np.degrees(np.max(np.abs(Y - U))):.2f}° (평균 지연 × 속도 정도)')
    travel = float(np.degrees(np.abs(np.diff(U, axis=0)).sum()))
    travel_y = float(np.degrees(np.abs(np.diff(Y, axis=0)).sum()))
    print(f'   관절 이동 합: u {travel:.0f}°, y {travel_y:.0f}° (직선 {np.degrees(np.abs(qb - qa).sum()):.0f}°), y 끝 − 목표 '
          f'{np.degrees(np.max(np.abs(Y[-1] - qb))):.3f}°')
    over = float(np.degrees(np.max(np.maximum((U - qb) * np.sign(qb - qa), 0.0))))
    import json
    print('RESULT', json.dumps({'mpc': {k: v for k, v in dataclasses.asdict(cc).items()}, 'shaper': sc.label(), 'arrive_s': t_arrive,
                                'path_s': round(float(tp[-1]), 3), 'travel_deg': round(travel, 1), 'travel_y_deg': round(travel_y, 1), 'overshoot_deg': round(over, 3),
                                'vel_ratio': round(float(np.max(np.abs(V) / mpc.vmax)), 3), 'acc_ratio': round(float(np.max(np.abs(A) / mpc.amax)), 3),
                                'mpc_acc_ratio': max(acc_r), 'solve_ms_med': float(np.median(ms)), 'solve_ms_max': float(max(ms)),
                                'y_err': max(y_err) if y_err else None, 'end_err_deg': round(float(np.degrees(np.max(np.abs(U[-1] - qb)))), 3)}))


if __name__ == '__main__':
    main()
