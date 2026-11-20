#!/usr/bin/env python3
# [만듦 2026-10-13] CR3A A ↔ B 왕복 + 장애물을 동작마다 랜덤으로 → 매번 cuRoboV2 1회성 계획 → R2 로 실행 (스트리밍 아님)
#   python3 plan_random.py --out <폴더> [--n 20] [--seed 20261013] [--kind mixed|pillar] [--execute]
#   흐름: (장애물 없음) 현재 자세 → A, 그다음 k = 1..n: 장애물을 새로 놓음 → 목표(B, A 번갈아)로 plan_cspace → R2 실행 → 도착 확인
#   --kind pillar : 첫 판 (15:33). 바닥에 선 10×10×30 cm 기둥 하나, 위치·회전만 랜덤
#   --kind mixed  : [추가 2026-10-13 사용자 "전부 바닥에 서 있는 길쭉한 상자네. 공중에 띄우고 … 다양한 위치 및 크기"]
#                   동작마다 1~3개, 종류 4가지 (크기·자세 랜덤):
#                     기둥 (바닥에 섬)        가로·세로 5~15 cm, 높이 15~45 cm, 회전 z
#                     벽 (얇은 세로판)        두께 2~5 cm, 폭 15~35 cm, 높이 10~35 cm, 바닥에 서거나 떠 있음
#                     판 (공중 가로판)        15~35 × 15~35 cm, 두께 2~5 cm, 높이 0.25~0.60 m, 기울기 ±15°
#                     떠 있는 상자            각 변 6~20 cm, 높이 0.10~0.60 m, 아무 방향
#                   첫 번째 장애물은 A↔B 를 관절 직선 보간으로 가면 부딪히는 위치만 (매번 실제로 피해야 하는 문제)
#   공통 조건 (거절 표본): A, B 자세에서 END_CLEARANCE_MIN(3 cm) 이상, 장애물 중심이 로봇 받침에서 0.20 m 밖,
#                         떠 있는 것은 바닥에서 2 cm 이상 위
#   장애물 크기가 바뀌므로 계획기에는 장면을 통째로 다시 넣는다 (update_world). 바닥은 늘 포함
#   계획 실패도 기록하고 (로봇은 그 자리), 다음 시도에서 새 장애물로 같은 목표를 다시
#   결과: plan.json — 'moves' = 실행된 동작 (각각 'obstacles'), 'trials' = 모든 시도. exec.json — R2 결과와 /joint_states
import argparse
import json
import math
import random
import sys
from pathlib import Path

import cr3a_scene as S
import plan_avoid as P

REGION = {'x': (0.20, 0.60), 'y': (-0.36, 0.36)}
KINDS = ['기둥', '벽', '판', '떠 있는 상자']


def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def q_axis(axis, ang):
    s = math.sin(ang / 2)
    return [math.cos(ang / 2)] + [s * a for a in axis]


def bottom_z(dims, pose):
    """회전한 상자의 가장 낮은 점 높이"""
    R = S.quat_to_rot(pose[3:7])
    return pose[2] - sum(abs(R[2][j]) * dims[j] / 2 for j in range(3))


def make_obstacle(rng, kind):
    U = rng.uniform
    x, y = U(*REGION['x']), U(*REGION['y'])
    yaw = q_axis([0, 0, 1], U(0, math.pi))
    if kind == '기둥':
        d = [U(0.05, 0.15), U(0.05, 0.15), U(0.15, 0.45)]
        q = yaw; z = d[2] / 2
    elif kind == '벽':
        d = [U(0.02, 0.05), U(0.15, 0.35), U(0.10, 0.35)]
        q = yaw
        z = d[2] / 2 if rng.random() < 0.5 else U(0.05, 0.35) + d[2] / 2
    elif kind == '판':
        d = [U(0.15, 0.35), U(0.15, 0.35), U(0.02, 0.05)]
        tilt = quat_mul(q_axis([1, 0, 0], math.radians(U(-15, 15))), q_axis([0, 1, 0], math.radians(U(-15, 15))))
        q = quat_mul(yaw, tilt); z = U(0.25, 0.60)
    else:   # 떠 있는 상자: 아무 방향 (균일 랜덤 쿼터니언)
        d = [U(0.06, 0.20) for _ in range(3)]
        u1, u2, u3 = rng.random(), rng.random(), rng.random()
        q = [math.sqrt(u1) * math.cos(2 * math.pi * u3), math.sqrt(1 - u1) * math.sin(2 * math.pi * u2),
             math.sqrt(1 - u1) * math.cos(2 * math.pi * u2), math.sqrt(u1) * math.sin(2 * math.pi * u3)]
        z = U(0.10, 0.60)
    return {'kind': kind, 'dims': d, 'pose': [x, y, z] + q}


def ok_basic(ob, kin, qA, qB):
    if math.hypot(ob['pose'][0], ob['pose'][1]) < 0.20:
        return False
    if ob['kind'] in ('판', '떠 있는 상자') or (ob['kind'] == '벽' and ob['pose'][2] - ob['dims'][2] / 2 > 1e-6):
        if bottom_z(ob['dims'], ob['pose']) < 0.02:
            return False
    return min(S.clearances(kin, [qA, qB], ob)['box']) >= S.END_CLEARANCE_MIN


def sample_obstacles(rng, kin, qA, qB, line, kind_mode):
    if kind_mode == 'pillar':
        for _ in range(500):
            yaw = rng.uniform(0, math.pi / 2)
            ob = {'kind': '기둥', 'dims': list(S.BOX['dims']),
                  'pose': [rng.uniform(0.30, 0.55), rng.uniform(-0.22, 0.22), S.BOX['dims'][2] / 2] + q_axis([0, 0, 1], yaw)}
            if ok_basic(ob, kin, qA, qB):
                return [ob]
        raise RuntimeError('기둥 위치를 못 찾음')
    obs = []
    # 첫 번째: 직선 보간 경로를 막는 것 (종류를 바꿔 가며)
    for attempt in range(3000):
        kind = KINDS[attempt % 4] if attempt > 400 else rng.choice(KINDS)
        ob = make_obstacle(rng, kind)
        if ok_basic(ob, kin, qA, qB) and min(S.clearances(kin, line, ob)['box']) < 0:
            obs.append(ob); break
    else:
        raise RuntimeError('경로를 막는 장애물을 못 찾음')
    # 나머지 0~2개: 아무 데나 (조건만 맞으면), 서로 겹쳐도 됨
    for _ in range(rng.randint(0, 2)):
        for _ in range(500):
            ob = make_obstacle(rng, rng.choice(KINDS))
            if ok_basic(ob, kin, qA, qB):
                obs.append(ob); break
    return obs


def set_world(planner, obstacles):
    from curobo.scene import Cuboid, Scene
    cubes = [Cuboid(name='floor', pose=list(S.FLOOR['pose']), dims=list(S.FLOOR['dims']))]
    cubes += [Cuboid(name=f'obs{i}', pose=list(o['pose']), dims=list(o['dims'])) for i, o in enumerate(obstacles)]
    planner.update_world(Scene(cuboid=cubes))


def fmt_obs(obs):
    return ', '.join(f'{o["kind"]}({o["pose"][0]:.2f},{o["pose"][1]:+.2f},{o["pose"][2]:.2f})' for o in obs) or '없음'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--n', type=int, default=20)
    ap.add_argument('--seed', type=int, default=20261013)
    ap.add_argument('--kind', choices=['mixed', 'pillar'], default='mixed')
    ap.add_argument('--execute', action='store_true')
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)

    kin = S.make_kinematics()
    planner = P.build_planner()
    _, home_quat = S.tool_pose(kin, S.HOME)
    okA, qA, *_ = P.ik(planner, S.A_POS, home_quat, S.HOME)
    okB, qB, *_ = P.ik(planner, S.B_POS, home_quat, qA)
    if not (okA and okB):
        P.log('A/B IK 실패 → 중단'); return 1
    line = [[a + (b - a) * i / 100 for a, b in zip(qA, qB)] for i in range(101)]
    P.log(f'A 관절(도) {S.deg(qA)}  B 관절(도) {S.deg(qB)}  시드 {args.seed}, 동작 {args.n}번, 장애물 {args.kind}')

    r2 = None
    if args.execute:
        r2 = P.R2()
        q_now = r2.current()
    else:
        q_now = [0.0] * 6
    P.log('시작 자세(도):', S.deg(q_now))

    set_world(planner, [])
    P.plan(planner, qA, qB)          # plan_cspace 첫 호출은 CUDA 그래프를 만드느라 느려서 미리 한 번

    moves, trials, events = [], [], []

    def run(name, target, obs, k):
        nonlocal q_now
        set_world(planner, obs)
        tr, tplan = P.plan(planner, q_now, target)
        item = {'k': k, 'name': name, 'obstacles': obs, 'plan_ok': tr is not None, 'plan_time': tplan}
        if obs:
            item['straight_min_box_mm'] = min(S.clearances(kin, line, obs)['box']) * 1000
        if tr is None:
            P.log(f'#{k:02d} {name}: 계획 실패 ({tplan:.2f}초)  장애물 {fmt_obs(obs)}')
            trials.append(item)
            return
        c = S.clearances(kin, tr['position'], obs) if obs else None
        item.update({'duration': tr['duration'], 'points': tr['n'],
                     'plan_min_box_mm': min(c['box']) * 1000 if c else None})
        msg = (f'#{k:02d} {name}: 계획 {tplan:.2f}초, 동작 {tr["duration"]:.2f}초'
               + (f', 장애물 최소 {item["plan_min_box_mm"]:.1f} mm' if c else '') + f'  장애물 {fmt_obs(obs)}')
        if r2 is not None:
            q_cur = r2.current()
            gap = max(abs(a - b) for a, b in zip(q_cur, tr['position'][0]))
            if gap > 0.01:
                P.log(msg + f' → 현재 자세와 {math.degrees(gap):.2f}도 차이, 실행 안 함'); trials.append(item); return
            ev = r2.execute(tr)
            ev['name'] = f'#{k:02d} {name}'
            events.append(ev)
            item.update({'accepted': ev.get('accepted'), 'result_status': ev.get('result_status'),
                         'settled_s': (ev['settled'] - ev['sent']) if ev.get('settled') else None,
                         'final_err_mrad': (ev.get('final_err') or 0) * 1000})
            res = '성공' if ev.get('result_status') == 4 else f'status {ev.get("result_status")}'
            arr = f'도착 {item["settled_s"]:.2f}초' if item['settled_s'] else '도착 확인 못함'
            msg += f' → R2 {res}, {arr}'
            q_now = ev['final_q'] if ev.get('final_q') else tr['position'][-1]
            r2.spin(0.3)
        else:
            q_now = tr['position'][-1]
        P.log(msg)
        moves.append({'name': f'#{k:02d} {name}', 'obstacles': obs, **tr})
        trials.append(item)

    run('시작→A', qA, [], 0)
    target_is_B = True
    for k in range(1, args.n + 1):
        obs = sample_obstacles(rng, kin, qA, qB, line, args.kind)
        name = 'A→B' if target_is_B else 'B→A'
        before = len(moves)
        run(name, qB if target_is_B else qA, obs, k)
        if len(moves) > before:          # 실행됐을 때만 방향을 바꿈 (실패하면 같은 목표를 새 장애물로 다시)
            target_is_B = not target_is_B

    ok = sum(1 for t in trials[1:] if t['plan_ok'])
    P.log(f'계획 성공 {ok}/{len(trials) - 1}, 실행 {len(moves) - 1}번')
    data = {'mode': 'random', 'kind': args.kind, 'seed': args.seed, 'n': args.n, 'region': REGION, 'joints': S.JOINTS,
            'scene': {'cuboid': {'floor': S.FLOOR, 'box': S.BOX}}, 'A_pos': S.A_POS, 'B_pos': S.B_POS,
            'tool_quat': home_quat, 'qA': qA, 'qB': qB, 'moves': moves, 'trials': trials,
            'settings': {'act': S.ACT, 'pos_tol': S.POS_TOL, 'ori_tol': S.ORI_TOL, 'ik_seeds': S.IK_SEEDS,
                         'trajopt_seeds': S.TRAJOPT_SEEDS, 'end_clearance_min': S.END_CLEARANCE_MIN}}
    (out / 'plan.json').write_text(json.dumps(data, ensure_ascii=False))
    if r2 is not None:
        (out / 'exec.json').write_text(json.dumps({'events': events, 'joint_states': r2.samples}))
    P.log('저장:', out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
