#!/usr/bin/env python3
# [만듦 2026-10-13] CR3A 1회성 장애물 회피 — cuRoboV2 로 계획하고, (선택) R2 의 궤적 액션으로 실행
#   python3 plan_avoid.py --out <폴더>              계획만 (시작 = 0 자세). 로봇·R2 필요 없음
#   python3 plan_avoid.py --out <폴더> --execute    R2 의 /joint_states 에서 시작 자세를 읽고, 계획을 R2 로 실행
#   동작: ① 시작 자세 → A   ② A → B (상자를 피하는 동작)
#   결과: <폴더>/plan.json  (장면, A/B, 직선 보간 검사, 두 동작의 계획 궤적, 상자/바닥 거리, 설정)
#         <폴더>/exec.json  (--execute: 액션 결과, /joint_states 기록, 보낸 시각)
#   R2 쪽 사실 (wmx_r2 main, joint_trajectory_controller.cpp): 관절 이름을 보지 않고 순서대로 축 0..5 에 넣음, 점 1000개까지,
#   StartCSplinePos(위치·시간만 씀). CR3A 파라미터의 InPosWidth 가 1000 rad 라 "성공" 응답 = 도착이 아님 → /joint_states 로 도착 확인
import argparse
import json
import math
import sys
import time
from pathlib import Path

import torch

import cr3a_scene as S

ACTION = '/movensys_manipulator_arm_controller/follow_joint_trajectory'   # wmx_r2 cr3a_manipulator_config.yaml


def log(*a):
    print(time.strftime('[%H:%M:%S]'), *a, flush=True)


def build_planner():
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    cfg = MotionPlannerCfg.create(
        robot=str(S.ROBOT_YML), scene_model=S.SCENE, collision_cache={'cuboid': 8},
        num_ik_seeds=S.IK_SEEDS, num_trajopt_seeds=S.TRAJOPT_SEEDS,
        position_tolerance=S.POS_TOL, orientation_tolerance=S.ORI_TOL,
        optimizer_collision_activation_distance=S.ACT)
    p = MotionPlanner(cfg)
    t = time.time()
    p.warmup(enable_graph=True)
    log(f'계획기 예열 {time.time() - t:.1f}초 (첫 실행은 GPU 코드 컴파일로 더 걸림)')
    return p


def js(q):
    from curobo.types import JointState
    return JointState.from_position(torch.tensor([q], device='cuda', dtype=torch.float32), joint_names=S.JOINTS)


def to_planner_order(planner, q):
    names = planner.joint_names
    return [q[S.JOINTS.index(n)] for n in names]


def ik(planner, pos, quat, seed_q):
    from curobo.types import GoalToolPose, Pose
    goal = GoalToolPose.from_poses({S.TOOL: Pose.from_list(list(pos) + list(quat))}, num_goalset=1)
    r = planner.ik_solver.solve_pose(goal, current_state=js(seed_q), return_seeds=1)
    ok = bool(r.success.view(-1)[0])
    q = r.js_solution.position.reshape(-1, len(planner.joint_names))[0].tolist()
    q = [q[planner.joint_names.index(n)] for n in S.JOINTS]
    return ok, q, float(r.position_error.view(-1)[0]), float(r.rotation_error.view(-1)[0])


def plan(planner, q0, q1):
    """관절 목표 계획 (plan_cspace: 요청한 관절값을 그대로 지킴). js_solution 을 그대로 씀
    (get_interpolated_plan 은 구간 시간을 올림해서 최대 60% 늘어남 — Mirobot 때 확인)"""
    from curobo.types import JointState
    names = planner.joint_names
    start = JointState.from_position(torch.tensor([to_planner_order(planner, q0)], device='cuda', dtype=torch.float32),
                                     joint_names=names)
    goal = JointState.from_position(torch.tensor([to_planner_order(planner, q1)], device='cuda', dtype=torch.float32),
                                    joint_names=names)
    t = time.time()
    r = planner.plan_cspace(goal, start, max_attempts=10)
    dt_plan = time.time() - t
    if r is None or not bool(r.success.any()):
        return None, dt_plan
    sol = r.js_solution
    pos = sol.position.reshape(-1, len(names))
    vel = sol.velocity.reshape(-1, len(names))
    acc = sol.acceleration.reshape(-1, len(names))
    jn = list(sol.joint_names) if sol.joint_names else names
    idx = [jn.index(n) for n in S.JOINTS]
    dt = float(sol.dt.view(-1)[0])
    return {'dt': dt, 'n': int(pos.shape[0]), 'duration': dt * (pos.shape[0] - 1), 'plan_time': dt_plan,
            'position': pos[:, idx].cpu().tolist(), 'velocity': vel[:, idx].cpu().tolist(),
            'acceleration': acc[:, idx].cpu().tolist()}, dt_plan


def summarize(name, tr, kin):
    c = S.clearances(kin, tr['position'])
    i = min(range(len(c['box'])), key=lambda k: c['box'][k])
    tr['clearance'] = c
    v = [max(abs(row[j]) for row in tr['velocity']) for j in range(6)]
    a = [max(abs(row[j]) for row in tr['acceleration']) for j in range(6)]
    log(f'{name}: {tr["n"]}점, dt {tr["dt"] * 1000:.1f} ms, 동작 {tr["duration"]:.2f}초, 계획 {tr["plan_time"]:.2f}초, '
        f'상자 최소 거리 {c["box"][i] * 1000:.1f} mm ({c["box_link"][i]}, {i}번째 점), 바닥 최소 {min(c["floor"]) * 1000:.1f} mm')
    log(f'   최대 속도 {[round(x, 2) for x in v]}  최대 가속 {[round(x, 2) for x in a]}')


# ---------------- R2 실행 ----------------
class R2:
    def __init__(self):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from sensor_msgs.msg import JointState as JS
        from control_msgs.action import FollowJointTrajectory
        rclpy.init()
        self.rclpy = rclpy
        self.node = Node('cr3a_avoid_demo')
        self.FJT = FollowJointTrajectory
        self.samples = []      # (벽시계, [joint1..6])
        self.node.create_subscription(JS, '/joint_states', self._on_js, 50)
        self.ac = ActionClient(self.node, FollowJointTrajectory, ACTION)

    def _on_js(self, m):
        try:
            q = [m.position[list(m.name).index(n)] for n in S.JOINTS]
        except ValueError:
            return
        self.samples.append((time.time(), q))

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)

    def current(self, timeout=10.0):
        n = len(self.samples)
        end = time.time() + timeout
        while time.time() < end and len(self.samples) <= n:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if len(self.samples) <= n:
            raise RuntimeError('/joint_states 를 못 받음 (R2 joint_state_broadcaster 가 떠 있나?)')
        return self.samples[-1][1]

    def execute(self, tr, settle_tol=1e-3, extra=5.0):
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        if not self.ac.wait_for_server(timeout_sec=20):
            raise RuntimeError(f'액션 서버 없음: {ACTION}')
        g = self.FJT.Goal()
        g.trajectory.joint_names = list(S.JOINTS)
        for i, (p, v, a) in enumerate(zip(tr['position'], tr['velocity'], tr['acceleration'])):
            pt = JointTrajectoryPoint()
            pt.positions, pt.velocities, pt.accelerations = list(p), list(v), list(a)
            t = i * tr['dt']
            pt.time_from_start = Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
            g.trajectory.points.append(pt)
        ev = {'sent': time.time()}
        fut = self.ac.send_goal_async(g)
        while not fut.done():
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
        gh = fut.result()
        ev['accepted'] = bool(gh.accepted)
        if not gh.accepted:
            return ev
        rf = gh.get_result_async()
        while not rf.done():
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
        res = rf.result()
        ev['result_time'] = time.time()
        ev['result_status'] = int(res.status)
        ev['error_code'] = int(res.result.error_code)
        # 도착 확인: /joint_states 가 마지막 점에 settle_tol 안으로 들어올 때까지
        goal = tr['position'][-1]
        end = ev['sent'] + tr['duration'] + extra
        ev['settled'] = None
        while time.time() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.01)
            if self.samples and max(abs(a - b) for a, b in zip(self.samples[-1][1], goal)) < settle_tol:
                ev['settled'] = time.time()
                break
        q = self.samples[-1][1] if self.samples else None
        ev['final_q'] = q
        ev['final_err'] = max(abs(a - b) for a, b in zip(q, goal)) if q else None
        return ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--execute', action='store_true')
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    kin = S.make_kinematics()
    planner = build_planner()
    log('계획기 관절 순서:', planner.joint_names)

    # A, B: 위치는 정해 두고, 방향은 Dobot home 자세의 도구 방향. IK 씨앗은 home (팔꿈치 위 자세 유지)
    _, home_quat = S.tool_pose(kin, S.HOME)
    okA, qA, epA, erA = ik(planner, S.A_POS, home_quat, S.HOME)
    okB, qB, epB, erB = ik(planner, S.B_POS, home_quat, qA)
    log(f'A IK {"OK" if okA else "실패"} 오차 {epA * 1000:.2f} mm / {erA:.4f}, 관절(도) {S.deg(qA)}')
    log(f'B IK {"OK" if okB else "실패"} 오차 {epB * 1000:.2f} mm / {erB:.4f}, 관절(도) {S.deg(qB)}')
    if not (okA and okB):
        log('IK 실패 → 중단'); return 1
    ce = S.clearances(kin, [qA, qB])
    log(f'끝 자세의 상자 거리: A {ce["box"][0] * 1000:.1f} mm ({ce["box_link"][0]}), B {ce["box"][1] * 1000:.1f} mm ({ce["box_link"][1]})')
    if min(ce['box']) < S.END_CLEARANCE_MIN:
        log(f'끝 자세가 상자에 {S.END_CLEARANCE_MIN * 1000:.0f} mm 보다 가까움 → 장면을 고쳐야 함, 중단'); return 1

    # 직선(관절 보간)으로 가면 상자에 부딪히는지
    N = 101
    line = [[a + (b - a) * k / (N - 1) for a, b in zip(qA, qB)] for k in range(N)]
    cl = S.clearances(kin, line)
    hit = [k for k, d in enumerate(cl['box']) if d < 0]
    log(f'직선 보간 A→B: 상자 최소 거리 {min(cl["box"]) * 1000:.1f} mm, 파고드는 점 {len(hit)}/{N}'
        + (f' ({hit[0]}~{hit[-1]}번, {cl["box_link"][hit[len(hit) // 2]]})' if hit else ''))

    r2 = None
    if args.execute:
        r2 = R2()
        q0 = r2.current()
        log('R2 /joint_states 시작 자세(도):', S.deg(q0))
    else:
        q0 = [0.0] * 6

    # 첫 plan_cspace 는 CUDA 그래프를 만드느라 느려서 한 번 미리 돌림 (warmup 은 plan_pose 만 덮음)
    plan(planner, qA, qB)

    t1, _ = plan(planner, q0, qA)
    if t1 is None:
        log('① 시작 → A 계획 실패'); return 1
    summarize('① 시작→A', t1, kin)
    t2, _ = plan(planner, qA, qB)
    if t2 is None:
        log('② A→B 계획 실패'); return 1
    summarize('② A→B (회피)', t2, kin)

    data = {'joints': S.JOINTS, 'scene': S.SCENE, 'A_pos': S.A_POS, 'B_pos': S.B_POS, 'tool_quat': home_quat,
            'qA': qA, 'qB': qB, 'q0': q0,
            'straight_line': {'q': line, 'clearance': cl},
            'moves': [{'name': '① 시작→A', **t1}, {'name': '② A→B (회피)', **t2}],
            'settings': {'act': S.ACT, 'pos_tol': S.POS_TOL, 'ori_tol': S.ORI_TOL, 'ik_seeds': S.IK_SEEDS,
                         'trajopt_seeds': S.TRAJOPT_SEEDS, 'robot_yml': str(S.ROBOT_YML)}}
    (out / 'plan.json').write_text(json.dumps(data))
    log('저장:', out / 'plan.json')

    if r2 is not None:
        events = []
        for mv in data['moves']:
            q_now = r2.current()
            gap = max(abs(a - b) for a, b in zip(q_now, mv['position'][0]))
            if gap > 0.01:
                log(f'   현재 자세가 계획 시작과 {math.degrees(gap):.2f}도 다름 → 중단'); break
            log(f'R2 로 실행: {mv["name"]} ({mv["n"]}점, {mv["duration"]:.2f}초) → {ACTION}')
            ev = r2.execute(mv)
            ev['name'] = mv['name']
            events.append(ev)
            st = (f'응답 {ev.get("result_time", 0) - ev["sent"]:.2f}초 뒤 (status {ev.get("result_status")}, '
                  f'error_code {ev.get("error_code")})') if ev.get('accepted') else '거부됨'
            ar = f'도착 {ev["settled"] - ev["sent"]:.2f}초' if ev.get('settled') else '도착 확인 못함'
            log(f'   수락 {ev.get("accepted")}, {st}, {ar}, 마지막 오차 {ev.get("final_err", float("nan")) * 1000:.3f} mrad')
            r2.spin(0.5)
        (out / 'exec.json').write_text(json.dumps({'events': events, 'joint_states': r2.samples}))
        log('저장:', out / 'exec.json')
    return 0


if __name__ == '__main__':
    sys.exit(main())
