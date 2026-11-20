"""cuRoboV2 ModelPredictiveControl 감싸기 — Mirobot 실시간 제어(mirobot_cumotion/curobo_v2_realtime.py)와 같은 방식.

  - 매 주기 optimize_action_sequence(MPC 상태) → action_sequence 끝점을 다음 MPC 상태로 (자기 출력 되먹임, 자르지 않음).
    optimization_dt = 주기, 매듭 수 기본(16). MPC 상태는 실제보다 빨리 앞서 가는 "당근" 이고, 로봇 명령은 rt_motion.track_target 이
    그 점을 한계 안에서 쫓아 만든다 (rt_run). 자른 상태를 되먹이면 MPC 가 헤맨다 (Mirobot 주석·2026-10-13 CR3A 비교 실측).
  - 목표는 관절값 (V2 6축 IK 목표 문제 #711 회피), 손끝 추종은 끔.
  - 전역 경로 따라가기: 기준 시각을 주기마다 dt 만큼만 진행하되, MPC 상태가 경로에서 벗어나 있으면(피하는 중) 멈춤.
    목표 = 경로의 (기준 시각 + lookahead) 점. 기준 시각이 stuck_s 동안 못 나아가면 "막힘".
  - resync: 안전 검사가 개입해 로봇이 MPC 쪽으로 못 가면 MPC 를 로봇 명령 자리로 맞춤 (rt_run 은 그 자리에서 새로 시작 — 명령의 가속까지
    넣으면 MPC 가 발산, 2026-10-14, rt_config.MpcCfg 설명).
참고 (2026-10-13 cuRoboV2 소스·실측): 상태 궤적 점 간격 = optimization_dt, action_sequence = 매듭 하나 뒤의 점들,
첫 매듭은 시작 상태로 다 정해짐, warm start 는 매듭 하나 밈 — 그래서 action_sequence 끝점은 MPC 시간으로 7·dt 뒤.
"""
import time

import numpy as np
import torch

from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg
from curobo.scene import Scene
from curobo.types import JointState

from rt_config import MpcCfg

TIE_RAD = 1e-4              # 경로에서 가장 가까운 점 고르기: 이 안 (rad) 은 같은 거리로 보고 가장 뒤 점 (경로 끝 속도 0 구간)
WIN_AHEAD_S = 0.5           # 가장 가까운 점을 찾는 창의 앞쪽 = 기준 + lookahead + 이만큼 (s)


def _q32(q):
    """관절값 → MPC 상태에 넣는 float32 값의 float64 배열 (GPU 상태 위치를 읽은 것과 같은 값)."""
    return np.asarray(q, np.float32).reshape(-1).astype(float)


def mpc_optimizer_cfg(pos_activation):
    """cuRoboV2 기본 MPC 최적화 설정 (content/configs/task/mpc/lbfgs_mpc.yml) 에서 관절 위치 한계 비용이 켜지는 거리 (cspace
    activation_distance[0], 관절 범위의 비율) 만 바꾼 dict — rt_config.MpcCfg.pos_limit_activation 설명. 속도·가속·저크 쪽은 그대로."""
    import os

    import curobo
    import yaml
    with open(os.path.join(os.path.dirname(curobo.__file__), 'content/configs/task/mpc/lbfgs_mpc.yml')) as f:
        y = yaml.safe_load(f)
    cs = y['rollout']['cost_cfg']['cspace_cfg']
    cs['activation_distance'] = [float(pos_activation)] + list(cs['activation_distance'][1:])
    return y


def _js(q, names, device, v=None, a=None):
    t = torch.as_tensor(np.asarray(q, np.float32).reshape(1, -1), device=device)
    st = JointState.from_position(t, joint_names=list(names))
    st.velocity = torch.zeros_like(t) if v is None else torch.as_tensor(np.asarray(v, np.float32).reshape(1, -1), device=device)
    st.acceleration = torch.zeros_like(t) if a is None else torch.as_tensor(np.asarray(a, np.float32).reshape(1, -1), device=device)
    return st


class Mpc:
    def __init__(self, robot_cfg: dict, c: MpcCfg, activation: float, voxel, cuboids):
        self.c = c
        self.dt = 1.0 / c.rate_hz
        cfg = ModelPredictiveControlCfg.create(
            optimizer_configs=[mpc_optimizer_cfg(c.pos_limit_activation)],
            robot=robot_cfg, scene_model=Scene(cuboid=list(cuboids), voxel=[voxel]), use_cuda_graph=True,
            optimization_dt=self.dt, interpolation_steps=c.interpolation_steps,
            optimizer_collision_activation_distance=activation,
            warm_start_optimization_num_iters=c.warm_iters, cold_start_optimization_num_iters=c.cold_iters)
        self.mpc = ModelPredictiveControl(cfg)
        self.names = list(self.mpc.joint_names)
        self.device = self.mpc.device_cfg.device
        lim = self.mpc.kinematics.get_joint_limits()
        k = [list(lim.joint_names).index(n) for n in self.names]
        self.vmax = lim.velocity[1].cpu().numpy()[k]          # URDF (cr3a_curobo.yml) 한계 그대로
        self.amax = lim.acceleration[1].cpu().numpy()[k]
        self.qlim = (lim.position[0].cpu().numpy()[k].astype(float), lim.position[1].cpu().numpy()[k].astype(float))
        self._setup = False
        self.state = None
        self._q_cpu = None                  # [2026-10-27 R-1] MPC 상태 관절값의 CPU 사본 (command) — 상태를 넣을 때·MPC 가 낼 때 같이 둠
        self._goal_js = None                # [2026-10-27 R-1] 목표 JointState (한 번 만들고 위치만 바꿔 넣음)
        self.mode = 'off'
        self.goal = None
        self.path = None
        self.ref_t = 0.0
        self.back_anchor = None             # 그 자리에서 비키기 (back) 의 고정 목표
        self.clock = 0.0
        self.t_progress = 0.0

    # ---------------- 켜기·목표 ----------------
    def start(self, q):
        """정지 상태 q 에서 (다시) 시작, 그 자리 지키기."""
        st = _js(q, self.names, self.device)
        if not self._setup:
            self.mpc.setup(st)
            self._setup = True
        else:
            self.mpc.reset_robot(st)
        self.mpc.disable_tool_pose_tracking()
        self.mpc.enable_joint_position_tracking()
        self.state, self._q_cpu = st, _q32(q)
        self.goal = None
        self.hold(np.asarray(q, float))

    def resync(self, q, v=None, a=None, restart=False):
        """MPC 상태를 로봇 명령 상태로 맞춤 (목표·경로·진행은 그대로). 안전 검사가 막아 로봇이 서 있는데 MPC 는 40° 앞에서
        계획을 이어 가다 내려오는 손에 닿은 실측이 있어 넣음. v·a 가 None 이면 0 (rt_config.MpcCfg 설명 — rt_run 은 restart 로만 부름).
        restart = 앞 해도 버리고 그 자리에서 새로 시작 (reset_robot → 다음 주기 cold start). [2026-10-13] 그때 코드로는 reset_robot 으로
        맞추면 계획이 나빠져서 (다리 7~16 실패) 상태만 바꿨다."""
        st = _js(q, self.names, self.device, v, a)
        if restart:
            self.mpc.reset_robot(st)
        self.state, self._q_cpu = st, _q32(q)

    def hold(self, q=None):
        q = self.command() if q is None else np.asarray(q, float)
        self.path, self.mode = None, 'hold'
        self.set_goal(q)

    def follow(self, t, q, from_nearest=False, near_q=None):
        """전역 계획 궤적 (시각 [N], 관절 [N, 6]) 을 따라간다. from_nearest = 지금 MPC 자세와 가장 가까운 경로 점부터
        (near_q 를 주면 그 자세와 — [2026-10-16] 물러나기 뒤에는 로봇 명령 자세: MPC 상태 (당근) 는 물러나는 동안 로봇보다 25~40° 뒤쪽이라
        그것과 가장 가까운 점부터 이으면 다시 뒤로 갔다 옴, WMX 13 번 다리)."""
        t = np.asarray(t, float)
        q = np.asarray(q, float)
        self.path = (t - t[0], q)
        ref = 0.0
        if from_nearest and (near_q is not None or self.state is not None):
            ref_q = self.command() if near_q is None else np.asarray(near_q, float)
            ref = float(self.path[0][int(np.argmin(np.max(np.abs(q - ref_q), axis=1)))])
        self.ref_t, self.t_progress, self.mode = ref, self.clock, 'move'
        self.set_goal(self.path_at(ref + self.c.lookahead_s))

    def back(self, anchor):
        """[2026-10-17 사용자 결정 '그 자리에서 비키기'] 경로 목표를 내려놓고 anchor (로봇 명령 자리) 를 목표로 고정 — MPC 충돌 비용이 물체
        반대쪽으로 밀고 목표가 되돌려 당김 (지키기와 같음). 경로·기준 시각은 그대로 (다시 앞으로 갈 때 rt_run 이 follow(from_nearest)).
        경로가 없으면 False. [2026-10-27 R-1] 지나온 경로를 따라 물러나기 (2026-10-16, anchor 없음) 는 뺌."""
        if self.path is None:
            return False
        self.mode = 'back'
        self.back_anchor = np.asarray(anchor, float).copy()
        self.set_goal(self.back_anchor)
        return True

    def set_goal(self, q):
        """MPC 목표 관절값 (같으면 그대로). [2026-10-28 R-1] 공개 — rt_server 가 지키기로 바꿀 때 목표를 맞춤 (예전 _set_goal)."""
        if self.goal is not None and np.array_equal(self.goal, q):
            return
        self.goal = np.array(q, float)
        if self._goal_js is None:
            self._goal_js = _js(q, self.names, self.device)
        else:                               # [R-1] 같은 JointState 에 위치만 (속도·가속 0 그대로 — cuRobo 는 목표 버퍼로 복사해 씀)
            self._goal_js.position.copy_(torch.from_numpy(np.asarray(q, np.float32).reshape(1, -1)))
        self.mpc.update_goal_state(self._goal_js)

    def path_at(self, ts):
        """따라가는 경로의 시각 ts (s, 경로 시작 기준 — 스칼라 또는 [n]) 의 관절값 ([6] 또는 [n, 6]): 점 사이 직선, 범위 밖은 끝 점.
        [2026-10-27 R-1] 여러 시각을 한 번에 (rt_run 경로 앞 검사) — 값은 점 하나씩 계산과 같음 (tests/test_api.py)."""
        t, q = self.path
        tt = np.clip(np.asarray(ts, float), 0.0, t[-1])
        i = np.searchsorted(t, tt, side='right') - 1
        j = np.minimum(i, len(t) - 2)                   # 끝 점 (i ≥ len − 1) 은 아래 where 가 q[-1] 로
        a = (tt - t[j]) / np.maximum(t[j + 1] - t[j], 1e-9)
        return np.where((i >= len(t) - 1)[..., None], q[-1], q[j] + a[..., None] * (q[j + 1] - q[j]))

    def advance_ref(self, q_now, dt) -> dict:
        """경로를 따라가는 중 (move) 기준 시각을 한 주기 진행 (Mirobot 방식): 경로의 [기준 − lookahead, 기준 + lookahead + 0.5 s] 점 중 q_now 와
        가장 가까운 점 (같은 거리면 가장 뒤 점 — 경로 끝 속도 0 구간) 이 follow_tol_rad 안이면 기준 = min(기준 + dt, 그 점 + ahead_s, 끝)
        (나아갔으면 t_progress = clock), 아니면 (피하는 중) 그대로. 반환 {'final': MPC 목표 (기준 + lookahead) = 경로 끝, 'at_end': 기준 = 끝,
        'stuck': stuck_s 넘게 못 나아감}."""
        t, q = self.path
        end = float(t[-1])
        win = (t >= self.ref_t - self.c.lookahead_s - 1e-9) & (t <= self.ref_t + self.c.lookahead_s + WIN_AHEAD_S)
        idx = np.nonzero(win)[0]
        if len(idx):
            d = np.max(np.abs(q[idx] - q_now), axis=1)
            k = int(np.nonzero(d <= d.min() + TIE_RAD)[0][-1])
            if d[k] < self.c.follow_tol_rad:
                new = min(self.ref_t + dt, float(t[idx[k]]) + self.c.ahead_s, end)
                if new > self.ref_t + 1e-9:
                    self.ref_t = new
                    self.t_progress = self.clock
        return {'final': self.ref_t + self.c.lookahead_s >= end - 1e-9, 'at_end': self.ref_t >= end - 1e-9,
                'stuck': self.clock - self.t_progress > self.c.stuck_s}

    def command(self):
        """MPC 상태의 관절값 (float32 값, float64 배열). [2026-10-27 R-1] GPU 에서 읽지 않고 CPU 사본 (넣을 때·MPC 가 낼 때 같은 값)."""
        return self._q_cpu.copy()

    # ---------------- 한 주기 ----------------
    def step(self) -> dict:
        """MPC 한 번. 반환: target (다음 MPC 상태 관절값 = 로봇 명령이 쫓을 점, ok 가 아니면 NaN 일 수 있음), ok, feasible,
        mode, final, at_end, stuck, raw_acc_ratio, ms."""
        t0 = time.perf_counter()
        self.clock += self.dt
        info = {'mode': self.mode, 'stuck': False, 'final': False}     # final = 경로를 따라가는 중 MPC 목표가 경로 끝
        if self.mode == 'move':
            info.update(self.advance_ref(self.command(), self.dt))
            self.set_goal(self.path_at(self.ref_t + self.c.lookahead_s))
        r = self.mpc.optimize_action_sequence(self.state)                # (비키기 back = 목표 고정 — Mpc.back)
        seq = r.action_sequence
        nxt = JointState.from_position(seq.position[:, -1, :].clone(), joint_names=self.names)
        nxt.velocity = seq.velocity[:, -1, :].clone()
        nxt.acceleration = seq.acceleration[:, -1, :].clone()
        # 다음 상태 위치·속도, 계획 전체 가속, 해 있음을 GPU → CPU 한 번에 (GPU 를 여기서 기다림 — [2026-10-27 R-1] 예전 네 번)
        J, na = len(self.names), seq.acceleration.numel()
        parts = [nxt.position.reshape(-1).float(), nxt.velocity.reshape(-1).float(), seq.acceleration.reshape(-1).float()]
        if r.success is not None:
            parts.append(r.success.reshape(-1)[:1].float())
        host = torch.cat(parts).cpu().numpy()
        q, v, sa = host[:J].astype(float), host[J:2 * J].astype(float), host[2 * J:2 * J + na].reshape(-1, J)
        ok = bool(np.all(np.isfinite(q)) and np.all(np.isfinite(v)))
        if ok:
            self.state, self._q_cpu = nxt, q.copy()                   # 자기 출력 되먹임 (자르지 않음)
        info.update(ok=ok, feasible=bool(host[-1]) if r.success is not None else True, target=q,
                    raw_acc_ratio=round(float(np.max(np.abs(sa) / self.amax)), 3), ms=round((time.perf_counter() - t0) * 1000, 1))
        return info
