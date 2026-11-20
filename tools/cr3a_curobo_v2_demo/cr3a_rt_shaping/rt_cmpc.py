"""[2026-10-14] 성형된 경로를 최적화하는 MPC (C 방식) — cuRoboV2 ModelPredictiveControl 을 실제 시간표대로 쓰고, 그 안에 입력성형기.

사용자 방향 (2026-10-14): "최적화 후 성형기 적용이 아니라 curobo 에서 최적화 할 때 성형기를 넣은 경로를 최적화".
  - 최적화 변수 = 성형 전 명령 u (B-스플라인 매듭). cuRobo 안에서 y = 성형기 · [이미 실행된 u, u] 를 만들어 충돌·자기충돌·도구 자세는 y 로,
    관절 상태 비용 (위치·속도·가속·저크 한계, 매끄러움, 목표) 은 u 로 계산한다 (cuRobo 사본 curobo_v2_shaping 의
    RobotStateTransition.set_input_shaper — 매끄러움을 y 에만 걸면 u 에 진동 주파수 성분이 숨어 주파수 오차에 약해짐, 작업기록 1-5).
  - 로봇에 보내는 것 = 같은 성형기·같은 기억 (rt_stream) 으로 만든 y → 이어 붙인 명령 전체 = 실행된 u 전체의 컨볼루션.
왜 '당근' MPC (rt_mpc, Mirobot 방식) 를 안 쓰나: 당근 MPC 는 50 ms 마다 자기 시간으로 350 ms 씩 앞서 가는 '갈 방향' 이고 실제 시간표는
  추종기가 정한다. 성형은 시간표 위의 컨볼루션이라 실제 시간표대로 계획하는 MPC 안에서만 맞는다.
시간표 (cuRoboV2 B-스플라인 MPC 실측 2026-10-13, rt_mpc 설명): 상태 궤적 점 간격 = optimization_dt, 매듭 = 4 점, 첫 매듭 구간은 시작 상태로
  다 정해짐 → 한 주기 = 매듭 2 개 (정해진 구간 + 이번에 바꾸는 구간): optimization_dt = 주기 / 8 (20 Hz → 6.25 ms). warm start 는 V2 가
  매듭 1 개만 밀어서 1 개 더 민다. 다음 주기 시작 상태 = 이번 궤적의 8 번째 점 (정확히 한 주기 뒤) = 실제로 실행된 u (Stream.cmd).
"""
import math
import time
import numpy as np
import torch

from curobo.model_predictive_control import ModelPredictiveControl, ModelPredictiveControlCfg
from curobo.scene import Scene
from curobo.types import JointState
from rt_config import CMpcCfg  # noqa: F401  (설정은 rt_config 한곳)


def shaper_matrix(h, grid_s: float, opt_dt: float, H: int, n_f: int = 0):
    """1 ms 격자 커널 h 를 MPC 상태 궤적 점 (간격 opt_dt) 에 적용하는 행렬: y = W_u @ u + W_h @ hist.
    n_f = 계획 앞의 '지난' 점 수 (첫 구간 자유화, CMpcCfg.knot_s: 계획을 한 매듭 앞 시각에서 시작) → 점 j 의 시각 = (j − n_f)·opt_dt
    (지금 = 0). 지난 점들의 u 는 지어낸 등가속 구간이라 쓰지 않고 실제 실행 기억 hist 로 읽음 (W_u 의 앞 n_f 열 = 0).
    hist = 지금 앞의 u (시각 −Hh·opt_dt … −opt_dt, 오래된 것부터). 커널 칸 m (지연 m·grid_s) 의 u 값은 앞뒤 두 점을 직선으로 이어 읽음
    (B-스플라인 u 는 매끄러워 6.25 ms 간격 직선 오차 ≈ a·dt²/8 = 0.04 mrad). 반환 (W_u [H, H], W_h [H, Hh], Hh)."""
    h = np.asarray(h, float)
    D = (len(h) - 1) * grid_s
    Hh = int(math.ceil(D / opt_dt - 1e-9)) + 1 + n_f
    Hr = H - n_f                                                    # 지금부터의 계획 점
    W = np.zeros((H, Hh + Hr))
    for j in range(H):
        for m, w in enumerate(h):
            if w == 0.0:
                continue
            x = ((j - n_f) * opt_dt - m * grid_s) / opt_dt + Hh
            i0 = int(math.floor(x + 1e-9))
            r = x - i0
            if r < 1e-9:
                W[j, i0] += w
            else:
                W[j, i0] += w * (1.0 - r)
                W[j, i0 + 1] += w * r
    Wu = np.zeros((H, H))
    Wu[:, n_f:] = W[:, Hh:]
    return Wu, W[:, :Hh], Hh


def hermite_fine(P, V, T, n_sub):
    """점 간격 T 의 (위치 P, 속도 V) [n, J] → 한 구간을 n_sub 로 나눈 (q, v, a) [(n−1)·n_sub + 1, 3, J] (3 차 Hermite: B-스플라인 3 차
    조각을 점 간격으로 나눈 것이라 그대로 같음)."""
    n, J = P.shape
    s = np.arange(n_sub) / n_sub
    out = np.zeros(((n - 1) * n_sub + 1, 3, J))
    for k in range(n - 1):
        p0, p1, m0, m1 = P[k], P[k + 1], V[k] * T, V[k + 1] * T
        u = s[:, None]
        q = (2 * u**3 - 3 * u**2 + 1) * p0 + (u**3 - 2 * u**2 + u) * m0 + (-2 * u**3 + 3 * u**2) * p1 + (u**3 - u**2) * m1
        v = ((6 * u**2 - 6 * u) * p0 + (3 * u**2 - 4 * u + 1) * m0 + (-6 * u**2 + 6 * u) * p1 + (3 * u**2 - 2 * u) * m1) / T
        a = ((12 * u - 6) * p0 + (6 * u - 4) * m0 + (-12 * u + 6) * p1 + (6 * u - 2) * m1) / (T * T)
        out[k * n_sub:(k + 1) * n_sub] = np.stack([q, v, a], axis=1)
    p0, p1, m0, m1 = P[-2], P[-1], V[-2] * T, V[-1] * T
    out[-1] = np.stack([P[-1], V[-1], ((6 * p0 + 2 * m0 - 6 * p1 + 4 * m1) / (T * T))])
    return out


def _js(q, names, device, v=None, a=None):
    t = torch.as_tensor(np.asarray(q, np.float32).reshape(1, -1), device=device)
    st = JointState.from_position(t, joint_names=list(names))
    if v is not None:
        st.velocity = torch.as_tensor(np.asarray(v, np.float32).reshape(1, -1), device=device)
    if a is not None:
        st.acceleration = torch.as_tensor(np.asarray(a, np.float32).reshape(1, -1), device=device)
    return st


class CMpc:
    """시간표 MPC + 입력성형 (C 방식). step(s0, past) → 이번 주기부터 2 주기의 u (1 ms 칸) 와 정보."""

    def __init__(self, robot_cfg: dict, c: CMpcCfg, activation: float, voxel, cuboids, kernel, grid_s: float = 0.001):
        self.c = c
        self.dt = 1.0 / c.rate_hz
        self.first_free = c.knot_s > 0
        if self.first_free:
            # [2026-10-15] 첫 실행 구간 저크 자유화 (CMpcCfg.knot_s 설명): 매듭 간격 고정, 시작 상태를 한 매듭 되돌려 줌 (지난 점 4 개)
            self.opt_dt = c.knot_s / 4.0
            self.M = int(round(self.dt / self.opt_dt))
            assert abs(self.M * self.opt_dt - self.dt) < 1e-9, f'주기 {self.dt} s 가 점 간격 {self.opt_dt} s 의 배수가 아님'
            self.n_f = 4
            self.n_knots = int(round(c.horizon_s / c.knot_s)) + 1  # 지난 매듭 1 개 더
        else:
            self.opt_dt = self.dt / 8.0                             # 매듭 2 개 (4 점씩) = 한 주기
            self.M = 8                                              # 한 주기 = 상태 궤적 점 8 개
            self.n_f = 0
            self.n_knots = int(round(c.horizon_s / (self.opt_dt * 4)))
        self.g = grid_s
        opt_cfgs = ['mpc/lbfgs_mpc.yml']
        if c.nonterminal != 0.05 or c.bound_scale != 1.0 or c.target_scale != 1.0 or c.ls_scales:
            import copy
            import os

            import curobo
            import yaml
            y = yaml.safe_load(open(os.path.join(os.path.dirname(curobo.__file__), 'content/configs/task/mpc/lbfgs_mpc.yml')))
            y = copy.deepcopy(y)
            cs = y['rollout']['cost_cfg']['cspace_cfg']
            cs['cspace_non_terminal_weight_factor'] = float(c.nonterminal)
            cs['weight'] = [x * c.bound_scale for x in cs['weight'][:4]] + list(cs['weight'][4:])
            cs['cspace_target_weight'] = float(cs['cspace_target_weight']) * c.target_scale
            if c.ls_scales:
                y['optimizer']['line_search_scale'] = [float(v) for v in c.ls_scales.split(',')]
            opt_cfgs = [y]
        if c.jerk > 0:
            import copy
            robot_cfg = copy.deepcopy(robot_cfg)
            cs = robot_cfg['robot_cfg']['kinematics']['cspace'] if 'robot_cfg' in robot_cfg else robot_cfg['kinematics']['cspace']
            cs['max_jerk'] = [float(c.jerk)] * len(cs['max_jerk'])
        cfg = ModelPredictiveControlCfg.create(
            optimizer_configs=opt_cfgs, num_control_points=self.n_knots,
            robot=robot_cfg, scene_model=Scene(cuboid=list(cuboids), voxel=[voxel]), use_cuda_graph=True,
            optimization_dt=self.opt_dt, interpolation_steps=4,
            optimizer_collision_activation_distance=activation,
            warm_start_optimization_num_iters=c.warm_iters, cold_start_optimization_num_iters=c.cold_iters)
        self.mpc = ModelPredictiveControl(cfg)
        tm = self.mpc.trajectory_execution_manager
        assert tm.command_start_idx == 4, tm.command_start_idx      # 첫 매듭 구간 = 시작 상태로 정해짐 (위 시간표 설명의 전제)
        self.names = list(self.mpc.joint_names)
        self.device = self.mpc.device_cfg.device
        lim = self.mpc.kinematics.get_joint_limits()
        k = [list(lim.joint_names).index(n) for n in self.names]
        self.vmax = lim.velocity[1].cpu().numpy()[k]
        self.amax = lim.acceleration[1].cpu().numpy()[k]
        self.qlim = (lim.position[0].cpu().numpy()[k].astype(float), lim.position[1].cpu().numpy()[k].astype(float))
        # 성형기를 모든 rollout (최적화·평가) 에 — CUDA graph 를 잡기 전 (첫 풀기 전) 에
        self.kernel = kernel
        self.H = int(self.mpc.transition_model.horizon)
        self.shaped = kernel is not None and (kernel.L > 1 or c.force_shaper)
        if self.shaped:
            Wu, Wh, self.Hh = shaper_matrix(kernel.h, grid_s, self.opt_dt, self.H, self.n_f)
            dev, dt = self.device, torch.float32
            self.W_u = torch.as_tensor(Wu, dtype=dt, device=dev)
            self.W_h = torch.as_tensor(Wh, dtype=dt, device=dev)
            self.hist = torch.zeros((self.Hh, len(self.names)), dtype=dt, device=dev)
            for r in self.mpc.get_all_rollout_instances():
                for t in (r.transition_model, r.metrics_transition_model):
                    t.set_input_shaper(self.W_u, self.W_h, self.hist)
            self._Wu_np, self._Wh_np = Wu, Wh
        else:
            self.Hh = 0
        # 시간 기준 궤적 따라가기 (cuRobo 사본 cost_manager_robot.set_reference_tracking) — CUDA graph 를 잡기 전에
        self.track = c.track_w > 0
        self.ref_delay = float(kernel.mean_delay_s) if (kernel is not None and kernel.L > 1) else 0.0
        if self.track:
            self.ref = torch.zeros((self.H, len(self.names)), dtype=torch.float32, device=self.device)
            self.ref_w = torch.full((self.H,), float(c.track_w), dtype=torch.float32, device=self.device)
            self.ref_u = torch.zeros_like(self.ref) if c.track_u_w > 0 else None
            self.ref_u_w = torch.full((self.H,), float(c.track_u_w), dtype=torch.float32, device=self.device) if c.track_u_w > 0 else None
            for r in self.mpc.get_all_rollout_instances():
                for cm in (r.cost_manager, r.metrics_cost_manager):
                    if cm is not None:
                        cm.set_reference_tracking(self.ref, self.ref_w, self.ref_u, self.ref_u_w)
        self._setup = False
        self.mode = 'off'
        self.goal = None
        self.path = None
        self.ref_t = 0.0
        self.clock = 0.0
        self.t_progress = 0.0
        self.q_now = None
        self.plan = None

    # ---------------- 켜기·목표 (rt_mpc 와 같은 따라가기, 기준 = 실제 명령 u) ----------------
    def start(self, q):
        st = _js(q, self.names, self.device)
        if self.track:
            self.ref.copy_(torch.as_tensor(np.repeat(np.asarray(q, np.float32)[None], self.H, 0), device=self.device))
            if self.ref_u is not None:
                self.ref_u.copy_(self.ref)
        if self.shaped:
            self.hist.copy_(torch.as_tensor(np.repeat(np.asarray(q, np.float32)[None], self.Hh, 0), device=self.device))
        if not self._setup:
            self.mpc.setup(st)
            self._setup = True
        else:
            self.mpc.reset_robot(st)
        self.mpc.disable_tool_pose_tracking()
        self.mpc.enable_joint_position_tracking()
        self.q_now = np.array(q, float)
        self.goal = None
        self.plan = None                                            # 다음 틱에 새로 계획
        self.hold(np.asarray(q, float))

    def restart(self, q, v=None, a=None):
        """앞 해 (warm start) 를 버리고 그 상태에서 새로 (다음 풀기는 cold start)."""
        self.mpc.reset_robot(_js(q, self.names, self.device, v, a))

    def hold(self, q=None):
        q = self.q_now if q is None else np.asarray(q, float)
        self.path, self.mode = None, 'hold'
        self.plan = None                                            # 목표가 바뀜 → 다음 틱에 새로 계획
        if self.track:
            self.ref.copy_(torch.as_tensor(np.repeat(np.asarray(q, np.float32)[None], self.H, 0), device=self.device))
            if self.ref_u is not None:
                self.ref_u.copy_(self.ref)
        self._set_goal(q)

    def follow(self, t, q, from_nearest=False):
        t = np.asarray(t, float)
        q = np.asarray(q, float)
        self.path = (t - t[0], q)
        ref = 0.0
        if from_nearest and self.q_now is not None:
            ref = float(self.path[0][int(np.argmin(np.max(np.abs(q - self.q_now), axis=1)))])
        self.ref_t, self.t_progress, self.mode = ref, self.clock, 'move'
        self.plan = None                                            # 경로가 바뀜 → 다음 틱에 새로 계획
        self._set_goal(self._path_at(ref + self.c.lookahead_s))

    def _set_goal(self, q):
        if self.goal is not None and np.array_equal(self.goal, q):
            return
        self.goal = np.array(q, float)
        self.mpc.update_goal_state(_js(q, self.names, self.device))

    def _path_at(self, tt):
        t, q = self.path
        tt = min(max(tt, 0.0), t[-1])
        i = int(np.searchsorted(t, tt, side='right')) - 1
        if i >= len(t) - 1:
            return q[-1].copy()
        a = (tt - t[i]) / max(t[i + 1] - t[i], 1e-9)
        return q[i] + a * (q[i + 1] - q[i])

    def command(self):
        return self.q_now.copy()

    # ---------------- 한 주기 ----------------
    def history(self, past_q, past_dt):
        """past_q [n, J] = 지금 앞 1 ms 칸들의 실행된 u (오래된 것부터, 마지막 = 지금 1 ms 전) → MPC 점 시각 (−Hh·opt_dt … −opt_dt) 값."""
        if not self.shaped:
            return None
        n = len(past_q)
        tq = (np.arange(n) - n) * past_dt                          # 시각 (마지막 = −past_dt)
        tm = -np.arange(self.Hh, 0, -1) * self.opt_dt
        assert tm[0] >= tq[0] - 1e-9, f'기억이 짧음: {tq[0]:.4f} > {tm[0]:.4f}'
        return np.stack([np.interp(tm, tq, past_q[:, j]) for j in range(past_q.shape[1])], axis=1)

    def step(self, s0, hist=None, elapsed=None) -> dict:
        """s0 = 지금 (스탬프) 의 실행된 u 상태 (q, v, a), hist = history() 결과, elapsed = 지난 풀기 뒤 흐른 시간 (기본 MPC 주기).
        반환: u1 [2 주기·1 ms + 1, 3, J] (u1[0] = s0), traj (MPC 상태 궤적 점 위치 [H, J]), ok, feasible, mode, final, at_end, stuck,
        ms 등. 계획은 self.plan 에 남아 sample() 로 이어서 읽음 (제어 주기가 MPC 주기보다 짧을 때)."""
        t0 = time.perf_counter()
        el = self.dt if elapsed is None else float(elapsed)
        self.clock += el
        q0 = np.asarray(s0[0], float)
        self.q_now = q0.copy()
        info = {'mode': self.mode, 'stuck': False, 'final': False}
        if self.mode == 'move':
            t, q = self.path
            end = float(t[-1])
            win = (t >= self.ref_t - self.c.lookahead_s - 1e-9) & (t <= self.ref_t + self.c.lookahead_s + 0.5)
            idx = np.nonzero(win)[0]
            if len(idx):
                d = np.max(np.abs(q[idx] - q0), axis=1)
                k = int(np.nonzero(d <= d.min() + 1e-4)[0][-1])
                if d[k] < self.c.follow_tol_rad:
                    new = min(self.ref_t + el, float(t[idx[k]]) + self.c.ahead_s, end)
                    if new > self.ref_t + 1e-9:
                        self.ref_t = new
                        self.t_progress = self.clock
            if self.track:
                # 성형이 켜져 있으면 y 의 기준 = 경로를 성형기 평균 지연만큼 늦춘 것: 최단 시간 경로 (한계에 붙음) 를 y 가 제시간에
                #  따라가려면 u 가 한계를 넘어 앞서야 해서 발산했다 (2026-10-14 5 Hz ZVD 실측). 지연은 성형의 몫으로 받아들이고, 코너에서
                #  경로를 벗어나는 것 (안쪽으로 깎임) 은 그대로 최적화가 보정
                #  (첫 구간 자유화면 계획 앞 n_f 점은 지난 시각 → 점 j 의 시각 = (j − n_f)·opt_dt)
                tj = self.ref_t - self.ref_delay + (np.arange(self.H) - self.n_f) * self.opt_dt
                R = np.stack([self._path_at(x) for x in tj])
                self.ref.copy_(torch.as_tensor(R.astype(np.float32), device=self.device))
                if self.ref_u is not None:
                    Ru = np.stack([self._path_at(x) for x in self.ref_t + (np.arange(self.H) - self.n_f) * self.opt_dt])
                    self.ref_u.copy_(torch.as_tensor(Ru.astype(np.float32), device=self.device))
                self._set_goal(R[-1])
            else:
                self._set_goal(self._path_at(self.ref_t + self.c.lookahead_s))
            info['final'] = self.ref_t + self.c.lookahead_s >= end - 1e-9
            info['at_end'] = self.ref_t >= end - 1e-9
            info['stuck'] = self.clock - self.t_progress > self.c.stuck_s
        if self.shaped and hist is not None:
            self.hist.copy_(torch.as_tensor(np.asarray(hist, np.float32), device=self.device))
        tm = self.mpc.trajectory_execution_manager
        buf = getattr(tm, '_current_action_trajectory', None)
        if buf is not None and buf.shape[-2] > 2:
            if self.first_free:
                # warm start: 한 주기 = 매듭 s 개 (매듭 100 ms, 20 Hz 면 0.5) → 앞 해를 s 매듭 민 씨앗 (제어점을 이웃끼리 직선으로).
                #  V2 가 안에서 1 매듭 미는 것 (get_shifteaction_dim_buffer) 을 미리 되돌려 둠
                s = self.dt / self.c.knot_s
                nxt = torch.cat([buf[..., 1:, :], buf[..., -1:, :]], dim=-2)
                kp = (1.0 - s) * buf + s * nxt
                tm._current_action_trajectory = torch.cat([kp[..., :1, :], kp[..., :-1, :]], dim=-2)
            else:
                # warm start: V2 는 앞 해를 매듭 1 개만 민다 → 한 주기 = 매듭 2 개라 1 개 더 (2026-10-13 consistent 방식과 같음)
                rolled = buf.roll(-1, dims=-2)
                rolled[..., -1:, :] = buf[..., -1:, :]
                tm._current_action_trajectory = rolled
        if self.first_free:
            # 시작 상태를 한 매듭 (n_f 점) 앞 시각으로 되돌림 — 등가속 역외삽이라 V2 가 시작 상태로 고정하는 첫 구간 (저크 0) 이 정확히 지금
            #  상태에서 끝남 → 실행하는 둘째 구간의 저크는 첫 자유 제어점이 정함 (작업기록 5-13)
            hb = self.n_f * self.opt_dt
            v0, a0 = np.asarray(s0[1], float), np.asarray(s0[2], float)
            st = _js(q0 - v0 * hb + 0.5 * a0 * hb * hb, self.names, self.device, v0 - a0 * hb, a0)
        else:
            st = _js(q0, self.names, self.device, s0[1], s0[2])
        r = self.mpc.optimize_action_sequence(st)
        full = tm._current_joint_state_trajectory
        P, V, A = (x[0].detach().cpu().numpy().astype(float) for x in (full.position, full.velocity, full.acceleration))
        ok = bool(np.all(np.isfinite(P)) and np.all(np.isfinite(V)))
        nf = self.n_f
        n_pts = 2 * self.M + 1                                      # 지금 ~ 2 주기
        sub = int(round(self.opt_dt / self.g * 1e6)) / 1e6
        if ok and abs(sub - round(sub)) < 1e-9:
            u1 = hermite_fine(P[nf:nf + n_pts], V[nf:nf + n_pts], self.opt_dt, int(round(sub)))
        elif ok:
            # 점 간격이 1 ms 의 배수가 아님 (예: 6.25 ms): 1 ms 칸 시각에서 Hermite 를 직접 계산
            u1 = self._fine_at_ms(P[nf:nf + n_pts], V[nf:nf + n_pts], int(round(2 * self.dt / self.g)))
        else:
            u1 = None
        self.plan = (P[nf:], V[nf:]) if ok else None                # 지금부터 (sample 의 시각 0 = 지금)
        info.update(ok=ok, feasible=bool(r.success.view(-1)[0]) if r.success is not None else True, u1=u1,
                    traj=P, traj_v=V, traj_a=A, n_f=nf, ms=round((time.perf_counter() - t0) * 1000, 1),
                    start_err=float(np.max(np.abs(P[nf] - q0))) if ok else None,
                    acc_ratio=round(float(np.max(np.abs(A[nf:nf + n_pts]) / self.amax)), 3) if ok else None,
                    vel_ratio=round(float(np.max(np.abs(V[nf:nf + n_pts]) / self.vmax)), 3) if ok else None)
        return info

    def sample(self, n0: int, n_ms: int):
        """마지막 계획 (step) 의 n0 ~ n0 + n_ms ms (계획 시작 기준, 1 ms 칸) 의 (q, v, a) [n_ms + 1, 3, J]. 계획 끝을 넘으면 끝 점."""
        P, V = self.plan
        return self._fine_at_ms(P, V, n_ms, n0)

    def _fine_at_ms(self, P, V, n_ms, n0=0):
        T = self.opt_dt
        out = np.zeros((n_ms + 1, 3, P.shape[1]))
        for i in range(n_ms + 1):
            t = min((n0 + i) * self.g, (len(P) - 1) * T)
            k = min(int(math.floor(t / T + 1e-9)), len(P) - 2)
            u = (t - k * T) / T
            p0, p1, m0, m1 = P[k], P[k + 1], V[k] * T, V[k + 1] * T
            q = (2 * u**3 - 3 * u**2 + 1) * p0 + (u**3 - 2 * u**2 + u) * m0 + (-2 * u**3 + 3 * u**2) * p1 + (u**3 - u**2) * m1
            v = ((6 * u**2 - 6 * u) * p0 + (3 * u**2 - 4 * u + 1) * m0 + (-6 * u**2 + 6 * u) * p1 + (3 * u**2 - 2 * u) * m1) / T
            a = ((12 * u - 6) * p0 + (6 * u - 4) * m0 + (-12 * u + 6) * p1 + (6 * u - 2) * m1) / (T * T)
            out[i] = (q, v, a)
        return out

    def shaped_traj(self, P, hist):
        """(검사용) cuRobo 안과 같은 행렬로 만든 y [H, J]."""
        if not self.shaped:
            return P
        return self._Wu_np @ P + self._Wh_np @ hist
