#!/usr/bin/env python3
"""CR3A 실시간 장애물 회피: 합성 깊이(카메라 1대) → 지도 → MPC → 명령 궤적 → 안전 검사 → 로봇.

  python3 rt_run.py --robot ideal --out <폴더>    시험용 (ROS 없음, 로봇 = 보낸 명령 그대로, 계산 시계 → 결정적)
  python3 rt_run.py --robot ros --out <폴더>      tools/rt_demo.sh 가 R2 (cr3a_gui/wmx_r2.sh — CyclicBuffer → WMX 시뮬레이터) 를 띄운 뒤 부름
  python3 rt_run.py --robot ideal --map oracle --out <폴더>   [검증 전용] 카메라 대신 참 형상 지도 (성형 탓 닿음 가리기, 작업기록 5-11)

한 주기 (20 Hz, Controller.cycle)
  1. 스탬프 S − lead 까지 차례가 온 카메라 영상을 지도에 쌓음 (rt_world.World.frame)
  2. 성형 전 명령 u (1 ms 칸, 2 주기 + 멈춤 꼬리) — 방식 (rt_config.Config.style):
     'cmpc'   [2026-10-14, 사용자 방향 C 방식] 성형된 경로를 최적화하는 시간표 MPC (rt_cmpc): cuRobo 안에서 충돌은 성형된 y,
              한계·매끄러움·목표는 u. MPC 궤적이 곧 u (한 주기 = 매듭 2 개)
     'carrot' 2026-10-14 전 방식: MPC 한 번 → 당근 (rt_mpc) → 당근을 속도·가속·저크 한계 안에서 쫓는 u (rt_motion)
  3. 입력성형 y = 성형기 · [실행된 u 기억, u] (rt_stream, rt_shaping, 설정 shaping.yaml) → 메시지 점 (5 ms 2 주기 + 꼬리 10 ms)
  4. 안전 검사 (성형된 y 로): 궤적+꼬리가 실제 거리 지도에서 닿으면 감속 정지 / 앞 메시지 계속 중 나은 쪽 (Controller._guard).
     감속 정지도 성형기를 거침 (사용자 결정 2026-10-14: 이어 붙인 경로 전체 = 컨볼루션)
  5. 메시지 = 앞 메시지 점 0.1 s + 새 점들 (rt_stream). 늦으면 안 보냄 (앞 메시지 = 실행 중인 u 가 계속 실행된 것으로 기억)
임무 (Controller.run): 배경 1 s → 시작→A → GO (움직이는 물체 출발) → A↔B 다리 N 개
  다리: 멈춰서 전역 계획 (실패면 그 자리를 지키며 1 s 마다 다시) → MPC 가 경로를 따라감 → 도착 (0.1° 안 0.3 s)
        막힘 3 s → 목표 자리가 새 물체에 막혔으면 경로 따라 가까이 가서 기다림, 아니면 다시 계획
        [2026-10-16 사용자 결정 (가)(나)] 경로 앞 (0.2~2 s) 이 지도와 닿으면 3 s 를 기다리지 않고 다시 계획, 다시 계획은 멈추지 않고
        (로봇은 지금 경로로 계속, 계획은 다른 스레드 → 끝나면 새 경로로) — rt_config.PlanCfg (가)(나)
기록: <out>/rt_run.json (주기·영상·사건·다리·새 물체 칸) → python3 -m tools.rt_report. 설정 rt_config.py, 물체 rt_scenarios.py
      <out>/u_exec.npz (실행된 성형 전 명령 u, 1 ms, 세그먼트별) → 보고서 '성형 확인' (WMX 명령 − 전체 컨볼루션)
"""
import argparse
import collections
import dataclasses
import gc
import json
import os
import sys
import time
from pathlib import Path

PTXAS = '/usr/local/cuda/bin/ptxas'          # cuRoboV2 공식 실행 설정 (Mirobot 노드와 같음: torch.compile + Thor ptxas)


def configure_curobo_runtime():
    """cuRoboV2 실행 설정 (torch.compile·torch.jit 켬 + Thor ptxas) — 공개 API 가 없어 비공개 curobo._src.runtime 의 깃발을 씀.
    환경 변수 CUROBO_COMPILE=0 이면 안 켬. cuRobo 의 다른 모듈을 불러오기 전에 불러야 함 → 이 모듈을 불러올 때 한 번 (아래, 예전과 같은 차례).
    [2026-10-27 R-1] 함수로 묶음 (cuRobo 판이 바뀌면 여기 한곳만)."""
    if os.environ.get('CUROBO_COMPILE', '1') != '0' and os.path.exists(PTXAS):
        os.environ.setdefault('TRITON_PTXAS_PATH', PTXAS)
        import curobo._src.runtime as _rt
        _rt.torch_compile = True
        _rt.torch_jit = True


configure_curobo_runtime()

import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import depth_sim as ds  # noqa: E402
from rt_config import Config, TaskCfg  # noqa: E402
from rt_motion import brake_fine, clamp_final, track_target  # noqa: E402
from rt_robot import IdealRobot, RosRobot, SimClock, WallClock  # noqa: E402
from rt_scenarios import SCENARIOS, get_scenario, obstacle_pose  # noqa: E402
from rt_shaping import load_cfg, make_kernel  # noqa: E402
from rt_stream import Stream  # noqa: E402
from rt_world import FAR_M, World, robot_cfg  # noqa: E402
import rt_model  # noqa: E402

NEWVOX_EVERY = 3            # 새 물체 칸은 영상 3장마다 기록 (보고서 3D 재생용, 칸 번호를 묶은 글자 rt_world.pack_voxels)
RESTART_FRAME_ROUNDS = 3    # (다시) 시작 때 밀린 영상 쌓기를 되풀이하는 최대 횟수 (영상 한 장이 영상 간격보다 오래 걸려도 끝나게)
CP_U_STEP = 5               # 성형 여유 보존 u 표본 간격 (1 ms 칸 → 5 ms, 메시지 점 간격과 같음)
MOVING_RAD_S = 1e-3         # 성형 여유 보존: 움직이는 중 = 관절 속도 최대가 이만큼 (rad/s)
SAME_POSE_RAD = 1e-3        # 성형 여유 보존: y 점과 비교 u 가 다른 자세 = 관절 차 최대가 이보다 큼 (rad) — 그때만 cp_margin
GOAL_CLOSER_M = 0.01        # 목표 자리 막힘 = 목표 자세의 새 물체 지도 거리가 다리 시작 때보다 이만큼 넘게 가까워짐 (leg 설명)
PLAN_MATCH_Q, PLAN_MATCH_V = 1e-6, 1e-4   # [cmpc] 실행된 상태가 계획과 같음 = 관절 (rad)·속도 (rad/s) 차 최대가 이 안 — 다르면 다시 계획


# [2026-10-27 R-1] 지운 되돌리기 설정 (rt_config 에서 뺌) 의 고정값 — 기록 settings.config 에 넣어 둠: 기록을 읽는 도구가 이 이름으로 실행을
#  묶고 머리말을 씀 (rt_report back_style·성형 규칙 cp_*, 보고서/runs_20261016/rt_index.py cp_margin_approach, rt_tools 묶음 이름 live_map·keep_if_clear·
#  back_replan·plan_seeds·cp_margin_approach). 값 = 지금 동작
REC_FIXED_CFG = {'plan': {'live_map': 'start', 'keep_if_clear': True, 'back_replan': False, 'plan_seeds': True, 'back_style': 'yield'},
                 'guard': {'cp_margin_approach': True, 'cp_dyn_approach': True}}


def log(*x):
    print(time.strftime('[%H:%M:%S]'), *x, flush=True)


def rec_config(c: Config) -> dict:
    """기록 settings.config = 설정 전체 + 지운 되돌리기 설정의 고정값 (REC_FIXED_CFG)."""
    d = dataclasses.asdict(c)
    for sec, kv in REC_FIXED_CFG.items():
        d[sec].update(kv)
    return d


class Controller:
    def __init__(self, c: Config, world: World, mpc, robot, clock, scenario: str, kernel, record_sent=False, shaping_label='none',
                 log=None):
        """log = 사건·저장 로그 함수 (None = 이 모듈의 log — rt_run.log 를 바꿔 끼워도 그대로 따름)."""
        self.c, self.W, self.mpc, self.robot, self.clock = c, world, mpc, robot, clock
        self.log_fn = log
        self.ideal = isinstance(robot, IdealRobot)
        self.style = c.style
        self.dt = 1.0 / c.mpc.rate_hz                       # 제어 주기 = 메시지 주기 (20 Hz). 성형 MPC 는 c.cmpc.rate_hz 로 다시 계획
        self.jmax = np.full(len(mpc.names), float(c.motion.jerk))
        self.obstacles = get_scenario(scenario)
        self.record_sent = record_sent
        self.q0 = robot.measured()
        self.kernel = kernel
        self.cp = c.guard.cp_thr > 0 and kernel.L > 1     # 성형 여유 보존 (rt_config.GuardCfg.cp_thr)
        self.g = kernel.grid_s                              # 입력성형 격자 = 명령 궤적 적분 단계 = 1 ms
        # 성형 MPC 는 상태 궤적 점 시각 (−Hh·opt_dt …) 의 실행된 u 가 필요 → 그만큼 기억 (CMpc.history)
        self.mem_n = int(np.ceil(mpc.Hh * mpc.opt_dt / self.g)) + 2 if self.style == 'cmpc' and mpc.shaped else 0
        self.stream = Stream(c.stream, self.dt, self.q0, h=kernel.h, grid_s=self.g, mem=self.mem_n)
        self.n_dt = self.stream.n_dt                       # 한 주기 칸 수 (50)
        self.n_main = int(round(c.motion.cycles * self.dt / self.g))     # 메시지가 덮는 명령 궤적 칸 수 (2 주기 = 100)
        self.step_n = int(round(c.motion.step_s / self.g))               # 보내는 점 간격 (5 ms)
        self.tail_n = int(round(c.motion.tail_dt / self.g))              # 꼬리 점 간격 (10 ms)
        # 성형 MPC: 계획 한 번 = 매듭 2 개 = MPC 주기 (5 Hz → 200 ms) 를 실행하고 다시 계획. 그 사이 20 Hz 틱은 같은 계획을 이어 읽음
        self.n_commit = int(round(1.0 / c.cmpc.rate_hz / self.g)) if self.style == 'cmpc' else 0
        self.plan_k = self.plan_T0 = None
        self.plan_info = None
        self.cam_next = clock.now()
        self.t_go = None
        self.label = '켜기'
        self.last_target = np.array(self.q0, float)     # 직전 주기에 쫓은 목표점 (MPC 발산 때 대신 씀)
        self.holding = False                # MPC 발산이라 직전 목표를 유지하는 중
        self._last_msg = None               # 마지막으로 보낸 메시지 점 (절대 시각 [n], [n, q·v·a 18]) — 실행된 명령 기록용
        self._sent_log = []                 # 실행된 명령 점 [m, 19] 묶음 (보고서 '보낸 명령' 선)
        # (가)(나) 멈추지 않는 다시 계획 (rt_config.PlanCfg): 진행 중인 계획 하나 {'t_req', 'why', 'label', 'ready'·'path'·'wall' (이상적)
        #  | 'fut' (실시간: 계획기 스레드)}, 마지막 요청 시각 (replan_cooldown_s), 계획기 스레드·CUDA 스트림 (실시간에서 처음 쓸 때 만듦)
        self._live = None
        self._live_last = -1e9
        self._pool = self._pstream = None
        self.n_live = {'req': 0, 'ok': 0, 'fail': 0, 'drop': 0, 'keep': 0}     # keep = 새 경로가 왔지만 지금 경로가 이제 안 막혀 그대로 (①)
        self._ahead_since = None
        # (가') 비키기 (rt_config.PlanCfg.back_off): 지금 y 자리의 MPC 지도 거리 (주기마다), 그 기록 [(시각, 거리)], 비키는 중 상태,
        #  횟수 {시작, 새 경로로 끝, 다시 앞으로}
        self._clr_mpc = None
        self._dhist = collections.deque()
        self._dsm = collections.deque()      # 고른 거리 (back_smooth_s 최소) 기록 — '다가옴' 의 최대값
        self._back = None
        self.n_back = {'start': 0, 'new_path': 0, 'resume': 0}
        t = c.task
        self.rec = {'settings': {
            'robot': 'ideal' if self.ideal else 'ros', 'scenario': scenario, 'legs': t.legs,
            'rate_hz': 1.0 / self.dt, 'lead_s': c.stream.lead_s, 'jerk': c.motion.jerk,
            'act_new': c.dist.act_new, 'act_static': c.dist.act_static, 'mask': c.dist.mask,
            'grid_center': list(c.map.center), 'grid_size': list(c.map.size), 'voxel': c.map.voxel, 'cam_fps': c.map.cam_fps,
            'grid': world.grid_info(),
            'cam_pose': ds.pose7(world.cam.cam_T[0]), 'cam_K': ds.intrinsics(),
            # [2026-10-27 R-1] map_uniform·live_replan·fill_grad·cp_dyn_approach = 지운 되돌리기 설정의 고정값 (기록을 읽는 rt_report·rt_tools
            #  가 실행을 묶고 머리말을 씀 — REC_FIXED_CFG)
            'model': c.model, 'map_uniform': True, 'live_replan': True, 'ahead_check': c.plan.ahead_check,
            'back_off': c.plan.back_off and self.style == 'carrot', 'fill_grad': True, 'cp_dyn_approach': True,
            'qA': list(t.goals(c.model)[0]), 'qB': list(t.goals(c.model)[1]),
            'pillar': [t.pillar[0], list(t.pillar[1]), list(t.pillar[2])],
            'obstacles': [{'name': n, 'dims': d, 'wps': w} for n, d, w in self.obstacles],
            'config': rec_config(c), 'style': self.style, 'map': 'oracle' if getattr(world, 'oracle', False) else 'camera',
            'shaping': {'label': shaping_label, **kernel.info()},
            'cmpc': None if self.style != 'cmpc' else {'opt_dt': mpc.opt_dt, 'n_knots': mpc.n_knots, 'H': mpc.H, 'Hh': mpc.Hh}},
            'cycles': [], 'frames': [], 'events': [], 'newvox': [], 'legs': []}
        # [2026-10-30 PVAT 단계 1] 보낸 메시지 전체 (시각·P·V·A) + 보낼 때 걸린 시간 — CR3A_PVAT_RECORD=1 (rt_run --pvat-record) 일 때만.
        #  기본 끔: 끄면 예전과 똑같음 (GUI 서버도 이 Controller 를 물려받음). 저장 = <결과 폴더>/pvat_msgs.npz (save_pvat_msgs)
        self.pvat = None
        if os.environ.get('CR3A_PVAT_RECORD') == '1':
            self._pvat_wrap_send()

    def _pvat_wrap_send(self):
        """robot.send 를 감싸 실제로 보낸 메시지마다 기록 (주기 메시지·GUI 비상정지 메시지 모두 이 길로 감). 시간 (perf_counter, µs):
          pack  = 제안 방식 (PVAT) 상위 몫 — 스탬프 S 부터의 점을 엔진 시각 정수 µs + P·V·A 배열로 (API 호출 직전까지, 사용자 정의
                  10-29). 실제로 PVAT 로 보내지는 않고 같은 주기 안에서 시간만 잼
          build = 지금 방식: JointTrajectory 만들기 (RosRobot.build_msg),  pub = 발행 (DDS 직렬화·보내기)
        기록 자체 (점 배열 복사) 는 시간 잰 뒤에 함. ROS 가 아닌 로봇 (이상적) 은 원래 send 를 부르고 build·pub = 0."""
        robot = self.robot
        orig = robot.send
        ros = hasattr(robot, 'build_msg') and hasattr(robot, 'pub')
        rec = self.pvat = {k: [] for k in ('wall', 'S', 't0', 'n_pre', 'pts', 'pack_us', 'build_us', 'pub_us')}

        def send(t0, points, q_next):
            S = self.stream.stamp
            c0 = time.perf_counter()
            keep = [p for p in points if t0 + p[0] >= S - 1e-6]
            T = np.array([int(round((t0 + p[0]) * 1e6)) for p in keep], np.int64)
            P, V, A = (np.array([p[i] for p in keep], np.float64) for i in (1, 2, 3))
            c1 = time.perf_counter()
            if ros:
                m = robot.build_msg(t0, points)
                c2 = time.perf_counter()
                robot.pub.publish(m)
                c3 = time.perf_counter()
            else:
                orig(t0, points, q_next)
                c2 = c3 = time.perf_counter()
            del T, P, V, A
            rec['wall'].append(time.time())
            rec['S'].append(S)
            rec['t0'].append(t0)
            rec['n_pre'].append(len(points) - len(keep))
            rec['pts'].append(np.array([[p[0], *p[1], *p[2], *p[3]] for p in points], np.float64))
            rec['pack_us'].append((c1 - c0) * 1e6)
            rec['build_us'].append((c2 - c1) * 1e6)
            rec['pub_us'].append((c3 - c2) * 1e6)

        robot.send = send

    # ---------------- 기록·장면 ----------------
    def event(self, text, **kw):
        self.rec['events'].append({'t': self.clock.now(), 'text': text, **kw})
        self._log(text)

    def _log(self, *x):
        """[2026-10-27 R-1] 로그 한 줄: log_fn (만들 때 log=) 이 있으면 그것, 없으면 이 모듈의 log (부를 때 찾으므로 GUI 서버의 rt_run.log
        바꿔 끼우기도 그대로 됨)."""
        (self.log_fn or log)(*x)

    def _boxes(self, t):
        """시각 t 의 상자들 [(중심, 크기, wxyz)]: 기둥 + (GO 뒤) 움직이는 물체."""
        p = self.c.task.pillar
        out = [(list(p[1]), list(p[2]), [1, 0, 0, 0])]
        if self.t_go is not None:
            out += [(obstacle_pose(w, t - self.t_go), d, [1, 0, 0, 0]) for _, d, w in self.obstacles]
        return out

    def _frames_until(self, t_end):
        """카메라 (cam_fps): t_end 전까지 차례가 온 영상을 렌더해 지도에 쌓는다 (MPC 주기와 따로). 영상마다 기록."""
        while self.cam_next <= t_end:
            tc = self.cam_next
            self.clock.sleep_until(tc)
            t = self.clock.now()
            qm = self.robot.measured()
            _, map_ms = self.W.frame(qm, self._boxes(t))
            nr = self.W.last_near                       # 이 영상 관절값의 진단 거리 (frame 이 같이 구함)
            fr = self.rec['frames']
            fr.append({'t': round(t, 4), 'q': np.round(qm, 6).tolist(), 'map_ms': round(map_ms, 1), 'fill': self.W.cm.boundary_fill,
                       'near_mm': nr.get('near_mm'), 'near_new_mm': nr.get('near_new_mm')})
            if len(fr) % NEWVOX_EVERY == 0 and self.W.cm.bg_ready:
                nv = {'t': round(t, 3), 'v': self.W.voxel_code('dyn')}
                if self.W.voxel_cut:                # [2026-10-27 R-1] maxn 을 넘어 버린 칸 수 (기록이 잘렸음을 알게)
                    nv['cut'] = self.W.voxel_cut
                self.rec['newvox'].append(nv)
            self.cam_next = tc + 1.0 / self.c.map.cam_fps
            if not self.ideal and time.time() - self.cam_next > self.c.map.cam_max_lag_s:
                self.cam_next = time.time()

    def _log_sent(self, S, pts):
        """실행된 명령 기록: 앞 메시지는 이 메시지의 스탬프 S 까지 실행됐으므로 (JTC 가 S 에서 갈아탐, 앞 점은 같은 궤적) 그 앞 점들을 적고
        이 메시지를 기억. pts = [(S 기준 시각, q, v, a)] (명령 궤적 + 멈춤 꼬리)."""
        self._flush_sent(S)
        # 세그먼트 번호 (다시 시작 사이 구간) 도 기록: 이상적 모드는 다시 계획할 때 시계를 ideal_plan_s 만 건너뛰어 앞 메시지 꼬리와 다음
        #  세그먼트의 시각이 겹칠 수 있음 → 성형 확인 (rt_tools shaping) 이 세그먼트별로 비교
        self._last_msg = (S + np.array([p[0] for p in pts]), np.array([np.concatenate(p[1:]) for p in pts]), len(self.stream.segments))

    def _flush_sent(self, until=np.inf):
        """마지막 메시지에서 until 전 점들을 기록 (until 없음 = 끝까지: 다음 메시지 없이 꼬리로 멈출 때)."""
        if self._last_msg is None:
            return
        t, X, seg = self._last_msg
        m = t < until - 1e-9
        if m.any():
            self._sent_log.append(np.column_stack([t[m], X[m], np.full(int(m.sum()), seg)]))
        self._last_msg = None

    # ---------------- 한 주기 ----------------
    def _u_tail(self, x):
        """상태 x = (q, v, a) 에서 저크 한계로 멈추는 1 ms 칸 [n, 3, J] (마지막 = 멈춤). 2026-10-14 전 braking_tail 과 같은 계산.
        [2026-10-27] 한계 = 추종기 한계 (_track_lim — 여기서는 MPC 한계 그대로, GUI 서버는 실시간 회피 속도 배율)."""
        _, amax, jmax = self._track_lim()
        F, _ = brake_fine(*x, amax, jmax)
        return F

    def _u_brake(self, s0):
        """지금 (s0) 부터 저크 한계로 멈추는 후보 u [1 + n_main + 꼬리, 3, J] (2026-10-14 전 brake + braking_tail 과 같은 값).
        [2026-10-15] 꼬리 = 같은 정지 궤적의 뒷부분 (정지 법칙은 지금 상태로만 정해져 main[-1] 에서 다시 계산해도 같은 값) — 정지 궤적을
        두 번 계산하던 것을 한 번으로 (실시간 마감, 안전 검사 주기 8.5 ms 중 절반)."""
        F, _ = brake_fine(*s0, self.mpc.amax, self.jmax)
        n = self.n_main
        if len(F) > n:
            return np.concatenate([np.stack(s0)[None], F])
        main = np.concatenate([F, np.repeat(F[-1:], n - len(F), axis=0)])
        return np.concatenate([np.stack(s0)[None], main, self._u_tail(main[-1])])

    def _shape_points(self, u):
        """후보 u → (성형된 y 칸, 메시지 점 [(스탬프 기준 s, q, v, a)])."""
        y = self.stream.shape(u)
        return y, self.stream.points(y, self.step_n, self.n_main, self.tail_n)

    def _track_lim(self):
        """한계 안 추종기 (당근을 쫓아 로봇 명령을 만듦) 의 속도·가속·저크 한계. 여기서는 MPC 한계 그대로 — GUI 서버가 실시간 회피 속도
        배율로 바꿈 (rt_server.GuiController._track_lim, 2026-10-27)."""
        return self.mpc.vmax, self.mpc.amax, self.jmax

    def _command_carrot(self, s0):
        """[2026-10-14 전 방식] 당근 MPC + 한계 안 추종기 → 후보 u [1 + n_main + 꼬리, 3, J] 와 기록할 값."""
        c = self.c
        info = self.mpc.step()
        target, clamp, final = info['target'], False, False
        held = bool(info['ok'] and not info['feasible'] and np.max(np.abs(target - s0[0])) > c.mpc.diverge_rad)
        if not info['ok']:
            target = s0[0]                              # MPC 해가 수가 아님 (NaN·inf): 이번 주기는 지금 명령 자리를 지킴
            self.event('MPC 해가 수가 아님 → 지금 자리 지킴')
        elif held:
            # MPC 발산: 해를 못 찾으면서 지금 명령에서 diverge_rad 넘게 떨어진 자세를 냄 → 따라가지 않고 직전 목표 유지 (rt_config.MpcCfg).
            #  MPC 는 다시 맞추지 않음 (다시 맞춤이 잦을수록 발산이 늘었다: 2026-10-14 WMX 90 회 → 해 없음 6 배, 1 rad 넘는 당근 0 → 73)
            target = self.last_target
        elif info['final']:
            # 마지막 접근 (경로를 따라가는 중 MPC 목표 = 경로 끝): 지금 명령 자세와 당근 둘 다 새 물체 반응 거리·정지 물체 반응 거리
            #  밖이면 (= MPC 가 피하는 중이 아님) 관절마다 목표를 넘지 않게 (rt_motion.clamp_final). [2026-10-13 사용자 지적: 도착 때
            #  목표를 살짝 지나쳤다 돌아옴] MPC 당근이 목표를 1.6° 까지 지나쳤다 돌아와서 로봇도 따라감 (이상적 모드 실측)
            cl2 = self.W.clearances(np.vstack([s0[0], target]))
            final = bool(cl2['raw'].min() >= c.dist.act_static and cl2['dyn'].min() >= c.dist.act_new)
            if final:
                target, clamp = clamp_final(target, s0[0], self.mpc.goal)
        if held != self.holding:
            self.event('MPC 발산 → 직전 목표 유지' if held else 'MPC 정상 → 다시 따라감')
            self.holding = held
        self.last_target = np.array(target, float)
        m = c.motion
        n = int(round(m.cycles * self.dt / m.step_s))
        vmax, amax, jmax = self._track_lim()
        Q, V, A, F = track_target(*s0, target, n, m.step_s, vmax, amax, jmax, self.dt, sub=m.sub,
                                  stop_decel=m.final_stop_decel if final else 1.0, qlim=self.mpc.qlim, fine=True)
        u = np.concatenate([np.stack(s0)[None], F, self._u_tail(F[-1])])
        k = int(round(self.dt / m.step_s)) - 1
        rec = {'carrot': info['target'], 'final': final, 'clamp': clamp, 'held': held,
               'carrot_out': bool(info['ok'] and np.any((info['target'] < self.mpc.qlim[0]) | (info['target'] > self.mpc.qlim[1]))),
               'lim_dev': round(float(np.max(np.abs(Q[k] - info['target']))), 5), 'raw_acc_ratio': info['raw_acc_ratio']}
        return u, info, rec

    def _command_cmpc(self, s0):
        """[2026-10-14 C 방식] 성형된 경로를 최적화하는 시간표 MPC → 후보 u (계획의 이번 스탬프부터 2 주기 + 멈춤 꼬리) 와 기록할 값.
        MPC 시작 상태 = 실행된 u (s0), 기억 = 실행된 u (rt_stream.past_q) — cuRobo 안의 y 와 보낼 y 가 같은 기억·같은 성형기.
        다시 계획: 계획을 MPC 주기 (매듭 2 개) 만큼 실행했거나, 실행된 상태가 계획과 다를 때 (안전 검사 개입·앞 메시지 계속·다시 시작).
        [2026-10-14 실측] 20 Hz (점 간격 6.25 ms) 시간표 MPC 는 cuRobo 최적화가 방향을 못 바꿈 → MPC 는 5 Hz (점 간격 25 ms) 로 풀고,
        메시지·안전 검사·지도는 20 Hz 그대로."""
        st = self.stream
        need = self.mpc.plan is None or self.plan_k is None or st.T0 != self.plan_T0 or st.k - self.plan_k >= self.n_commit
        if not need:
            x = self.mpc.sample(st.k - self.plan_k, 0)[0]
            need = bool(np.max(np.abs(x[0] - s0[0])) > PLAN_MATCH_Q or np.max(np.abs(x[1] - s0[1])) > PLAN_MATCH_V)
        if need:
            hist = self.mpc.history(st.past_q(self.mem_n), self.g) if self.mpc.shaped else None
            elapsed = None if self.plan_k is None or st.T0 != self.plan_T0 else (st.k - self.plan_k) * self.g
            info = self.mpc.step(s0, hist, elapsed=elapsed)
            self.plan_k, self.plan_T0, self.plan_info = st.k, st.T0, info
            info = dict(info, replan=True)
        else:
            info = dict(self.plan_info, ms=0.0, replan=False, stuck=False)   # 막힘 판정은 다시 계획한 틱에서만 (한 번만)
        rec = {'carrot': None, 'final': info.get('final', False), 'clamp': False, 'held': False, 'carrot_out': False,
               'lim_dev': None, 'raw_acc_ratio': info.get('acc_ratio'), 'vel_ratio': info.get('vel_ratio'),
               'start_err': info.get('start_err'), 'replan': info['replan']}
        if not info['ok'] or self.mpc.plan is None:
            if info['replan']:
                self.event('MPC 해가 수가 아님 → 감속 정지')
            u = self._u_brake(s0)
        else:
            u1 = self.mpc.sample(st.k - self.plan_k, self.n_main)
            u1[0] = np.stack(s0)                         # 시작 = 실행된 상태 그대로 (MPC 첫 점과 float32 반올림만 다름, start_err)
            u = np.concatenate([u1, self._u_tail(u1[-1])])
            rec['carrot'] = u[self.n_dt, 0]
        info['target'] = u[self.n_dt, 0]
        return u, info, rec

    def _cp_u(self, u):
        """성형 여유 보존용 u 표본: 스탬프 앞 성형기 길이 (실행된 u) + 후보 u + 뒤에 멈춘 자세, 5 ms 마다. 거리 계산은 부르는 쪽이 y 점과
        한 번에 (rt_world.clearances 한 번 = 기구학·GPU 기다림 한 번). [2026-10-15 WMX] 따로 부르면 주기마다 ~6 ms (안전 검사 주기 ~8 ms)
        늘어 실시간 마감을 자주 넘겼다 (안 보낸 메시지 28 → 532)."""
        L, st = self.kernel.L, self.stream
        u = np.asarray(u)
        U = np.concatenate([st.past()[:, 0], u[:, 0], np.repeat(u[-1:, 0], L - 1, axis=0)])
        Uv = np.concatenate([np.zeros((L - 1, U.shape[1])), u[:, 1], np.zeros((L - 1, U.shape[1]))])   # u 속도 (앞뒤 = 서 있음 취급)
        idx = np.arange(0, len(U), CP_U_STEP)
        mv = np.max(np.abs(Uv), axis=1) >= MOVING_RAD_S
        last_mv = np.maximum.accumulate(np.where(mv, np.arange(len(Uv)), -1))
        since = np.where(last_mv >= 0, np.arange(len(Uv)) - last_mv, 0)          # u 가 선 뒤 지난 시간 (ms, 칸마다)
        return {'Uq': U[idx], 'tu': idx - (L - 1), 'mv': mv, 'since': since, 'L': L}

    def _clearances_with_u(self, pts, uu):
        """메시지 점 pts 와 (있으면) 성형 여유 보존 u 표본의 거리를 한 번에 → (pts 거리, u 표본 거리).
        비키기 (PlanCfg.back_off) 가 켜져 있으면 MPC 지도 거리도 (같은 기구학 한 번, 판단에는 pts[0] = 지금 y 자리만 씀)."""
        Qp = np.array([p[1] for p in pts])
        grids = ('raw', 'dyn', 'mpc') if self.c.plan.back_off else ('raw', 'dyn')
        if uu is None:
            return self.W.clearances(Qp, grids), None
        both = self.W.clearances(np.vstack([Qp, uu['Uq']]), grids)
        n = len(Qp)
        return {k: v[:n] for k, v in both.items()}, {k: v[n:] for k, v in both.items()}

    def _cp_slack(self, uu, cu, pts, c_y):
        """[2026-10-15] 성형 여유 보존 (rt_config.GuardCfg.cp_thr, 작업기록 5-14·5-15): 성형된 y 의 앞으로 점마다 실제 거리 지도 거리 ≥
        min(그 앞 성형기 길이 동안 성형 전 u 가 지난 자리들의 최소 거리, cp_thr) − cp_eps 인가. y(t) 는 u 의 [t − 성형기 길이, t] 평균이라
        굽은 곳에서 u 보다 장애물에 가까워질 수 있음 (5-12: u 는 +0.1~+4 mm 로 지났는데 y 만 −0.0~−1.2 mm 닿음 10 번) → 그런 후보를 막음.
        uu = _cp_u(후보 u), cu = 그 표본 거리, pts = 후보 y 의 메시지 점, c_y = pts 의 {'raw', 'dyn'} 거리.
        움직이는 물체 (cp_vobj > 0): 새 물체 거리는 지나는 시각의 예측 거리 (거리 − cp_vobj × 지금부터 그 시각) 로 비교 (서 있으면 y = u 라 같음).
        반환 (여유 — 0 이상 = 통과, 가장 나쁜 조건 1·2·3 — 0 = 걸린 점 없음). 같은 값이면 앞 점, 한 점에서는 (1)(2)(3) 차례로 먼저인 것.
        [2026-10-27 R-1] 점마다 돌던 계산을 배열로 (창 = sliding_window_view + 창 밖 inf) — 여유·조건이 예전 반복과 같음
        (tests/test_cp_slack.py 가 예전 반복과 비교)."""
        g = self.c.guard
        L, tu, Uq, mv, since = uu['L'], uu['tu'], uu['Uq'], uu['mv'], uu['since']
        lead_ms = self.c.stream.lead_s * 1000.0             # 지도 = 계산 시작 (스탬프 − lead) 때 것

        def infl(t_ms):                                     # 그 시각까지 물체가 다가올 수 있는 거리 (m)
            return g.cp_vobj * np.maximum(0.0, (np.asarray(t_ms, float) + lead_ms) / 1000.0)
        cu_raw, cu_dyn = cu['raw'], cu['dyn'] - infl(tu)
        tms = np.array([int(round(p[0] * 1000)) for p in pts])
        los, his = np.searchsorted(tu, tms - (L - 1), 'left'), np.searchsorted(tu, tms, 'right')
        y_ds = c_y['dyn'] - infl(tms)
        y_now = float(c_y['raw'][0])                       # 지금 y 자리의 실제 거리 지도 거리 (여유는 더 다가갈 때만)
        y_now_d = float(c_y['dyn'][0])                     # 지금 y 자리의 물체 지도 거리 ((2)(3) 은 더 다가갈 때만)
        if len(pts) < 2:
            return np.inf + g.cp_eps, 0
        # 앞으로 점 k = 1.. 마다 창 = u 표본 [lo, hi) (y(t) 를 만든 u 의 [t − 성형기 길이, t]). 창이 비면 그 점은 건너뜀
        lo, hi, tm = los[1:], his[1:], tms[1:]
        n_w = hi - lo
        W = max(int(n_w.max()), 1)
        inwin = np.arange(W)[None, :] < n_w[:, None]

        def window(x):                                      # x[lo:hi] 를 [점, W] 로 (창 밖 inf)
            xp = np.concatenate([x, np.full(W, np.inf, x.dtype)])
            return np.where(inwin, np.lib.stride_tricks.sliding_window_view(xp, W)[lo], np.inf)
        S = np.full((len(lo), 3), np.inf)                   # 점마다 조건 (1)(2)(3) 의 여유 (안 걸리면 inf)
        # (1) 지금 지도 (정지·새 물체 모두): y 가 창 안 u 의 가장 가까운 자리보다 가까우면 안 됨 (cp_thr 안에서). u 가 cp_margin_zone 안까지
        #  붙었고 자세가 다르면 cp_margin 더 (카메라 지도 오차가 자리마다 다름)
        j = np.minimum(lo + np.argmin(window(cu_raw), axis=1), len(cu_raw) - 1)
        ref = cu_raw[j].astype(float)
        yr = c_y['raw'][1:].astype(float)
        # [2026-10-16] 여유는 y 가 지금보다 더 다가가는 점에서만 (기둥을 따라 미끄러지며 막히는 것 고침, rt_config GuardCfg.cp_margin)
        closer = yr < y_now - g.cp_eps
        moved = np.max(np.abs(np.array([p[1] for p in pts[1:]]) - Uq[j]), axis=1) > SAME_POSE_RAD
        S[:, 0] = yr - np.where(ref >= g.cp_thr, g.cp_thr, ref + np.where(closer & (ref < g.cp_margin_zone) & moved, g.cp_margin, 0.0))
        if g.cp_vobj > 0:
            # [2026-10-16] (2)(3) 은 이 점이 지금 y 자리보다 물체 지도에서 더 가까울 때만 (물러나는 점은 안 막음, rt_config GuardCfg.cp_vobj)
            yd = c_y['dyn'][1:].astype(float)
            dyn_on = ~(yd >= y_now_d - g.cp_eps)
            # (2) 늦게 지나감: u 가 (물체가 최고 속도로 다가와도) 안 닿는데 y 는 닿게 되면 안 됨 — 서 있으면 y = u 라 같은 값
            ref_d = window(cu_dyn).min(axis=1)
            S[:, 1] = np.where(dyn_on & (ref_d >= 0.0), y_ds[1:] - np.minimum(ref_d, g.cp_margin), np.inf)
            # (3) 늦게 섬: u 는 섰는데 y 만 아직 움직이는 동안 (성형 때문에 생긴 움직임) 에는 u 가 선 뒤 지난 시간만큼 물체가 다가와도
            #  안 닿아야 함. [고침 2026-10-15] '지금부터' 다가오는 몫 (0.5 s 뒤 점이면 7.5 cm) 까지 요구했더니 안전 검사 개입이 성형 없음의
            #  8 배 — 그 몫은 성형 없음 로봇도 u 가 설 때까지 똑같이 지는 기존 위험이라, 성형으로 늘어난 '선 뒤 더 움직이는 시간' 만 요구
            ku = tm + (L - 1)                               # u 칸 번호
            kc = np.clip(ku, 0, len(mv) - 1)
            y_moving = np.max(np.abs(np.array([p[2] for p in pts[1:]])), axis=1) > MOVING_RAD_S
            on3 = dyn_on & (ku >= 0) & (ku < len(mv)) & ~mv[kc] & y_moving
            S[:, 2] = np.where(on3, yd - g.cp_vobj * since[kc] / 1000.0, np.inf)
        S[n_w <= 0] = np.inf
        S[np.isnan(S)] = np.inf                             # (예전 반복의 '<' 는 NaN 을 고르지 않음)
        i = int(np.argmin(S))                               # 가장 나쁜 것 — 같으면 앞 점·앞 조건 (예전 반복의 '<' 와 같음)
        worst = float(S.flat[i])
        return worst + g.cp_eps, (0 if worst == np.inf else i % 3 + 1)

    def _guard(self, s0, slack, d_now, dd_now):
        """마지막 안전 검사 (보낼 궤적+꼬리의 여유가 0 아래일 때 부름: 실제 거리 지도에서 닿거나, 새 물체 6 cm 안으로 들어감).
        V2 MPC 는 충돌을 비용으로만 다뤄 지도상 충돌 구간(−56 mm)을 낸 적이 있다. 대안: (가) 지금부터 저크 한계 정지 (나) 앞 메시지
        (지난 주기에 검사됨) 계속 — 여유가 큰 쪽, 둘 다 지금 궤적보다 나쁘면 그대로. 반환 (None | 'brake' | 'keep', (u, y, 점), 실제 거리).
        [2026-10-14] 검사는 성형된 y (로봇이 실제로 가는 명령) 로. 감속 정지도 성형기를 거친 y (멈추는 데 성형 길이만큼 더 걸림)."""
        ub = self._u_brake(s0)
        yb, pb = self._shape_points(ub)
        uub = self._cp_u(ub) if self.cp else None
        old = self.stream.remaining()
        # 브레이크 점 + (성형 여유 보존 u 표본) + 앞 메시지 점의 거리를 한 번에 (실시간 마감, _cp_u 설명)
        parts = [np.array([p[1] for p in pb])] + ([uub['Uq']] if uub is not None else []) + ([np.array([p[1] for p in old])] if old else [])
        allc = self.W.clearances(np.vstack(parts), ('raw', 'dyn'))
        cuts = np.cumsum([0] + [len(x) for x in parts])
        sub = [{k: v[cuts[i]:cuts[i + 1]] for k, v in allc.items()} for i in range(len(parts))]
        clb = sub[0]
        cub = sub[1] if uub is not None else None
        clo = sub[-1] if old else None
        s_brk, d_brk = self._slack_of(clb, 1, d_now, dd_now)
        if self.cp:                                     # 브레이크 후보도 성형 여유 보존 (앞 메시지 계속은 보낼 때 이미 검사됨)
            s_brk = min(s_brk, self._cp_slack(uub, cub, pb, clb)[0])
        s_old, d_old = self._slack_of(clo, 0, d_now, dd_now) if old else (-np.inf, -np.inf)
        if max(s_brk, s_old) <= slack:
            return None, None, None
        if s_old > s_brk:
            return 'keep', None, d_old
        return 'brake', (ub, yb, pb), d_brk

    def cycle(self) -> dict:
        """한 주기 (모듈 머리말 1~5): 스탬프·영상 (_cycle_stamp) → 성형 전 명령 u (_command_carrot | _command_cmpc) → 성형된 y·안전 검사
        (_cycle_check) → 보내기 (_cycle_send) → 기록 (_cycle_record — 안전 검사가 개입했으면 MPC 다시 맞춤). MPC 정보 (info) 를 돌려줌.
        [2026-10-27 R-1] 한 함수이던 것을 나눔 (계산·차례 그대로)."""
        S, late = self._cycle_stamp()
        t = self.clock.now()
        qm = self.robot.measured()
        s0 = tuple(np.array(x, float) for x in self.stream.cmd)      # 스탬프 시각의 성형 전 명령 상태 (실행된 u)
        t_cyc = time.perf_counter()
        if self.style == 'cmpc':
            u, info, rec = self._command_cmpc(s0)
        else:
            u, info, rec = self._command_carrot(s0)
        u, y, pts, chk = self._cycle_check(s0, u)
        snd = self._cycle_send(S, u, y, pts, chk['keep'])
        cyc_ms = round((time.perf_counter() - t_cyc) * 1000, 1)        # MPC 부터 보내기까지 (영상 처리 뺌)
        self._cycle_record(t, S, qm, u, pts, info, rec, chk, snd, late, cyc_ms)
        return info

    def _cycle_stamp(self):
        """이번 주기 스탬프 S 를 정하고 그 전 영상을 지도에 쌓은 뒤 계산 시작 (S − lead) 까지 기다림 → (S, 늦어서 미룬 시간 s)."""
        st = self.stream
        lead = self.c.stream.lead_s
        if st.stamp is None:
            # (다시) 시작: 로봇은 서 있으므로 밀린 영상을 다 쌓은 뒤에 시간표를 정함. [2026-10-13 WMX rt_20261013_230834] 움직이다
            #  다시 계획하면 꼬리·계획을 기다리는 동안 영상이 5장 밀려, 그걸 쌓느라 첫 메시지가 스탬프보다 81 ms 늦게 갔고 JTC 가 정지에서
            #  궤적 중간 (관절 1 0.47 rad/s) 으로 뛰어 CyclicBuffer MaxAccError (이 뒤 R2 가 멈춰 다리 8~16 실패). 이상적 모드는 그대로
            for _ in range(RESTART_FRAME_ROUNDS):   # 영상 한 장이 영상 간격보다 오래 걸려도 끝나게 (이상적 모드는 한 번)
                if self.cam_next > self.clock.now():
                    break
                self._frames_until(self.clock.now())
        S = st.next_stamp(self.clock.now())
        self._frames_until(S - lead)
        late = 0.0 if self.ideal else st.catch_up(self.clock.now)
        S = st.stamp
        self.clock.sleep_until(S - lead)
        return S, late

    def _slack_of(self, cl, first, d_now, dd_now):
        """거리 {'raw', 'dyn'} 의 first 번째 점부터의 안전 검사 여유 (0 이상 = 통과) 와 실제 거리 최소: 실제 거리 ≥ min(margin_m, 지금),
        새 물체 거리 ≥ min(new_margin_m, 지금) — 이미 그 안이면 더 나빠지지만 않으면 됨 (빠져나가는 움직임은 막지 않음)."""
        g = self.c.guard
        d = float(cl['raw'][first:].min())
        return min(d - min(g.margin_m, d_now), float(cl['dyn'][first:].min()) - min(g.new_margin_m, dd_now)), d

    def _cycle_check(self, s0, u):
        """후보 u → 성형된 y·메시지 점 → 안전 검사 (성형된 y = 로봇이 실제로 가는 명령: 지금 + 명령 궤적 점 + 꼬리. 성형 없음이면 2026-10-14
        전과 같은 점) → 개입했으면 감속 정지 후보로 바꿈. 반환 (u, y, 점, 검사 값 {d_now, d_seg, dd_now, dd_seg, guard, keep, cp_mm, cp_kind})."""
        y, pts = self._shape_points(u)
        uu = self._cp_u(u) if self.cp else None          # 성형 여유 보존이 켜져 있으면 u 표본 거리도 같은 한 번의 계산으로
        cl, cu = self._clearances_with_u(pts, uu)
        d_now = float(cl['raw'][0])
        self._clr_mpc = float(cl['mpc'][0]) if 'mpc' in cl else None     # 지금 y 자리의 MPC 지도 거리 (비키기 판단)
        dd_now, dd_seg = float(cl['dyn'][0]), float(cl['dyn'][1:].min())
        slack, d_seg = self._slack_of(cl, 1, d_now, dd_now)
        guard, keep = None, False
        cp_mm = cp_kind = None
        if self.cp:                                         # 성형 여유 보존: 성형된 y 가 최근 u 보다 장애물에 가까워지면 개입
            s_cp, cp_kind = self._cp_slack(uu, cu, pts, cl)
            cp_mm = round(s_cp * 1000, 2)
            slack = min(slack, s_cp)
        if slack < 0:                                       # 실제 거리 지도에서 닿거나 새 물체 6 cm 안으로 (이미 안이면 더 가까워짐)
            guard, brk, d_g = self._guard(s0, slack, d_now, dd_now)
            keep = guard == 'keep'
            if guard == 'brake':
                u, y, pts = brk
            if guard is not None:
                d_seg = d_g
        return u, y, pts, {'d_now': d_now, 'd_seg': d_seg, 'dd_now': dd_now, 'dd_seg': dd_seg, 'guard': guard, 'keep': keep,
                           'cp_mm': cp_mm, 'cp_kind': cp_kind}

    def _cycle_send(self, S, u, y, pts, keep):
        """메시지를 보냄 (또는 안 보냄: 안전 검사의 '앞 메시지 계속', 늦음) → {cmd_next, y_next, msg, msg_t0, n_pre, late_send, first, lead_ms}."""
        c, st, n = self.c, self.stream, self.n_dt
        cmd_next = (u[n, 0], u[n, 1], u[n, 2])            # 다음 스탬프의 성형 전 명령 (다음 명령 궤적·MPC 의 시작)
        y_next = y[n, 0]                                 # 그때 로봇이 따라가는 명령 (성형된 y)
        msg, msg_t0, n_pre = st.message(pts)
        # 늦음 (실시간): 스탬프 뒤에 도착하면 JTC 가 앞 궤적 → 새 궤적으로 그 차이만큼 튐 (실측 1.4 mrad) → 안 보냄
        late_send = not self.ideal and self.clock.now() > S - c.stream.late_send_s
        first = not st.pts
        if late_send and first:
            # (다시) 시작 첫 메시지가 늦음: 이어 갈 앞 메시지가 없고 로봇은 서 있으므로 보내지 않고 다음 주기에 시간표를 새로 시작
            #  (MPC 는 한 걸음 갔으므로 로봇 명령 = 정지 상태로 다시 맞춤)
            st.stamp = None
            if self.style == 'cmpc':
                self.mpc.restart(*st.cmd)
            else:
                self.mpc.resync(*st.cmd)
        elif keep or late_send:
            st.kept()                                   # 앞 메시지 계속 (실행 중인 u 가 계속 실행됨 = 성형 기억)
            self.robot.keep(st.y_state(st.k + n)[0])
        else:
            self.robot.send(msg_t0, msg, y_next)
            st.sent(pts, cmd_next, u, y)
            self._log_sent(S, pts)
        lead_ms = None if self.ideal else round((S - time.time()) * 1000, 1)
        return {'cmd_next': cmd_next, 'y_next': y_next, 'msg': msg, 'msg_t0': msg_t0, 'n_pre': n_pre, 'late_send': late_send,
                'first': first, 'lead_ms': lead_ms}

    def _cycle_record(self, t, S, qm, u, pts, info, rec, chk, snd, late, cyc_ms):
        """주기 기록 (rec['cycles']) + 안전 검사가 개입했으면 MPC 다시 맞춤 (_resync_after_guard) + 늦음 사건."""
        cmd_next, y_next, lead_ms = snd['cmd_next'], snd['y_next'], snd['lead_ms']
        d_now, d_seg, dd_now, dd_seg = chk['d_now'], chk['d_seg'], chk['dd_now'], chk['dd_seg']
        carrot = rec['carrot'] if rec['carrot'] is not None else info['target']
        cy = {'t': round(t, 4), 'stamp': round(S, 4), 'q': np.round(qm, 6).tolist(), 'carrot': np.round(carrot, 6).tolist(),
              'cmd': np.round(cmd_next[0], 6).tolist(), 'cmd_y': np.round(y_next, 6).tolist(),
              'cmd_v': np.round(cmd_next[1], 5).tolist(), 'cmd_a': np.round(cmd_next[2], 4).tolist(),
              'seq_a_max': round(float(np.abs(np.array([p[3] for p in pts[1:self.n_main // self.step_n + 1]])).max()), 3),
              'u_a_max': round(float(np.abs(u[1:self.n_main + 1, 2]).max()), 3),
              'raw_acc_ratio': rec['raw_acc_ratio'], 'vel_ratio': rec.get('vel_ratio'), 'start_err': rec.get('start_err'),
              'lim_dev': rec['lim_dev'], 'replan': rec.get('replan'),
              'goal': None if self.mpc.goal is None else np.round(self.mpc.goal, 5).tolist(), 'mode': info['mode'],
              'leg': self.label, 'mpc_ms': info['ms'], 'cyc_ms': cyc_ms, 'boxes': [[np.round(b[0], 4).tolist(), b[1]] for b in self._boxes(t)[1:]],
              'feasible': info['feasible'], 'lead_ms': lead_ms, 'late_s': late,
              'clr_now_mm': round(d_now * 1000, 1), 'clr_seg_mm': round(d_seg * 1000, 1), 'guard': chk['guard'], 'cp_mm': chk['cp_mm'],
              'cp_kind': chk['cp_kind'], 'msg_t0': round(snd['msg_t0'], 4), 'n_pre': snd['n_pre'], 'skipped': chk['keep'] or snd['late_send'],
              'final': rec['final'], 'clamp': rec['clamp'], 'carrot_out': rec['carrot_out'], 'held': rec['held'],
              'clr_new_now_mm': round(min(dd_now, FAR_M) * 1000, 1), 'clr_new_seg_mm': round(min(dd_seg, FAR_M) * 1000, 1),
              'clr_mpc_mm': None if self._clr_mpc is None else round(min(self._clr_mpc, FAR_M) * 1000, 1)}
        if self.record_sent:
            cy['sent'] = np.array([[tt, *q] for tt, q, v, a in snd['msg']], np.float64)    # 저장할 때 리스트로 (run 의 GC 설명)
        if chk['guard'] is not None:
            self._resync_after_guard(cy)
        self.rec['cycles'].append(cy)
        if snd['late_send']:
            self.event('메시지 늦음 → 안 보냄 ' + ('(첫 메시지: 다음 주기에 새로 시작)' if snd['first'] else '(앞 메시지 계속)'), lead_ms=lead_ms)
        if late:
            self.event(f'주기 늦음 → {late * 1000:.0f} ms 뒤 격자 시각으로 (앞 메시지 상태에서 다시)', lead_ms=lead_ms)

    def _resync_after_guard(self, cy):
        """안전 검사가 개입했으면 (로봇이 MPC 쪽으로 못 감) MPC 상태를 로봇 명령 상태로 (rt_mpc.Mpc.resync) — 그 위치에서 새로 시작
        (rt_config.MpcCfg 설명: 가속까지 넣으면 MPC 가 발산, 2026-10-14). 맞춘 값은 주기 기록 cy 에 (rt_tools replay 가 그대로 씀).
        [시도 후 뺌] 20° 넘게 벌어질 때마다 맞추기 = 평소 빠른 이동에서도 자주 걸려 다리 실패. 못 피하면 멈추고 다시 맞추기 (2026-10-14) =
        한 실행에 74 번 멈추며 MPC 를 흔들어 발산 → 로봇이 목표 반대로 젖혀짐 (사용자 지적) → 뺌.
        [cmpc] 시간표 MPC 는 매 주기 실행된 u 에서 시작하므로 상태는 저절로 맞음 — 앞 해 (실행 안 된 계획) 만 버리고 새로 시작."""
        st = self.stream
        cy['resync_mpc'] = round(float(np.max(np.abs(st.cmd[0] - self.mpc.command()))), 3)
        q, v, a = st.cmd
        cy['resync_to'] = [np.round(q, 6).tolist(), np.round(v, 5).tolist(), np.round(a, 4).tolist()]
        if self.style == 'cmpc':
            self.mpc.restart(q, v, a)
        else:
            self.mpc.resync(q, restart=True)

    def wait_cycles(self, seconds):
        end = self.clock.now() + seconds
        while self.clock.now() < end:
            self.cycle()

    # ---------------- 전역 계획·다리 ----------------
    def _plan_from_rest(self, goal):
        """전역 계획 (그동안 MPC 멈춤 → JTC 는 마지막 메시지의 꼬리로 멈춤). 꼬리 끝(로봇이 멈출 자리)에서 계획하고,
        꼬리가 끝난 뒤 MPC 를 그 자리 정지 상태로 다시 시작 (시간표도 다시)."""
        self._live_wait()                       # (실시간) 계획기 스레드가 돌고 있으면 끝날 때까지 (계획기는 한 번에 하나)
        st = self.stream
        q_rest = st.tail_end.copy()             # 로봇은 마지막 메시지 꼬리 끝까지 감
        path, wall = self.W.plan(q_rest, goal, solve=self._solve)
        if self.ideal:
            self.clock.advance(self.c.task.ideal_plan_s)
            self.robot.rest(q_rest)
        else:
            self.clock.sleep_until(st.tail_done - self.c.stream.lead_s)
        self._restart_at(q_rest)
        return path, wall

    def _solve(self, q0, q1, seed=None):
        """전역 계획 한 번 (World.plan_solve) → 계획 뒤 고치기 (_post_solve). 계획기만 써서 다른 스레드에서 불러도 됨 (멈추지 않는 다시 계획)."""
        path = self.W.plan_solve(q0, q1, seed)
        return None if path is None else self._post_solve(path, q0, q1)

    def _post_solve(self, path, q0, q1):
        """[2026-10-27 R-1] 계획기가 낸 경로 (시각 [N], 관절 [N, 6]) 를 검증 (plan_check) 전에 고치는 자리 — 기본은 그대로. GUI 서버가 속도 배율
        (시각을 1/배율 로 고르게 늘림)·끝점 맞추기를 여기서 (World.plan_solve 를 바꿔 끼우던 것 대신). 실시간이면 계획기 스레드에서 불림."""
        return path

    def _leg_timeout_s(self) -> float:
        """[2026-10-27 R-1] 다리 시간 초과 (s, 계획 성공 뒤 움직임 — 계획 실패 되풀이는 _plan_until 이 task.leg_timeout_s 그대로). GUI 서버가
        속도 배율로 늘림 (self.c 를 바꿔 끼우던 것 대신)."""
        return self.c.task.leg_timeout_s

    def _restart_at(self, q, t_done=None):
        """[2026-10-27 R-1] 명령 시간표·성형 기억·MPC 를 서 있는 자세 q 에서 새로 시작 (실행 중이던 메시지는 꼬리 끝까지 실행된 것으로 기록).
        t_done = 로봇이 q 에 서는 시각 (Stream.tail_done 을 그 값으로 — None 이면 그대로 = 마지막 메시지 꼬리 끝). 다시 계획 (_plan_from_rest)
        과 GUI 서버의 서보 켜기·조그 끝·그 자리에서 새로 시작이 함께 씀."""
        q = np.array(q, float)
        self._flush_sent()
        self.stream.restart(q)
        if t_done is not None:
            self.stream.tail_done = t_done
        self.mpc.start(q)
        self.last_target, self.holding = q.copy(), False

    def _plan_until(self, label, goal, L) -> bool:
        """멈춰서 계획, 실패하면 그 자리를 지키며(MPC 는 계속 피함) retry_s 마다 다시 — 성공하면 따라가기 시작. 시간 초과면 False.
        [2026-10-13] 다시 계획을 한 번만 하던 때는 실패하면 지키기 모드로 남아 B 앞 어중간한 자세로 ~20 s 서 있었다 (rt_20261013_195227)."""
        while True:
            path, wall = self._plan_from_rest(goal)
            L['plans'].append(round(wall, 3))
            self.event(f'{label} 계획 {"성공" if path else "실패"} {wall:.2f}초', plan_s=wall)
            if path is not None:
                L['path'] = path
                self._log_path(L, path, 'rest')
                self.mpc.follow(*path)
                L.setdefault('plan_duration', float(path[0][-1]))
                return True
            if self.clock.now() - L['t0'] > self.c.task.leg_timeout_s:
                return False
            self.mpc.hold()
            self.wait_cycles(self.c.plan.retry_s)

    def _log_path(self, L, path, kind):
        """[2026-10-16] 받아 들인 전역 경로 기록 (경로 품질 분석 tools/motion_quality.py): 시각, 종류 ('rest' 멈춰 계획 / 'live' 멈추지 않는
        다시 계획), 점 간격 s, 관절 [N, 6] (rad, 소수 4 자리)."""
        t, q = path
        L.setdefault('paths', []).append({'t': round(self.clock.now(), 3), 'kind': kind, 'dt': round(float(t[1] - t[0]), 5) if len(t) > 1 else 0.0,
                                          'q': np.round(np.asarray(q, float), 4).tolist()})

    def _end_leg(self, L):
        L.pop('path', None)                     # 기록(JSON)에는 경로 배열을 넣지 않음 (받아 들인 경로는 L['paths'] 에 줄여서)
        self.rec['legs'].append(L)
        return L

    def leg(self, label, goal):
        t = self.c.task
        self.label = label
        p = self.c.plan
        L = {'label': label, 't0': self.clock.now(), 'plans': [], 'ok': False, 'live_ok': 0, 'live_fail': 0}
        self.event(f'{label} 시작')
        if not self._plan_until(label, goal, L):
            L['why'] = '계획 실패'
            self.event(f'{label} 시간 초과 (계획 실패)')
            self.mpc.hold()
            return self._end_leg(L)
        # 목표 자리 막힘 판정의 기준 = 다리 시작 때 목표 자세의 새 물체 지도 거리. [2026-10-16] 구분 없는 지도 (cr3a_mapper 머리말) 에서는
        #  기둥 같은 정지 물체도 새 물체 지도에 있어 목표 자세가 반응 거리 안이면 늘 '막힘' 이 됨 → 시작 때보다 1 cm 넘게 가까워졌을 때만
        g0 = float(self.W.clearance(goal[None], grid='dyn')[0])
        arrived = None
        self._back = None
        self._dhist.clear()
        self._dsm.clear()
        self._ahead_since = None                        # ① 경로 앞 막힘이 시작된 시각 (안 막혔으면 None)
        while True:
            info = self.cycle()
            now = self.clock.now()
            if self.mpc.mode == 'move' and info.get('at_end') and float(np.max(np.abs(self.robot.measured() - goal))) < t.arrive_tol_rad:
                arrived = now if arrived is None else arrived
                if now - arrived >= t.arrive_hold_s:
                    L.update(ok=True, t1=now, duration=now - L['t0'])
                    self.event(f'{label} 도착 {now - L["t0"]:.2f}초')
                    break
            else:
                arrived = None
            if now - L['t0'] > self._leg_timeout_s():
                L['why'] = '시간 초과'
                self.event(f'{label} 시간 초과')
                break
            if self._live is not None:                  # (나) 다시 계획이 도는 중: 끝났으면 새 경로로 (로봇은 그동안 계속 움직임)
                self._live_poll(label, L)
                self._back_check(label, goal, L)        # (가') 계획이 도는 동안에도 다가오면 비킴
                continue
            if self._back_check(label, goal, L):        # (가') 비키는 중: 아래 막힘 판단은 건너뜀
                continue
            if self.mpc.mode != 'move':
                continue
            why = None
            if info.get('stuck'):
                why = '막힘 (경로를 3 s 못 따라감)'
            elif p.ahead_check and self.style == 'carrot':
                # (가) 경로 앞이 지도와 닿나 — [2026-10-16 ①] 막힘이 ahead_persist_s 동안 이어질 때만 다시 계획 (주기마다 봐서 이어진 시간을 잼)
                r = self._path_ahead()
                if r is not None and r[0] < p.ahead_block_m:
                    self._ahead_since = now if self._ahead_since is None else self._ahead_since
                    if now - self._ahead_since >= p.ahead_persist_s - 1e-9 and now - self._live_last >= p.replan_cooldown_s:
                        why = f'경로 앞 막힘 ({r[1]:.1f} s 앞 {r[0] * 1000:.0f} mm, {now - self._ahead_since:.1f} s 째)'
                else:
                    self._ahead_since = None
            if why is None:
                continue
            # [2026-10-13 사용자 지적] 목표 자리가 물체에 막혀 있으면 다시 계획해도 실패하고 엉뚱한 자세로 서 있었다
            #  (rt_20261013_203349). 목표 자세가 막혔으면 지난 경로를 지금과 가장 가까운 점부터 따라가 목표 쪽으로 가서 기다린다
            #  (물체 앞에서는 MPC·안전 검사가 세우고, 목표가 비면 그대로 도착). 경로 앞 막힘이면 다시 계획하지 않음 (MPC·안전 검사가 피함)
            blocked = float(self.W.clearance(goal[None], grid='dyn')[0]) < min(self.c.dist.act_new, g0 - GOAL_CLOSER_M)
            if blocked:
                if info.get('stuck') and 'path' in L:
                    if not L.get('waiting'):
                        self.event(f'{label} 목표 자리가 막힘 → 경로 따라 가까이 가서 기다림')
                    L['waiting'] = True
                    self.mpc.follow(*L['path'], from_nearest=True)
                continue
            L['waiting'] = False
            if now - self._live_last < p.replan_cooldown_s:
                continue
            self._live_submit(label, goal, why)        # (나) 멈추지 않고 다시 계획 ([2026-10-27 R-1] 멈춰서 다시 계획하는 되돌리기는 뺌)
        self._live_wait()                               # 다리가 끝났으면 진행 중인 다시 계획은 버림
        self.mpc.hold(goal if L['ok'] else None)     # 못 갔으면 목표로 곧장 끌려가지 않게 그 자리 지키기
        return self._end_leg(L)

    # ---------------- (가)(나) 경로 앞 막힘 → 멈추지 않고 다시 계획 (rt_config.PlanCfg, 2026-10-16) ----------------
    def _path_ahead(self, from_q=None):
        """(가) 따라가는 전역 경로의 [기준 시각 + ahead_from_s, + ahead_to_s] 점들 (ahead_step_s 간격) 의 실제 거리 지도 (충돌 구 + 바닥 면)
        거리 최소 (m) 와 그 점이 기준 시각에서 몇 s 앞인지. 볼 점이 없으면 None.
        from_q = [비키기] 기준 시각 대신 경로에서 이 자세와 가장 가까운 점부터 (비킨 자리 ~ 앞 ahead_to_s 를 빼지 않고 다 봄)."""
        p, m = self.c.plan, self.mpc
        if m.path is None:
            return None
        t, q = m.path
        if from_q is None:
            t0, a = m.ref_t, m.ref_t + p.ahead_from_s
        else:
            t0 = a = float(t[int(np.argmin(np.max(np.abs(q - from_q), axis=1)))])
        b = min(t0 + p.ahead_to_s, float(t[-1]))
        if b < a:
            return None
        ts = np.arange(a, b + 1e-9, p.ahead_step_s)
        d = self.W.clearance(m.path_at(ts), grid='raw')
        k = int(np.argmin(d))
        return float(d[k]), float(ts[k] - t0)

    def _back_check(self, label, goal, L) -> bool:
        """(가') 비키기 (rt_config.PlanCfg.back_off, 사용자 결정 2026-10-16·10-17): 주기마다 지금 y 자리의 MPC 지도 거리로 판단 (_back_dist).
        시작 = 경로를 따라가는 중 (move) 막혀 서 있는데 다가옴 → 그 자리에서 비킴 (_back_begin). 비키는 중 = 길이 막혀 있으면 다시 계획하고,
        안 다가옴 + 길 열림 또는 back_max_s 지남 → 경로로 돌아감 (_back_continue). 새 경로가 나오면 (_live_poll 이 이음) 끝.
        비키는 중이면 True. [2026-10-27 R-1] 지나온 경로를 따라 물러나기 (1~5 차) 와 그 사이 되돌리기 설정은 뺌 (rt_config.PlanCfg 설명)."""
        p, m = self.c.plan, self.mpc
        if not p.back_off or self.style != 'carrot' or self._clr_mpc is None:
            return False
        now = self.clock.now()
        d, peak, approaching = self._back_dist(now)
        if self._back is not None and m.mode != 'back':         # 새 경로로 끝남 (_live_poll) 또는 다리 끝
            self.n_back['new_path'] += int(m.mode == 'move')
            self._back = None
        if m.mode == 'move':
            return self._back_begin(label, now, d, peak, approaching)
        if m.mode != 'back':
            return False
        return self._back_continue(label, goal, L, now, d, approaching)

    def _back_dist(self, now):
        """이번 주기 지금 y 자리 MPC 지도 거리를 기록에 넣고 → (d, peak, 다가옴): d = back_smooth_s 동안의 최소 (경계 채우기가 영상마다 켜졌다
        꺼져서), peak = 그렇게 고른 값들의 back_window_s 안 최대 ([2026-10-17] 깜빡임을 다가옴으로 안 봄), 다가옴 = d < 0 (팔 지우기 안으로
        들어와 경계 채우기에 닿음) 또는 d < 반응 거리이고 peak 보다 back_approach_m 넘게 줆."""
        p = self.c.plan
        H = self._dhist
        H.append((now, self._clr_mpc))
        while H and H[0][0] < now - p.back_smooth_s - 1e-9:
            H.popleft()
        d = min(x for _, x in H)
        S = self._dsm
        S.append((now, d))
        while S and S[0][0] < now - p.back_window_s - 1e-9:
            S.popleft()
        peak = max(x for _, x in S)
        return d, peak, d < 0.0 or (d < self.c.dist.act_new and d < peak - p.back_approach_m)

    def _back_begin(self, label, now, d, peak, approaching) -> bool:
        """경로를 따라가는 중: 기준 시각이 back_stall_s 동안 못 나아갔고 (막혀 서 있음) 다가오면 그 자리에서 비킴 (Mpc.back: 목표 = 지금 로봇
        명령 자세로 고정) → True."""
        p, m = self.c.plan, self.mpc
        if not (m.clock - m.t_progress >= p.back_stall_s and approaching) or not m.back(np.asarray(self.stream.cmd[0], float)):
            return False
        self.n_back['start'] += 1
        # calm = 안 다가옴이 시작된 시각, clear = 경로 앞이 열린 채 이어진 시작 시각, blk = 경로 앞이 막힌 채 이어진 시작 시각
        self._back = {'t0': now, 'calm': None, 'clear': None, 'blk': None}
        self.event(f'{label} 막혀 서 있는데 물체가 다가옴 (MPC 지도 {d * 1000:.0f} mm, {p.back_window_s:.1f} s 안 최대 '
                   f'{peak * 1000:.0f} mm) → 그 자리에서 비킴', clr_mpc_mm=round(d * 1000, 1))
        return True

    def _back_continue(self, label, goal, L, now, d, approaching) -> bool:
        """비키는 중: 지금 자리부터 경로 앞이 ahead_persist_s 이어 막혀 있으면 다시 계획 (쿨다운마다). 끝 = 안 다가옴 (MPC 지도 거리 ≥ 0 이고
        안 줆) 이 back_clear_s 이어지고 경로 앞이 back_clear_hold_s 열림 (막혀 있으면 그 자리에서 기다리며 다시 계획) 또는 back_max_s 지남 →
        로봇 명령 자리와 가장 가까운 경로 점부터 다시 (False). 아직 비키는 중이면 True."""
        p, B = self.c.plan, self._back
        calm = d >= 0.0 and not approaching
        B['calm'] = (now if B['calm'] is None else B['calm']) if calm else None
        if B['calm'] is None:
            B['clear'] = None
        r = self._path_ahead(from_q=np.asarray(self.stream.cmd[0], float))     # 지금 자리부터 경로 앞 (주기마다 한 번만 계산)
        # [2026-10-17 (작업기록 5-24)] 비키는 중에도 경로 앞이 ahead_persist_s 이어 막혀 있으면 다시 계획 ('안 다가옴' 을 안 기다림)
        blk = r is not None and r[0] < p.ahead_block_m
        B['blk'] = (now if B['blk'] is None else B['blk']) if blk else None
        if blk and now - B['blk'] >= p.ahead_persist_s - 1e-9 and self._live is None and \
                now - self._live_last >= p.replan_cooldown_s:
            self._live_submit(label, goal, f'비키는 중 길 막힘 ({r[1]:.1f} s 앞 {r[0] * 1000:.0f} mm, {now - B["blk"]:.1f} s 째)')
        why = None
        if B['calm'] is not None and now - B['calm'] >= p.back_clear_s - 1e-9:
            if r is not None and r[0] < p.ahead_block_m:
                B['clear'] = None                       # 막혀 있음 → 그 자리에서 기다리며 다시 계획 (쿨다운마다 한 번)
                if self._live is None and now - self._live_last >= p.replan_cooldown_s:
                    self._live_submit(label, goal, f'비킨 뒤 길 막힘 ({r[1]:.1f} s 앞 {r[0] * 1000:.0f} mm)')
            else:                                       # 열린 채 back_clear_hold_s 이어져야
                B['clear'] = now if B['clear'] is None else B['clear']
                why = '안 다가옴·길 열림' if now - B['clear'] >= p.back_clear_hold_s - 1e-9 else None
        if why is None and now - B['t0'] >= p.back_max_s - 1e-9:
            why = f'{p.back_max_s:.0f} s 지남'
        if why is None:
            return True
        self._follow_from_robot(L['path'])
        self.n_back['resume'] += 1
        self._back = None
        self._ahead_since = None
        self.event(f'{label} 비키기 끝 ({why}, MPC 지도 {d * 1000:.0f} mm, 비킨 시간 {now - B["t0"]:.2f} s) → 다시 목표 쪽으로',
                   clr_mpc_mm=round(d * 1000, 1))
        return False

    def _follow_from_robot(self, path):
        """비키기 뒤 다시 앞으로 (또는 비키는 중 새 경로): MPC 를 로봇 명령 자리에서 새로 시작하고 (resync restart = 앞 해 버림) 그 자리와
        가장 가까운 경로 점부터 따라감. [2026-10-16 물러나기 4 차] 그 전 (MPC 상태 = 당근과 가장 가까운 점부터) 은 물러나는 동안 당근이 로봇보다
        25~40° 뒤라 다시 뒤로 갔다 옴 (WMX 13 번 다리). [2026-10-27 R-1] 그 되돌리기 (back_resync False) 는 뺌."""
        qc = np.asarray(self.stream.cmd[0], float)
        self.mpc.resync(qc, restart=True)
        self.mpc.follow(*path, from_nearest=True, near_q=qc)

    def _live_submit(self, label, goal, why):
        """(나) 멈추지 않고 다시 계획 요청. 출발 = 다음 스탬프의 성형 전 명령 자세 (로봇은 그동안 지금 경로를 따라 계속 감).
        지도는 지금 계획기에 복사 (World.plan_prep). 이상적 모드 = 바로 풀고 ideal_plan_s 뒤에 씀 (계산 시계, 결정적),
        실시간 = 계획기 스레드 (별도 CUDA 스트림, 지도 복사가 끝난 뒤 시작)."""
        now = self.clock.now()
        q0 = np.array(self.stream.cmd[0], float)
        goal = np.array(goal, float)
        self._live_last = now
        self.n_live['req'] += 1
        self.W.plan_prep('start', q0=q0, clear_m=self.c.plan.live_clear_m)     # 지금 자세 둘레만 실제 거리 (rt_config.PlanCfg.live_clear_m)
        cl = self.W.clearances(np.vstack([q0, goal]), ('raw', 'mpc'))       # (진단) 출발·목표 자세의 두 지도 거리
        job = {'t_req': now, 'why': why, 'label': label,
               'clr': {k: [round(float(v) * 1000, 1) for v in cl[k]] for k in ('raw', 'mpc')}}
        seed = self._remaining_path(q0)          # ③ 지금 경로의 남은 부분을 시작값으로
        if self.ideal:
            t0 = time.perf_counter()
            job['path'] = self._solve(q0, goal, seed)
            job['wall'] = time.perf_counter() - t0
            job['ready'] = now + self.c.task.ideal_plan_s
        else:
            import torch
            if self._pool is None:
                from concurrent.futures import ThreadPoolExecutor
                self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='live_plan')
                self._pstream = torch.cuda.Stream()
            ev = torch.cuda.Event()
            ev.record()
            job['fut'] = self._pool.submit(self._live_job, q0, goal, ev, seed)
        self._live = job
        self.event(f'{label} {why} → 멈추지 않고 다시 계획')

    def _remaining_path(self, q0):
        """[2026-10-16 ③] 지금 따라가는 전역 경로에서 q0 와 가장 가까운 점부터 끝까지 [M, 6] (다시 계획 시작값). 경로가 없으면 None."""
        if self.mpc.path is None:
            return None
        q = self.mpc.path[1]
        return np.array(q[int(np.argmin(np.max(np.abs(q - q0), axis=1))):], float)

    def _live_job(self, q0, goal, ev, seed=None):
        """(계획기 스레드) 지도 복사 (메인 스트림) 가 끝난 뒤 계획기 스트림에서 풂 → (경로 또는 None, 걸린 시간 s)."""
        import torch
        t0 = time.perf_counter()
        with torch.cuda.stream(self._pstream):
            self._pstream.wait_event(ev)
            path = self._solve(q0, goal, seed)
        return path, time.perf_counter() - t0

    def _live_poll(self, label, L):
        """(나) 다시 계획이 끝났으면: 지금 지도로 다시 검증 (계획하는 동안 물체가 움직였을 수 있음) → 새 경로를 지금 MPC 자세에서 가장
        가까운 점부터 따라감 (MPC 는 다시 시작하지 않음 = 멈추지 않음). 실패면 지금 경로 계속 (replan_cooldown_s 뒤 다시)."""
        J = self._live
        if self.ideal:
            if self.clock.now() < J['ready'] - 1e-9:
                return
            path, wall = J['path'], J['wall']
        else:
            if not J['fut'].done():
                return
            try:
                path, wall = J['fut'].result()
            except Exception as e:                      # 계획기 스레드 오류는 계획 실패로 (실행은 계속)
                path, wall = None, 0.0
                self.event(f'{label} 다시 계획 오류: {e!r}')
        self._live = None
        L['plans'].append(round(wall, 3))
        wait_ms = (self.clock.now() - J['t_req']) * 1000
        if path is not None and self.mpc.mode in ('move', 'back') and J['label'] == label:     # 비키는 중이면 새 길로 (비키기 끝)
            path = self.W.plan_check(path)
            # [2026-10-16 ①] 지금 경로 앞이 이제 안 막혔으면 (물체가 지나감) 지금 경로 그대로 — '막힘 3 s' 요청은 늘 받음
            if path is not None and self.mpc.mode == 'move' and not J['why'].startswith('막힘'):
                r = self._path_ahead()
                if r is None or r[0] >= self.c.plan.ahead_block_m:
                    self.n_live['keep'] += 1
                    self.event(f'{label} 새 경로 안 받음 (지금 경로 앞이 이제 안 막힘, 계획 {wall:.2f}초) → 지금 경로 계속', plan_s=wall)
                    return
            if path is not None:
                if self.mpc.mode == 'back':                 # 비키는 중 받은 새 경로: 로봇 명령 자리에서 이음 (_follow_from_robot)
                    self._follow_from_robot(path)
                else:
                    self.mpc.follow(*path, from_nearest=True)
                L['path'] = path
                self._ahead_since = None
                self._log_path(L, path, 'live')
                L['live_ok'] += 1
                self.n_live['ok'] += 1
                self.event(f'{label} 새 경로로 (멈추지 않음, 계획 {wall:.2f}초, 요청 뒤 {wait_ms:.0f} ms)', plan_s=wall)
                return
        L['live_fail'] += 1
        self.n_live['fail'] += 1
        c0 = J['clr']
        self.event(f'{label} 다시 계획 실패 (멈추지 않음, 계획 {wall:.2f}초, 출발 실제 {c0["raw"][0]} / MPC {c0["mpc"][0]} mm, '
                   f'목표 실제 {c0["raw"][1]} / MPC {c0["mpc"][1]} mm) → 지금 경로 계속', plan_s=wall)

    def _live_wait(self):
        """진행 중인 다시 계획을 버림. 실시간은 계획기 스레드가 끝날 때까지 기다림 (계획기는 한 번에 하나만 풂)."""
        J, self._live = self._live, None
        if J is None:
            return
        if 'fut' in J:
            try:
                J['fut'].result()
            except Exception:
                pass
        self.n_live['drop'] += 1

    # ---------------- 임무 ----------------
    def run(self, out: Path):
        t = self.c.task
        qa, qb = (np.array(g, float) for g in t.goals(self.c.model))     # 로봇 모델의 A·B (rt_config.TaskCfg.goals)
        # [2026-10-13] 제어 루프 동안 파이썬 순환 GC 끔: 주기마다 쌓는 기록(파이썬 객체 수십만 개)을 GC 가 훑느라 프로세스가
        #  ~0.4 s 멈춰 메시지가 338 ms 늦게 도착 → JTC 가 정지에서 0.21 rad/s 로 뜀 → MaxAccError (rt_20261013_210145).
        #  참조 수로 대부분 바로 풀리고, 순환 쓰레기는 2~3 분 실행에 문제될 만큼 안 쌓임
        gc.collect()
        gc.disable()
        self.W.cm.start_background()            # 켜기: 지도가 차도록 1 초 (cr3a_mapper 머리말)
        self.mpc.start(self.q0)
        self.event('켜기: 배경 모으기')
        while not self.W.cm.bg_ready:
            self.cycle()
        self.event('배경 완료')
        # (보고서 3D) 정지 물체 칸 기록 — 구분 없는 지도 (2026-10-16) 는 정지 물체 칸이 따로 없어 빈 글자 (기둥도 새 물체 칸 기록 newvox 에
        #  들어감). [2026-10-27 R-1] 키는 기록을 읽는 rt_report·rt_tools 때문에 남김
        self.rec['static_vox'] = ''
        try:
            self.leg('시작→A', qa)
            self.wait_cycles(t.hold_at_goal_s)
            self.t_go = self.clock.now()
            self.event('GO: 움직이는 물체 출발')
            for k in range(t.legs):
                lbl, g = (('A→B', qb), ('B→A', qa))[k % 2]
                self.leg(f'{k + 1}. {lbl}', g)
                self.label = '지키기'
                self.wait_cycles(t.hold_at_goal_s)
            self.label = '끝 지키기'
            self.wait_cycles(t.end_hold_s)
        finally:
            self._live_wait()
            if self._pool is not None:
                self._pool.shutdown(wait=True)
            self.rec['live'] = dict(self.n_live)
            self.rec['back'] = dict(self.n_back)
            self.save(out)
            gc.enable()

    def save(self, out: Path):
        rec = self.rec
        # [2026-10-26] GUI 는 물체를 치우면 t_go = None → 저장 때 덮어쓰면 움직인 구간을 잃음 (보고서 물체 거리가 빔). 치운 뒤엔
        #  GUI 가 남긴 마지막 구간 (rec t_go·t_go_end) 을 둠. rt_run 은 t_go 를 되돌리지 않아 같음
        rec['t_go'] = self.t_go if self.t_go is not None else rec.get('t_go')
        self._flush_sent()
        if self._sent_log:                      # 실행된 명령 (보낸 메시지 점, 5 ms + 멈출 때 꼬리 10 ms): 시각·위치·속도·가속
            a = np.vstack(self._sent_log)
            rec['sent_traj'] = {'t': np.round(a[:, 0], 4).tolist(), 'q': np.round(a[:, 1:7], 6).tolist(),
                                'v': np.round(a[:, 7:13], 5).tolist(), 'a': np.round(a[:, 13:19], 4).tolist(),
                                'seg': a[:, 19].astype(int).tolist()}
        for cy in rec['cycles']:
            if isinstance(cy.get('sent'), np.ndarray):
                cy['sent'] = np.round(cy['sent'], 6).tolist()
        if not self.ideal:
            rec['jtc'] = self.robot.jtc
        (out / 'rt_run.json').write_text(json.dumps(rec))
        # 실행된 성형 전 명령 u (1 ms, 세그먼트 = 다시 시작 사이) — 보고서 '성형 확인': WMX 명령 − (u 전체를 한 번에 컨볼루션)
        save_u_exec(out / 'u_exec.npz', self.stream.executed(), self.g, self.kernel.h)
        if self.pvat is not None and self.pvat['S']:
            save_pvat_msgs(out / 'pvat_msgs.npz', self.pvat)
        ms = [cy['mpc_ms'] for cy in rec['cycles']] or [0]
        mp = [f['map_ms'] for f in rec['frames']] or [0]
        self._log(f'저장 {out / "rt_run.json"}: 주기 {len(rec["cycles"])}, MPC 중앙값 {np.median(ms):.1f} ms (최대 {max(ms):.1f}), '
            f'지도 중앙값 {np.median(mp):.1f} ms, 다리 {[(L["label"], L["ok"]) for L in rec["legs"]]}')


def save_u_exec(path: Path, segs, grid_s: float, h) -> bool:
    """실행된 성형 전 명령 u (Stream.executed 세그먼트들) 를 npz 로 (rt_report.shaping_check·rt_tools shaping·wmx_check.py 가 읽는 형식):
    T0·n·before (세그먼트마다), q (이어 붙인 1 ms 칸), grid_s, h. 세그먼트가 없으면 안 씀 (False). [2026-10-27 R-1] GUI 서버의 성형기별
    기록 (u_exec_k<i>.npz) 과 같은 함수."""
    if not segs:
        return False
    np.savez_compressed(path, T0=np.array([sg['T0'] for sg in segs]), n=np.array([len(sg['q']) for sg in segs]),
                        before=np.array([sg['before'] for sg in segs]), q=np.concatenate([sg['q'] for sg in segs]),
                        grid_s=grid_s, h=h)
    return True


def save_pvat_msgs(path: Path, rec) -> None:
    """[2026-10-30 PVAT 단계 1] 보낸 메시지 기록 (Controller._pvat_wrap_send) → npz. 메시지 i 의 점 = pts[off[i]:off[i]+n[i]]
    (열: 메시지 시각 t0 기준 s, q 6, v 6, a 6 — JTC 가 받은 그대로, 앞 n_pre 점은 앞 메시지 점), S = 스탬프 (갈아타는 시각, 유닉스 s),
    wall = 보낸 뒤 시각, pack_us·build_us·pub_us = 보낼 때 걸린 시간 (µs)."""
    n = np.array([len(x) for x in rec['pts']], np.int64)
    np.savez_compressed(path, wall=np.array(rec['wall']), S=np.array(rec['S']), t0=np.array(rec['t0']),
                        n_pre=np.array(rec['n_pre'], np.int64), n=n, off=np.r_[0, np.cumsum(n)[:-1]],
                        pts=np.concatenate(rec['pts']), pack_us=np.array(rec['pack_us']),
                        build_us=np.array(rec['build_us']), pub_us=np.array(rec['pub_us']))


def override(c: Config, items) -> Config:
    """'묶음.이름=값' (예: dist.act_new=0.10, plan.back_off=0, guard.cp_thr=0) 로 설정 바꾸기 — 비교 시험용. 숫자·참거짓·글자만."""
    subs = {}
    for it in items:
        key, val = it.split('=', 1)
        sec, name = key.split('.')
        sub = subs.get(sec, getattr(c, sec))
        cur = getattr(sub, name)
        if isinstance(cur, bool):
            v = val.lower() in ('1', 'true', 'yes')
        elif isinstance(cur, (int, float, str)):
            v = type(cur)(val)
        else:
            raise ValueError(f'{key}: 숫자·참거짓·글자만 바꿀 수 있음')
        subs[sec] = dataclasses.replace(sub, **{name: v})
    # 묶음을 한꺼번에 바꿈 (Config 검사는 한 번): dist.act_new 와 dist.act_static 처럼 같이 바꿔야 하는 값의 중간 상태가 검사에 걸리지 않게
    return dataclasses.replace(c, **subs) if subs else c


def main():
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--robot', choices=['ideal', 'ros'], default='ideal')
    ap.add_argument('--out', required=True)
    ap.add_argument('--scenario', default='moving5', help=f'{sorted(SCENARIOS)}, moving5:숫자 = 변형')
    ap.add_argument('--legs', type=int, default=TaskCfg.legs)         # GO 뒤 A↔B 다리 수
    ap.add_argument('--style', choices=['cmpc', 'carrot'], default=Config.style)   # 명령 u 를 만드는 방식 (rt_config.Config.style)
    ap.add_argument('--model', choices=['tool', 'bare'], default=Config.model)     # 로봇 형상 (rt_model): tool = 끝단 공구 붙음
    # 입력성형 (rt_shaping): 파일 (기본 shaping.yaml) 위에 명령줄 값을 덮음. 예: --shaper ZVD --shaper-hz 8 15 --shaper-zeta 0.02
    ap.add_argument('--shaping-file', default=str(HERE / 'shaping.yaml'))
    ap.add_argument('--shaper', nargs='*', help='none | ZV | ZVD | EI | ETM4 | ETM5 (주파수마다 다르게면 여러 개)')
    ap.add_argument('--shaper-hz', nargs='*', type=float, help='진동 주파수 Hz (여러 개 = 임펄스 컨볼루션)')
    ap.add_argument('--shaper-zeta', nargs='*', type=float, help='감쇠비 (하나 = 모든 주파수)')
    ap.add_argument('--set', nargs='*', default=[], metavar='묶음.이름=값')   # rt_config 값 바꾸기 (비교 시험용, 예 cmpc.nonterminal=1)
    ap.add_argument('--record-sent', action='store_true')             # 주기마다 보낸 메시지 점 전체도 기록 (메시지 분석용, 파일 큼)
    ap.add_argument('--pvat-record', action='store_true')             # [2026-10-30] 보낸 메시지 전체 P·V·A + 보낼 때 시간 → pvat_msgs.npz
    # [검증 전용, 2026-10-15 사용자 결정 "검증에만 쓰고 실제로 장애물 회피는 카메라만"] oracle = 카메라 대신 시뮬레이터의 참 형상 지도
    #  (안 보이는 면·팔 지우기 사각지대 없음) — 성형 때문에 생긴 닿음을 카메라의 기존 한계와 섞지 않고 가릴 때. 이상적 모드에서만
    ap.add_argument('--map', choices=['camera', 'oracle'], default='camera')
    a = ap.parse_args()
    if a.pvat_record:
        os.environ['CR3A_PVAT_RECORD'] = '1'
    if a.map == 'oracle' and a.robot != 'ideal':
        sys.exit('--map oracle 은 검증 전용 (이상적 모드만). 실제 회피는 카메라 지도만 씀')
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    c = override(Config(task=dataclasses.replace(TaskCfg(), legs=a.legs), style=a.style, model=a.model), a.set)
    if a.set:
        log('설정 바꿈:', ' '.join(a.set))
    sfile = a.shaping_file if a.shaping_file and Path(a.shaping_file).exists() else None
    sc = load_cfg(sfile, method=a.shaper, freq_hz=a.shaper_hz, zeta=a.shaper_zeta)
    K = make_kernel(sc)
    log(f'입력성형 {sc.label()} ({sfile or "파일 없음"}): 1 ms 칸 {K.L}, 길이 {K.duration_s * 1000:.1f} ms, '
        f'평균 지연 {K.mean_delay_s * 1000:.1f} ms, 임펄스 {len(K.A)} 개')

    robot_dict = robot_cfg(c.model)
    log(f'로봇 모델 {c.model}: {rt_model.get(c.model).yml.name}, 충돌 구 '
        f'{sum(len(v) for v in robot_dict["kinematics"]["collision_spheres"].values())} 개')
    W = World(robot_dict, c, log=log, oracle=a.map == 'oracle')
    if W.oracle:
        log('[검증 전용] 참 형상 지도 (카메라 대신 시뮬레이터의 상자 모양)')
    grid = W.cm.esdf_mpc()                  # 늘 같은 GPU 버퍼 (영상마다 다시 씀)
    if c.style == 'cmpc':
        from rt_cmpc import CMpc
        mpc = CMpc(robot_dict, c.cmpc, c.dist.act_new, grid, [W.floor], K)
        log(f'성형 MPC (C 방식) {c.cmpc.rate_hz:.0f} Hz: 점 간격 {mpc.opt_dt * 1000:.3f} ms, 매듭 {mpc.n_knots}, '
            f'상태 궤적 점 {mpc.H}, 기억 점 {mpc.Hh}, 성형 {"켬" if mpc.shaped else "끔"}')
    else:
        from rt_mpc import Mpc
        mpc = Mpc(robot_dict, c.mpc, c.dist.act_new, grid, [W.floor])
        log(f'당근 MPC {c.mpc.rate_hz:.0f} Hz')
    vox = mpc.mpc.scene_collision_checker.data.voxels
    log('MPC 가 지도를 복사 없이 봄:', vox is not None and vox.features.data_ptr() == grid.feature_tensor.data_ptr())
    if a.robot == 'ideal':
        robot, clock = IdealRobot(np.zeros(6)), SimClock()
    else:
        robot, clock = RosRobot(), WallClock()
        robot.wait_js()
    try:
        Controller(c, W, mpc, robot, clock, a.scenario, K, record_sent=a.record_sent, shaping_label=sc.label()).run(out)
    finally:
        robot.close()


if __name__ == '__main__':
    main()
