"""JointTrajectory 메시지 시간표 + 입력성형 기억 (numpy, ROS 없음). 주기마다 보내는 메시지를 만들고, 보낸 점들을 절대 시각으로 기억한다.

메시지 = [앞 메시지 점들 (S − pre_s ≤ t < S, 앞 메시지는 지난 스탬프부터라 평소 한 주기)] + [지금 명령 상태 (S)] + [명령 궤적 점들]
        + [멈춤 꼬리]
  S = 이번 주기 스탬프 (명령 첫 점 시각). 계산은 S − lead_s 에 시작해서, 메시지가 S 전에 도착해 JTC 가 끊김 없이 이어 간다.
  앞 메시지 점을 붙이는 이유 (2026-10-13 WMX 실측): Jazzy JTC 는 새 궤적으로 갈아탈 때 궤적 앞 점을 (지난 update 시각,
  한 주기 뒤 명령) 으로 잡아서 (joint_trajectory_controller.cpp: last_commanded_state_ = command_next_,
  last_commanded_time_ = time) 움직이는 중에 명령 위치가 속도 × 10 ms 튀었다 (1 ms 곡선 점에서 maxAcc 차단).
  지금 실행 중인 앞 메시지 점들을 그대로 앞에 붙이고 메시지 시각을 그 첫 점으로 하면, 받는 순간 이미 궤적 안이라 이어 붙이기가 없다.
늦음 (실시간만): 계산 시작이 늦으면 스탬프를 다음 주기 격자로 미루고 (catch_up), 보낼 때 늦으면 안 보내고 앞 메시지를 계속 (kept).
  둘 다 앞 메시지 점 격자 위의 상태에서 다시 시작해서 JTC 가 실행하는 궤적과 이어진다.

입력성형 [2026-10-14 추가] (rt_shaping, 커널 h, 1 ms 격자 g):
  u = 성형 전 명령 (MPC·명령 궤적이 만든 것), y = 성형된 명령 = 메시지 점 (JTC → R2 → WMX 로 가는 것). y[k] = Σ_i h[i]·u[k − i].
  기억 = 실제로 실행된 u: 지금 실행 중인 메시지의 u 는 다음 메시지가 그 스탬프에 갈아탈 때까지만 실행된다 (sent), 안 보내면 계속 실행
  (kept·catch_up), 다시 시작이면 멈춘 자세 (restart). 그래서 새 메시지의 y 는 앞 메시지의 y 와 스탬프에서 위치·속도·가속이 같고, 실행된
  메시지들을 이어 붙인 y 전체 = 실행된 u 전체의 컨볼루션 (tests/test_shaping.py, python3 -m tools.rt_tools shaping 으로 확인).
  한 세그먼트 (다시 시작부터 다음 다시 시작까지) 안의 격자 칸 번호 k: 스탬프 S = T0 + k·g (T0 = 세그먼트 첫 스탬프).
  성형 없음 (h = [1]) 이면 y = u 이고 메시지 점은 2026-10-14 전 코드와 같다.
"""
import numpy as np

from rt_config import StreamCfg


class Stream:
    def __init__(self, c: StreamCfg, dt: float, q0, h=None, grid_s: float = 0.001, mem: int = 0):
        self.c, self.dt = c, dt
        self.h = np.array([1.0]) if h is None else np.asarray(h, float)
        self.L = len(self.h)
        self.mem = max(self.L - 1, int(mem))            # 기억하는 실행된 u 칸 수 (성형 L − 1 칸, 성형 MPC 는 그보다 조금 더: past_q)
        self.g = grid_s
        self.n_dt = int(round(dt / grid_s))             # 한 주기 칸 수
        assert abs(self.n_dt * grid_s - dt) < 1e-9, f'주기 {dt} 가 격자 {grid_s} 의 배수가 아님'
        # [2026-10-27 R-1] 0 이 아닌 칸만 (지연 칸, 세기) — 성형기 임펄스 몇 개가 1 ms 격자 양옆 칸으로 나뉜 것이라 L (100 칸 넘음) 중 몇 개뿐
        self._taps = [(int(i), float(self.h[i])) for i in np.nonzero(self.h)[0]]
        self.stamp = None           # 이번 주기 스탬프 S (None = 다음 주기에 새로 시작)
        self.k = 0                  # S 의 격자 칸 번호 (세그먼트 안)
        self.T0 = None              # 세그먼트 첫 스탬프 (칸 0 의 절대 시각)
        self.pts = []               # 마지막으로 보낸 메시지의 점들 (y) [(절대 시각, q, v, a)]
        self.cmd = None             # 스탬프 시각의 성형 전 명령 상태 (q, v, a) — 다음 명령 궤적의 시작
        self.tail_end = None        # 마지막 메시지 꼬리 끝 = 메시지가 더 안 오면 로봇이 멈출 자리 (y 가 멈추는 자리 = u 가 멈춘 자리)
        self.tail_done = 0.0        # 그 시각 (y 가 멈추는 시각: u 가 멈춘 뒤 성형 길이만큼 더)
        self.msg = None             # 실행 중인 메시지 {'s': 시작 칸, 'u': [M, 3, J], 'y': [M + L − 1, 3, J]}
        self.hist = None            # 실행 중인 메시지 시작 칸 앞의 실행된 u (마지막 L − 1 칸) [L − 1, 3, J]
        self.segments = []          # 실행된 u 기록: [{'T0', 'before': 시작 전 자세, 'chunks': [(시작 칸, q [n, J])]}]
        self._seg = None            # 지금 세그먼트 (segments 에 넣기 전) — restart 가 만듦
        self.restart(q0)

    # ---------------- 세그먼트 ----------------
    def restart(self, q):
        """정지 상태 q 에서 새로 시작 (다음 주기 스탬프 = 그때 시각 + lead_s). 실행 중이던 메시지는 끝까지 실행된 것으로 기록
        (로봇은 그 꼬리를 따라 멈춤). 성형 기억 = 멈춘 자세 (y 도 멈춘 뒤에 부름: Controller 가 tail_done 까지 기다림)."""
        self._close_segment()
        q = np.array(q, float)
        self.cmd = (q.copy(), np.zeros_like(q), np.zeros_like(q))
        self.tail_end = q.copy()
        self.stamp = None
        self.msg = None
        self.hist = np.repeat(self._rest(q)[None], self.mem, axis=0)
        self._seg = {'T0': None, 'before': q.copy(), 'chunks': []}

    def _close_segment(self):
        seg = self._seg
        if seg is None:
            return
        if self.msg is not None:
            seg['chunks'].append((self.msg['s'], self.msg['u'][:, 0].copy()))       # 꼬리까지 끝까지 실행됨
            self.msg = None
        if seg['chunks']:
            self.segments.append(seg)
        self._seg = None

    @staticmethod
    def _rest(q):
        q = np.asarray(q, float)
        return np.stack([q, np.zeros_like(q), np.zeros_like(q)])

    def next_stamp(self, now: float) -> float:
        if self.stamp is None:
            self.stamp = now + self.c.lead_s
            self.k = 0
            self.T0 = self.stamp
            self._seg['T0'] = self.T0
            self.pts = []
        else:
            self.stamp += self.dt
            self.k += self.n_dt
        return self.stamp

    def catch_up(self, clock) -> float:
        """(실시간) 스탬프까지 late_before_s 보다 덜 남았으면 스탬프를 lead_s 이상 남는 주기 격자로 미루고, 명령 상태를 그 시각
        앞 메시지 상태로. 미룬 시간 (s), 안 늦었으면 0. clock = 지금 시각 함수."""
        if not self.pts or self.stamp - clock() >= self.c.late_before_s:
            return 0.0
        s0 = self.stamp
        while self.stamp - clock() < self.c.lead_s:
            self.stamp += self.dt
            self.k += self.n_dt
        self.cmd = self.u_state(self.k)
        return round(self.stamp - s0, 3)

    # ---------------- 실행 중인 명령 ----------------
    def _sample(self, arr, i, rest_from):
        """arr 의 칸 i (넘으면 마지막 값에서 멈춘 상태)."""
        if i < len(arr):
            return arr[i]
        return self._rest(arr[-1, 0] if rest_from is None else rest_from)

    def u_state(self, k: int):
        """칸 k 의 성형 전 명령 (q, v, a) — 실행 중인 메시지 (끝났으면 멈춘 자세)."""
        if self.msg is None:
            x = self.hist[-1] if self.mem > 0 else self._rest(self.tail_end)
        else:
            x = self._sample(self.msg['u'], k - self.msg['s'], None)
        return tuple(np.array(v, float) for v in x)

    def y_state(self, k: int):
        """칸 k 의 성형된 명령 (q, v, a) = 로봇이 따라가는 명령."""
        if self.msg is None:
            return (self.tail_end.copy(), np.zeros_like(self.tail_end), np.zeros_like(self.tail_end))
        return tuple(np.array(v, float) for v in self._sample(self.msg['y'], k - self.msg['s'], None))

    def past(self, k: int = None, n_keep: int = None) -> np.ndarray:
        """칸 k (None = 이번 스탬프 칸) 앞의 실행된 u 마지막 n_keep 칸 (None = 성형 L − 1) [n_keep, 3, J] (오래된 것부터)."""
        k = self.k if k is None else k
        n_keep = self.L - 1 if n_keep is None else n_keep
        if n_keep == 0:
            return np.zeros((0,) + self.hist.shape[1:])
        if self.msg is None:
            return self.hist[-n_keep:]
        n = k - self.msg['s']
        u = self.msg['u']
        ex = u[:n] if n <= len(u) else np.concatenate([u, np.repeat(self._rest(u[-1, 0])[None], n - len(u), axis=0)])
        return np.concatenate([self.hist, ex])[-n_keep:]

    def past_q(self, n: int) -> np.ndarray:
        """지금 스탬프 앞의 실행된 u 위치 마지막 n 칸 [n, J] (오래된 것부터, 마지막 = 스탬프 1 칸 전). n ≤ mem."""
        assert n <= self.mem, (n, self.mem)
        return self.past(self.k, n)[:, 0]

    def shape(self, u: np.ndarray) -> np.ndarray:
        """스탬프 칸 k 부터의 후보 u [N, 3, J] (u[0] = 지금 명령 상태, u[-1] = 멈춘 상태) → y [N + L − 1, 3, J] (마지막 = 멈춘 y).
        과거 = 실제로 실행된 u (기억), u 끝 뒤 = 멈춘 자세."""
        u = np.asarray(u, float)
        if self.L == 1:
            return u.copy()
        ext = np.concatenate([self.past(self.k), u, np.repeat(u[-1:], self.L - 1, axis=0)])
        # y[w] = Σ_i h[i]·ext[w + L − 1 − i] — [2026-10-27 R-1] 0 이 아닌 칸만 더함 (예전: L 칸 창 전체 행렬곱 — 더하는 차례가 달라 값은
        #  반올림 수준 (1e-16) 만 다름, tests/test_shaping.py 가 전체 컨볼루션과 1e-12 안을 봄)
        n, L1 = len(u) + self.L - 1, self.L - 1
        y = np.zeros((n,) + u.shape[1:])
        for i, w in self._taps:
            y += w * ext[L1 - i:L1 - i + n]
        return y

    def points(self, y: np.ndarray, step_n: int, main_n: int, tail_n: int):
        """y 칸들 → 메시지 점 [(스탬프 기준 s, q, v, a)]: 0, step, …, main (명령 궤적), 그 뒤 tail 간격 (멈춤 꼬리), 마지막 점 = 멈춘 자리
        (y 가 멈추는 칸과 main + tail 중 늦은 쪽, 속도·가속 0). 2026-10-14 전 braking_tail 과 같은 규칙 (성형 없음이면 같은 점)."""
        g = self.g
        settle = len(y) - 1
        idx = list(range(0, main_n + 1, step_n))
        idx += list(range(main_n + tail_n, settle, tail_n))
        out = [(round(i * g, 6), y[i, 0], y[i, 1], y[i, 2]) for i in idx]
        last = max(settle, main_n + tail_n)
        q = y[settle, 0]
        out.append((round(last * g, 6), q, np.zeros_like(q), np.zeros_like(q)))
        return out

    def remaining(self):
        """앞 메시지의 이번 스탬프 뒤 점들 (안전 검사의 '앞 메시지 계속' 후보)."""
        return [p for p in self.pts if p[0] >= self.stamp - 1e-6]

    def message(self, pts):
        """pts = [(스탬프 기준 시각, q, v, a)] → (메시지 점 [(메시지 시각 기준, q, v, a)], 메시지 시각, 앞 메시지 점 수)."""
        S = self.stamp
        pre = [p for p in self.pts if S - self.c.pre_s - 1e-9 <= p[0] < S - 1e-6]
        t0 = pre[0][0] if pre else S
        msg = [(tt - t0, q, v, a) for tt, q, v, a in pre] + [(S + tt - t0, q, v, a) for tt, q, v, a in pts]
        return msg, t0, len(pre)

    def sent(self, pts, cmd_next, u=None, y=None):
        """pts (y 점) 를 보냈음: 앞 메시지로 기억, 다음 주기 명령 상태 = cmd_next (다음 스탬프 시각의 u). u·y = 이번 메시지의 1 ms 칸
        (shape 의 입력·출력) — 앞 메시지는 이 스탬프까지만 실행된 것으로 기록하고 이 메시지를 실행 중으로."""
        S, k = self.stamp, self.k
        if u is not None:
            self.finish_at(k)
            self.msg = {'s': k, 'u': np.asarray(u, float), 'y': np.asarray(y, float)}
        self.cmd = tuple(np.array(x, float) for x in cmd_next)
        self.pts = [(S + tt, q, v, a) for tt, q, v, a in pts]
        self.tail_end, self.tail_done = np.array(pts[-1][1], float), S + pts[-1][0]

    def finish_at(self, k: int) -> np.ndarray:
        """[2026-10-27 R-1] 실행 중인 메시지는 칸 k 앞까지만 실행됨: 그만큼을 실행 기록에 넣고 버림 (메시지가 없으면 기록할 것 없음). 기억 (hist)
        = 칸 k 앞의 실행된 u. 새 메시지를 보낼 때 (sent) 와 GUI 비상정지 (그 스탬프부터 성형을 건너뛴 감속 — 이 기억 밖) 가 씀. 반환 = 그 기억."""
        past = self.past(k, self.mem)
        if self.msg is not None:
            n = k - self.msg['s']
            uu = self.msg['u'][:, 0]
            ex = uu[:n] if n <= len(uu) else np.concatenate([uu, np.repeat(uu[-1:], n - len(uu), axis=0)])
            self._seg['chunks'].append((self.msg['s'], ex.copy()))
            self.msg = None
        self.hist = past
        return past

    def set_stamp(self, S: float, k: int):
        """[2026-10-27 R-1] 이번 주기 스탬프를 S (세그먼트 칸 k — S = T0 + k·격자) 로 옮김: GUI 비상정지가 보낼 수 있는 첫 격자 스탬프에서
        감속 메시지를 만들 때 (message() 는 이 스탬프 기준)."""
        self.stamp, self.k = S, k

    def kept(self):
        """안 보냈음 (앞 메시지 계속): 다음 주기 명령 상태 = 앞 메시지의 다음 스탬프 시각 상태."""
        self.cmd = self.u_state(self.k + self.n_dt)
        return self.cmd

    def trim_executed(self, keep: int) -> int:
        """[2026-10-28 R-1, rt_server.trim_stream_chunks 를 옮김] 지금 세그먼트의 실행된 u 조각 (보낸 메시지마다 한 주기) 을 마지막 keep 개
        (≥ 1) 만 남김 — 오래 켜 둔 서버 (실시간 회피로 지키면 세그먼트가 안 닫혀 한 시간 ~180 MB). 앞 조각을 버리고 칸 번호를 남은 첫
        조각 = 칸 0 으로 다시 매김: 세그먼트 첫 스탬프 (T0)·지금 스탬프 칸 (k)·실행 중 메시지 시작 칸도 같이 옮겨서 executed 의 '칸 0 부터
        끊김 없이' 가 그대로이고 메시지 계산 (칸 차이만 씀) 도 같음 (tests/test_stream_trim.py). 버린 구간 마지막 u = 'before' (기록의
        시작 전 자세). 반환: 버린 조각 수."""
        assert keep >= 1, keep
        seg = self._seg
        if seg is None or len(seg['chunks']) <= keep:
            return 0
        ch = seg['chunks']
        j = len(ch) - keep
        cut = int(ch[j][0])
        seg['before'] = np.array(ch[j - 1][1][-1], float)
        ch[:] = [(s - cut, q) for s, q in ch[j:]]
        seg['T0'] += cut * self.g
        self.T0 = seg['T0']
        self.k -= cut
        if self.msg is not None:
            self.msg['s'] -= cut
        return j

    def executed(self):
        """실행된 u 기록: 세그먼트마다 {'T0': 칸 0 의 절대 시각, 'before': 시작 전 자세, 'q': 칸 0 부터 이어진 q [n, J]}.
        마지막 메시지는 꼬리 끝까지 실행된 것으로 (로봇은 그 꼬리로 멈춤). 실행이 끝난 뒤 (저장할 때) 한 번 부른다."""
        self._close_segment()
        out = []
        for seg in self.segments:
            chunks = sorted(seg['chunks'], key=lambda c: c[0])
            ends = [s + len(q) for s, q in chunks]
            assert chunks[0][0] == 0 and all(e == s for e, (s, _) in zip(ends, chunks[1:])), '실행 기록 칸이 끊김'
            out.append({'T0': seg['T0'], 'before': seg['before'], 'q': np.concatenate([q for _, q in chunks])})
        return out
