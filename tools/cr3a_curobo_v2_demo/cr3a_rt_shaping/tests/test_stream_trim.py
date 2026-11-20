"""[2026-10-28 R-1] Stream.trim_executed (rt_server.trim_stream_chunks 를 엔진으로 옮김): 지금 세그먼트의 실행된 u 조각을 줄여도
(1) 메시지 계산 (성형 y·멈춤 상태·앞 u) 이 안 줄인 것과 똑같고 (2) executed 가 '칸 0 부터 끊김 없이' 를 지키며 남긴 구간의 u 가 같고
(3) 시각 (T0 + 칸 × 격자) 도 같음. numpy 만 (cr3a_gui/tests/test_srv_trim_stream.py 와 같은 시험).
"""
import copy

import numpy as np

from rt_config import StreamCfg
from rt_stream import Stream

J, DT, G = 6, 0.05, 0.001


def drive(st, rng_seed, n_cycles, trim_every=None, keep=7, kept_every=5):
    """주기마다 무작위 매끄러운 u 를 보내고 (sent), 가끔 안 보냄 (kept). trim_every 주기마다 줄임. 반환: 주기마다 (shape 결과, cmd, y_state, 앞 u)."""
    rng = np.random.default_rng(rng_seed)
    out, now = [], 100.0
    n_main = int(round(2 * DT / G))
    for c in range(n_cycles):
        S = st.next_stamp(now)
        now = S + DT - st.c.lead_s
        q0, v0, a0 = st.cmd
        t = np.arange(n_main + 20)[:, None] * G
        acc = rng.normal(0, 0.5, J)
        u = np.stack([q0 + v0 * t + 0.5 * acc * t * t, np.broadcast_to(v0 + acc * t, (len(t), J)),
                      np.broadcast_to(acc, (len(t), J))], 1)
        u[0] = np.stack([q0, v0, a0])
        y = st.shape(u)
        res = (y.copy(), tuple(x.copy() for x in st.cmd), st.y_state(st.k + st.n_dt), st.past(st.k).copy())
        if c % kept_every == kept_every - 1 and st.pts:
            st.kept()
        else:
            pts = st.points(y, 5, n_main, 10)
            st.sent(pts, (u[st.n_dt, 0], u[st.n_dt, 1], u[st.n_dt, 2]), u, y)
        if trim_every and c % trim_every == trim_every - 1:
            st.trim_executed(keep)
        out.append(res)
    return out


def make_stream():
    h = np.ones(30) / 30.0                      # 성형 길이 30 칸 (기억이 있어야 줄이기가 메시지에 안 닿는지 볼 수 있음)
    return Stream(StreamCfg(), DT, np.zeros(J), h=h, grid_s=G)


def test_trim_keeps_messages_identical_and_executed_valid():
    a, b = make_stream(), make_stream()
    ra = drive(a, 1, 120)
    rb = drive(b, 1, 120, trim_every=10, keep=7)
    for (ya, ca, sa, pa), (yb, cb, sb, pb) in zip(ra, rb):
        assert np.array_equal(ya, yb)
        assert all(np.array_equal(x, z) for x, z in zip(ca, cb))
        assert all(np.array_equal(x, z) for x, z in zip(sa, sb))
        assert np.array_equal(pa, pb)
    assert len(b._seg['chunks']) <= 7 + 10      # 줄인 뒤 다음 줄이기까지 더 쌓인 것까지
    ea, eb = copy.deepcopy(a).executed(), copy.deepcopy(b).executed()     # (executed 는 세그먼트를 닫음 — 사본으로)
    assert len(ea) == len(eb) == 1
    qa, qb = ea[0]['q'], eb[0]['q']
    n = len(qb)
    assert 0 < n < len(qa)
    assert np.array_equal(qa[-n:], qb)                                  # 남긴 구간의 u 가 같음
    assert abs((eb[0]['T0'] - ea[0]['T0']) - (len(qa) - n) * G) < 1e-9   # 같은 칸 = 같은 시각
    assert np.array_equal(eb[0]['before'], qa[len(qa) - n - 1])         # 시작 전 자세 = 버린 구간의 마지막 u


def test_trim_noop_when_short():
    st = make_stream()
    drive(st, 2, 5)
    before = list(st._seg['chunks'])
    assert st.trim_executed(100) == 0
    assert st._seg['chunks'] == before


def test_trim_then_restart_and_save():
    """줄인 세그먼트를 닫고 (다시 시작) 새 세그먼트로 이어도 executed 가 됨."""
    st = make_stream()
    drive(st, 3, 40, trim_every=8, keep=5)
    st.restart(st.tail_end)
    drive(st, 4, 12)
    segs = st.executed()
    assert len(segs) == 2 and all(len(s['q']) > 0 for s in segs)
