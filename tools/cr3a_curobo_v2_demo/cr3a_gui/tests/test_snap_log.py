"""[2026-10-28 D-4] snap_log (이미지 기록) 단위 시험 — 가짜 카메라·구로: 팔 지우기·팔 옆 점 분류가 직접 계산과 같은지, 지도에서 빼는 점
(바닥 띠·받침 둘레) 은 빨강에서 빠지는지, 트리거 간격·횟수, 파일 (16 비트 깊이 그대로·컬러·영상 JSON·겹친 그림·snap.json), 넘기기가
제어 스레드에서 짧은지, 분석 프로세스 (snap_log.py --analyze) 결과. numpy·cv2 (Thor 컨테이너 — PC 는 cv2 가 없으면 건너뜀)."""
import json
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
cv2 = pytest.importorskip('cv2')
import snap_log as sl  # noqa: E402

W, H = 640, 480
K = np.array([[400.0, 0, 320], [0, 400.0, 240], [0, 0, 1]])
C0, R0 = np.array([0.0, 0.0, 1.0]), 0.1         # 구 하나 (카메라 = 받침 좌표, 앞 1 m)
PATCH = (240, 330, 10)                           # 팔 옆 조각 (가운데 v, u, 크기 px)


def scene():
    """벽 (z 2 m) + 구 앞면 + 구 오른쪽 옆 조각 (z 1 m, 구 겉면에서 ~5 cm)."""
    v, u = np.mgrid[0:H, 0:W]
    x, y = (u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1]
    n2 = x * x + y * y + 1
    b = C0[2]                                    # 광선 t·(x, y, 1) 와 구: t² n2 − 2 t b + b² − r² = 0
    disc = b * b - n2 * (b * b - R0 * R0)
    t = np.where(disc > 0, (b - np.sqrt(np.maximum(disc, 0))) / n2, np.inf)
    z = np.where(np.isfinite(t), t, 2.0)
    D = np.round(z * 1000).astype(np.uint16)
    pv, pu, s = PATCH
    D[pv - s // 2:pv + s // 2, pu + 48:pu + 48 + s] = 1000      # u 378~387 → x 0.145~0.17 m → 구 겉면 4.5~7 cm
    return D


def brute(D):
    v, u = np.nonzero(D)
    z = D[v, u] / 1000.0
    P = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)
    return v, u, P, np.linalg.norm(P - C0, axis=1) - R0


def test_analyze_matches_bruteforce():
    D = scene()
    a = sl.analyze(D, K, np.eye(4), C0[None], np.array([R0]), mask_m=0.04, near_m=0.06)
    v, u, P, d = brute(D)
    # (분석은 float32 — 문턱에서 1e-5 m 안 점은 어느 쪽이든 됨)
    edge = (np.abs(d - 0.04) < 1e-5) | (np.abs(d - 0.06) < 1e-5)
    assert abs(a['n']['masked'] - int((d < 0.04).sum())) <= int(edge.sum()) and a['n']['masked'] > 1000
    want = (d >= 0.04) & (d < 0.06)
    got = set(map(tuple, a['near_px']))
    assert len(got) > 0 and got ^ set(zip(v[want], u[want])) <= set(zip(v[edge], u[edge]))
    assert np.all(np.diff(a['near_d']) >= 0)                     # 가까운 차례
    assert np.all((a['near_d'] > 0.04 - 1e-5) & (a['near_d'] < 0.06 + 1e-5))


def test_excluded_points_not_red():
    D = scene()
    full = sl.analyze(D, K, np.eye(4), C0[None], np.array([R0]), mask_m=0.04, near_m=0.06)
    # 바닥 띠: z (받침 좌표 = 카메라 z) 가 1.5 m 아래 점은 지도에서 뺌 → 팔 옆 점 (z ~1 m) 모두 빨강에서 빠짐
    fl = sl.analyze(D, K, np.eye(4), C0[None], np.array([R0]), mask_m=0.04, near_m=0.06, floor_z=1.5)
    assert fl['n']['near'] == 0 and fl['n']['excluded_near'] == full['n']['near']
    ring = sl.analyze(D, K, np.eye(4), C0[None], np.array([R0]), mask_m=0.04, near_m=0.06, base_ring=(5.0, 5.0))
    assert ring['n']['near'] == 0
    assert fl['n']['masked'] == full['n']['masked']              # 팔 지우기는 그대로


def test_sphere_dist_chunks():
    rng = np.random.default_rng(0)
    P = rng.uniform(-1, 1, (5003, 3)).astype(np.float32)
    c, r = rng.uniform(-1, 1, (7, 3)), rng.uniform(0.01, 0.2, 7)
    d, k = sl.sphere_dist(P, c, r, chunk=1000)
    Dm = np.linalg.norm(P[:, None] - c[None], axis=2) - r
    assert np.allclose(d, Dm.min(1), atol=1e-5) and np.array_equal(k, Dm.argmin(1))


def test_project_identity():
    uv, front = sl.project(np.array([[0.1, -0.05, 1.0], [0, 0, -1.0]]), np.eye(4), K)
    assert np.allclose(uv[0], [360, 220]) and front.tolist() == [True, False]


def test_reason_of():
    assert sl.reason_of('반복 1: → 목표 막혀 서 있는데 물체가 다가옴 (MPC 지도 -25 mm) → 그 자리에서 비킴') == 'yield'
    assert sl.reason_of('이동 경로 앞 막힘 → 멈추지 않고 다시 계획') == 'replan'
    assert sl.reason_of('실시간 회피 켜기 전 지도 점검 걸림: …') == 'mapcheck'
    assert sl.reason_of('지도 점검 통과: …') is None
    assert sl.reason_of('이동 새 경로로 (멈추지 않음, 계획 0.2초)') is None


class FakeFeed:
    color_src = 'feed'

    def __init__(self, t0, n=30, fps=15.0):
        D = scene()
        img = np.full((H, W, 3), 90, np.uint8)
        self.depth = [(i + 1, t0 + i / fps, t0 + i / fps + 0.03, D) for i in range(n)]
        self.colors = [(t0 + i / fps + 0.02, (t0 + i / fps + 0.004, img)) for i in range(n)]

    def depth_between(self, a, b):
        return [f for f in self.depth if a <= f[1] <= b]

    def color_between(self, a, b):
        return [c for c in self.colors if a <= c[0] <= b]


def make_log(tmp_path, feed, lines, **kw):
    cam = {'K': K, 'T_base_depth': np.eye(4), 'T_base_color': np.eye(4), 'Kc': K, 'Dc': None}
    near = {'q': np.zeros(6), 'raw': 0.03, 'mpc': -0.012, 'B': np.array([[0.13, 0.0, 1.0], [0.0, 0.3, 1.0]]), 'd_b': 0.03, 'link': 'L1'}
    return sl.SnapLog(feed, tmp_path / 'snap', cam, mask_m=0.04, near_m=0.06, floor_z=-1.0, base_exclude=None, base_ring=None,
                      link_names=['L0', 'L1'], q_fn=lambda st: np.zeros(6),
                      spheres_fn=lambda Q: (np.repeat([[[9.0, 9, 9], C0]], len(Q), 0), np.repeat([[0.05, R0]], len(Q), 0)),
                      frame_info_fn=lambda st: [{'near_mm': -12.0}] * len(st), near_fn=lambda: dict(near),
                      state_fn=lambda: {'realtime': True}, speck=(50, 40), log=lines.append, rec_list=[], **kw)


def test_snaplog_files(tmp_path):
    t_trig = time.time() - 2.0                       # 뒤 영상이 이미 다 온 순간
    feed = FakeFeed(t_trig - 1.0)
    lines = []
    s = make_log(tmp_path, feed, lines)
    assert s.trigger('yield', '막혀 서 있는데 물체가 다가옴', t=t_trig, clr_mpc_mm=-25.0)
    assert not s.trigger('replan', 'x', t=t_trig + 0.5)              # 1 s 안 → 안 받음
    s.poll()                                        # 1: 그 순간 지도 칸
    assert s._pending and s._pending[0]['near'] is not None
    s.poll()                                        # 2: 넘김
    assert not s._pending and len(s.rec_list) == 1
    s.close()
    d = tmp_path / 'snap' / s.rec_list[0]['dir'].split('/')[1]
    depths = sorted(d.glob('depth_*.png'))
    assert 14 <= len(depths) <= 16                  # 앞뒤 0.5 s × 15 fps
    back = cv2.imread(str(depths[0]), cv2.IMREAD_UNCHANGED)
    assert back.dtype == np.uint16 and np.array_equal(back, scene())
    assert len(list(d.glob('color_*.jpg'))) >= len(depths) and (d / 'overlay.jpg').exists()
    fj = json.loads((d / 'frame_00.json').read_text(encoding='utf-8'))
    assert fj['counts']['near'] > 0 and fj['near_pts'][0][4] == 'L1' and fj['map']['near_mm'] == -12.0
    assert 40 <= fj['min_d_mm'] <= 60 and list(fj['near_by_link']) == ['L1']
    sj = json.loads((d / 'snap.json').read_text(encoding='utf-8'))
    assert sj['reason'] == 'yield' and sj['info']['clr_mpc_mm'] == -25.0 and sj['map_near']['n_vox'] == 2
    assert sj['map_vox'][0][4] == 'L1' and abs(sj['map_vox'][0][3] - 30.0) < 0.5      # (0.13, 0, 1) 은 구 겉면 3 cm
    assert any('이미지 기록 [yield]' in x for x in lines)
    ov = cv2.imread(str(d / 'overlay.jpg'))
    pv, pu, sz = PATCH
    assert ov[pv, pu + 53, 2] > 200 and ov[pv, pu + 53, 0] < 60   # 조각 = 빨강
    assert ov[240, 320, 0] > ov[240, 320, 2]                                           # 구 가운데 = 파랑 섞임


def test_snaplog_cap_and_waits_for_post(tmp_path):
    feed = FakeFeed(time.time() - 1.0)
    lines = []
    s = make_log(tmp_path, feed, lines)
    t = time.time()
    assert s.trigger('near', 'x', t=t)
    s.poll()
    s.poll()
    assert s._pending                               # 뒤 0.5 s 가 아직 → 안 넘김
    s.n_taken = sl.SNAP_MAX
    assert not s.trigger('near', 'y', t=t + 5) and any(f'{sl.SNAP_MAX} 번' in x for x in lines)
    s.close()                                       # 끝: 있는 영상으로 넘김
    assert not s._pending and s.n_written == 1


def test_poll_is_cheap(tmp_path):
    """넘기기 (제어 스레드) 는 파일·분석 없이 — 쓰기 스레드가 함."""
    feed = FakeFeed(time.time() - 3.0)
    s = make_log(tmp_path, feed, [])
    s.trigger('guard', 'g', t=time.time() - 2.0)
    s.poll()
    s.poll()
    assert max(s.poll_ms) < 20.0, s.poll_ms
    s.close()


def test_color_ring_pick():
    """일꾼: 깊이 받은 시각마다 가장 가까운 컬러, 겹치면 한 번."""
    import collections
    import threading
    r = sl.ColorRing.__new__(sl.ColorRing)
    r.lock, r.ring = threading.Lock(), collections.deque([(10.00, b'a'), (10.07, b'b'), (10.13, b'c')])
    assert [x[1] for x in r.pick([10.01, 10.02, 10.12, 10.5])] == [b'a', b'c']


def test_sphere_dist_near_matches_inside_reach():
    rng = np.random.default_rng(1)
    P = rng.uniform(-0.5, 0.5, (20000, 3)).astype(np.float32)
    c, r = rng.uniform(-0.4, 0.4, (30, 3)), rng.uniform(0.02, 0.08, 30)
    d0, k0 = sl.sphere_dist(P, c, r)
    d1, k1 = sl.sphere_dist_near(P, c, r, 0.06)
    inside = d0 < 0.06 - 1e-5
    assert inside.sum() > 1000
    assert np.allclose(d1[inside], d0[inside], atol=1e-5) and np.array_equal(k1[inside], k0[inside])
    assert np.all(np.isinf(d1[d0 > 0.06 + 1e-5]))


def test_near_threshold_margin(monkeypatch):
    """near 기준 = act_new − 5 mm: 아무것도 없을 때의 59.7 mm 는 안 걸림 (2026-10-28 실물), 시험 환경 변수는 그대로."""
    monkeypatch.delenv('CR3A_SNAP_NEAR_MM', raising=False)
    assert abs(sl.env_near_m(0.06) - 0.055) < 1e-9 and not (59.7 < sl.env_near_m(0.06) * 1000)
    monkeypatch.setenv('CR3A_SNAP_NEAR_MM', '2000')
    assert sl.env_near_m(0.06) == 2.0
