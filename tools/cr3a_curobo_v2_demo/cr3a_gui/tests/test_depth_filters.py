"""[2026-10-28 D-4] depth_filters (가림 띠 지우기 B · 시간 확인 A) 단위 시험 — 가짜 깊이 (torch CPU). Thor 컨테이너 (torch 있음)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
torch = pytest.importorskip('torch')
import depth_filters as dfl  # noqa: E402

FX = 400.0


def scene():
    """줄마다: 배경 1.5 m, 로봇 300~339 열 0.6 m. 띠 폭 (배경 무한) = 0.095 × 400 / 0.6 = 63 px → 로봇 왼쪽 237~299."""
    d = torch.full((10, 640), 1.5)
    robot = torch.zeros((10, 640), dtype=torch.bool)
    d[:, 300:340] = 0.6
    robot[:, 300:340] = True
    return d, robot


def test_band_same_depth_left_only():
    d, robot = scene()
    d[0, 280:300] = 0.0                 # 띠 = 깊이 없음
    d[0, 270:280] = 0.61                # 띠 안 로봇 깊이 섬 (유령) → 지움
    d[1, 230] = 0.6                     # 띠 밖 (70 px) 같은 깊이 → 그대로
    d[2, 350] = 0.6                     # 로봇 오른쪽 같은 깊이 → side left 면 그대로
    d[3, 290] = 0.70                    # 띠 안이지만 깊이 +10 cm → same 은 그대로, behind 는 지움
    rm = dfl.shadow_band(d, robot, FX, 'left')
    assert rm[0, 270:280].all() and not rm[0, 260] and not rm[1, 230] and not rm[2, 350] and not rm[3, 290]
    assert not rm[robot].any()          # 로봇 픽셀은 안 건드림
    assert not rm[4, 250]               # 띠 안 배경 1.5 m (same) 는 그대로
    rb = dfl.shadow_band(d, robot, FX, 'left', behind=True)
    assert rb[3, 290] and rb[4, 250] and rb[0, 270:280].all() and not rb[1, 230]
    assert dfl.shadow_band(d, robot, FX, 'right')[2, 350]


def test_band_width_scales_with_depth():
    d, robot = scene()
    d[:, 300:340] = 1.2                 # 더 먼 로봇 → 띠 폭 0.095 × 400 / 1.2 = 31.7 px
    d[0, 300 - 31] = 1.2
    d[0, 300 - 34] = 1.2
    rm = dfl.shadow_band(d, robot, FX, 'left')
    assert rm[0, 300 - 31] and not rm[0, 300 - 34]


def test_temporal_valid_count():
    st = torch.zeros((4, 1, 4))
    st[:, 0, 0] = torch.tensor([0, 0, 0, 0.6])        # 지금만 → 지움 (유령 섬: 깊이 있음 20~46 %)
    st[:, 0, 1] = torch.tensor([0.6, 0, 0.6, 0.6])     # 4 장 중 3 장 → 남김
    st[:, 0, 2] = torch.tensor([1.10, 1.067, 1.033, 1.0])   # 0.5 m/s 로 다가옴: 깊이 있음만 (A') 은 남김, ±20 mm (A) 는 지움
    st[:, 0, 3] = torch.tensor([0.6, 0.6, 0.6, 0.0])   # 지금 깊이 없음 → 없음
    a = dfl.temporal_keep(st, 3, None)[0]
    assert a.tolist() == [False, True, True, False]
    s = dfl.temporal_keep(st, 3, 0.02)[0]
    assert s.tolist() == [False, True, False, False]


def test_temporal_filter_missing_frames():
    tf = dfl.TemporalFilter(3, 4)
    one = torch.full((2, 2), 0.8)
    tf.push(1, one)
    tf.push(2, one)
    assert tf.keep(3, one).all()                       # 1·2·3 = 3 장
    assert not tf.keep(7, one).any()                   # 4·5·6 빠짐 → 1 장뿐
    tf.push(8, one)
    tf.push(9, one)
    assert tf.keep(10, one).all() and set(tf.hist) == {7, 8, 9, 10}


def test_settings_env(monkeypatch):
    for k in ('CR3A_DEPTH_TEMPORAL', 'CR3A_DEPTH_TEMPORAL_TOL_MM', 'CR3A_DEPTH_BAND', 'CR3A_DEPTH_BAND_SIDE'):
        monkeypatch.delenv(k, raising=False)
    t, b = dfl.settings_from_env()
    assert t == (*dfl.TEMPORAL_KN, dfl.TEMPORAL_TOL_M) and b == (dfl.BAND_SIDE, dfl.BAND_TOL_M, dfl.BAND_MODE)
    monkeypatch.setenv('CR3A_DEPTH_TEMPORAL', '0')
    monkeypatch.setenv('CR3A_DEPTH_BAND', '0')
    assert dfl.settings_from_env() == (None, None)
    monkeypatch.setenv('CR3A_DEPTH_TEMPORAL', '3,4')
    monkeypatch.setenv('CR3A_DEPTH_TEMPORAL_TOL_MM', '20')
    monkeypatch.setenv('CR3A_DEPTH_BAND', 'behind')
    assert dfl.settings_from_env() == ((3, 4, 0.02), (dfl.BAND_SIDE, dfl.BAND_TOL_M, 'behind'))
    assert dfl.band_kw('behind_adaptive') == {'behind': True, 'adaptive': True, 'ref_rows': 0}
    assert dfl.band_kw('behind_adaptive_2d') == {'behind': True, 'adaptive': True, 'ref_rows': dfl.BAND_REF_ROWS}
    monkeypatch.setenv('CR3A_DEPTH_BAND_ROWS', '15')
    assert dfl.band_kw('behind_adaptive_2d')['ref_rows'] == 15
    monkeypatch.setenv('CR3A_DEPTH_BAND', 'bogus')
    with pytest.raises(AssertionError):
        dfl.settings_from_env()


def test_band_adaptive_width():
    """적응 폭: 로봇 (0.6 m) 바로 뒤 물체 (0.65 m) 가 가장 넓은 띠 밖까지 이어지면 띠가 몇 픽셀로 줄어 물체가 남고, 배경이 멀면 (1.9 m)
    계산 폭 (0.095·400·(1/0.6 − 1/1.9) = 43 px) + 여유 (3 + 6.7 px), 깊이 없는 배경이면 가장 넓은 띠 (63 px)."""
    d, robot = scene()
    d[0, 180:300] = 0.65                 # 줄 0: 로봇 바로 뒤 물체 (가장 넓은 띠 237~299 밖까지)
    d[1, :300] = 1.9                     # 줄 1: 먼 벽
    d[1, 250:256] = 0.62                 # 띠 안 섬 (로봇에서 44~50 px) — 계산 폭 43 + 여유 9.7 = 52.7 px 안 → 지움
    d[2, :236] = 0.0                     # 줄 2: 배경 깊이 없음 → 가장 넓은 띠
    d[2, 240:243] = 0.62                 # 로봇에서 57~60 px → 가장 넓은 띠 (63) 안 → 지움
    ra = dfl.shadow_band(d, robot, FX, 'left', behind=True, adaptive=True)
    rp = dfl.shadow_band(d, robot, FX, 'left', behind=True)
    assert rp[0, 240:300].all() and not ra[0, 240:285].any() and ra[0, 287:300].all()   # 물체: B+ 는 63 px 지우고 적응은 ~14 px (계산 5 + 여유 9.7)
    assert ra[1, 250:256].all() and not ra[1, 240:245].any() and rp[1, 240:245].all()    # 띠 안 섬은 지움, 계산 폭 밖 벽은 남김
    assert ra[2, 240:243].all()
    ro = dfl.shadow_band(torch.flip(d, [1]), torch.flip(robot, [1]), FX, 'right', behind=True, adaptive=True)
    assert torch.equal(torch.flip(ro, [1]), ra)                                          # 오른쪽 = 뒤집은 왼쪽


def test_band_2d_reference():
    """2D 기준: 띠 안 섬 (0.55 m) 이 같은 줄 가장자리 (0.62 m) 보다 앞이라 B+ 는 남기지만, 위 20 줄 오른쪽에 더 가까운 로봇 부분
    (0.545 m) 이 있으면 지움 (ref_rows 30) · 15 줄이면 안 닿아 남김. 그 가까운 부분보다 앞 (0.50 m) 실제 물체는 남김."""
    d = torch.full((60, 640), 1.1)
    robot = torch.zeros((60, 640), dtype=torch.bool)
    d[:, 300:340] = 0.62
    robot[:, 300:340] = True
    d[10:15, 290:300] = 0.545           # 위쪽 줄의 더 가까운 로봇 부분 (가장자리 290)
    robot[10:15, 290:300] = True
    d[34, 270:276] = 0.55               # 줄 34 띠 안 섬 (같은 줄 가장자리 300 에서 24~30 px)
    d[36, 270:276] = 0.50               # 줄 36: 그 가까운 부분보다 앞인 실제 물체
    a = dfl.shadow_band(d, robot, FX, 'left', behind=True, adaptive=True)
    b30 = dfl.shadow_band(d, robot, FX, 'left', behind=True, adaptive=True, ref_rows=30)
    b15 = dfl.shadow_band(d, robot, FX, 'left', behind=True, adaptive=True, ref_rows=15)
    assert not a[34, 270:276].any() and b30[34, 270:276].all() and not b15[34, 270:276].any()
    assert not b30[36, 270:276].any()
    assert not (b30 & ~a)[50].any()     # 가까운 부분에서 먼 줄 (40 줄 아래) 은 B+ 적응과 같음
