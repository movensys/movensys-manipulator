"""[2026-10-28 R-1] GuiController._near_now (실시간 회피 켜기 전 지도 점검) 가 엔진 cr3a_mapper.min_dist 로 바뀐 뒤: 겉면 가까운 칸 B 의
수, 가장 가까운 칸의 구 겉면 거리·링크가 float64 직접 계산 (반지름 0 이하 구는 뺌) 과 같은지. 안 쓰는 구를 빼고 고른 번호를 전체 구 번호
(sphere_link_names 순서) 로 되돌리는 것까지. torch·cuRobo 가 있는 곳 (Thor 컨테이너) 에서만 — PC 는 건너뜀."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[2] / 'cr3a_rt_shaping')]   # 서버 · 엔진
torch = pytest.importorskip('torch')
gc_mod = pytest.importorskip('gui_controller')

VS, MASK = 0.02, 0.04                       # 칸 크기·팔 지우기 거리 (m) — B = 구 겉면에서 MASK + 1.5 칸 안


class FakeWorld:
    """_near_now 가 쓰는 World 부분만: 지도 설정·새 물체 칸·거리·구·구 링크 이름."""
    def __init__(self, P, c, r, names):
        self.cm = SimpleNamespace(s=SimpleNamespace(esdf_voxel_size=VS, segment_distance_threshold=MASK), new_voxel_centers=lambda: P)
        self._c, self._r, self._names = c, r, names

    def clearances(self, Q, kinds, floor=False):
        return {'raw': [0.123], 'mpc': [0.456]}

    def spheres(self, Q):
        return self._c[None], self._r[None]

    def sphere_link_names(self):
        return self._names


@pytest.mark.parametrize('seed', [0, 1, 2])
def test_near_now_matches_float64(seed):
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    g = torch.Generator().manual_seed(seed)
    n_s = 40
    c = torch.rand(n_s, 3, generator=g) * 0.6 - 0.3
    r = torch.rand(n_s, generator=g) * 0.05 + 0.02
    r[[0, 7, 13]] = 0.0                     # 안 쓰는 구 (반지름 0·음수) — 빼야 함
    r[21] = -1.0
    P = torch.rand(600, 3, generator=g) * 0.8 - 0.4
    names = [f'L{k}' for k in range(n_s)]
    fake = SimpleNamespace(W=FakeWorld(P.to(dev), c.to(dev), r.to(dev), names))
    out = gc_mod.GuiController._near_now(fake, np.zeros(6))
    assert (out['raw'], out['mpc']) == (0.123, 0.456)
    # float64 기준: 안 쓰는 구는 무한대
    D = torch.cdist(P.double(), c.double()) - r.double()[None]
    D[:, r <= 0] = float('inf')
    dmin, imin = D.min(dim=1)
    thr = MASK + 1.5 * VS
    n_ref = int((dmin <= thr).sum())
    n_edge = int(((dmin - thr).abs() < 1e-5).sum())         # float32 로 갈릴 수 있는 문턱 바로 옆
    assert n_ref > 0
    assert abs(out['B'].shape[0] - n_ref) <= n_edge
    k = int(torch.argmin(dmin))
    assert abs(out['d_b'] - float(dmin[k])) < 1e-5
    assert out['link'] == names[int(imin[k])]
    assert int(imin[k]) not in (0, 7, 13, 21)


def test_near_now_no_new_voxels():
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    c, r = torch.zeros(3, 3, device=dev), torch.full((3,), 0.05, device=dev)
    fake = SimpleNamespace(W=FakeWorld(torch.zeros(0, 3, device=dev), c, r, ['a', 'b', 'c']))
    out = gc_mod.GuiController._near_now(fake, np.zeros(6))
    assert out['B'].shape == (0, 3) and out['d_b'] is None and out['link'] is None
