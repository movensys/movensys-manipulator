"""[2026-10-27 R-1] cr3a_mapper.min_dist (TF32 와 상관없는 최소 거리) — float64 기준과 0.01 mm 안, 거리 0 근처도.
cuRobo 를 불러온 뒤 (행렬곱 TF32 켜짐) 에도. GPU·cuRobo 가 있어야 해서 컨테이너에서만 (없으면 건너뜀).
   python3 -m pytest   (cr3a_rt_shaping 에서 — pytest.ini)
"""
import pytest


def test_min_dist_matches_float64_near_zero():
    torch = pytest.importorskip('torch')
    cmap = pytest.importorskip('cr3a_mapper')               # cuRobo 를 불러옴 → torch.backends.cuda.matmul.allow_tf32 = True
    if not torch.cuda.is_available():
        pytest.skip('GPU 없음')
    g = torch.Generator(device='cuda').manual_seed(3)
    # 지도 칸 중심 (받침 기준 m, 지도 범위) 과 그 가까이의 구 / 점 — 거리 0~10 cm
    a = torch.rand((70000, 3), device='cuda', generator=g) * torch.tensor([1.3, 1.3, 0.8], device='cuda') + torch.tensor([-0.45, -0.65, -0.02], device='cuda')
    b = a[torch.randint(0, a.shape[0], (400,), device='cuda', generator=g)] + (torch.rand((400, 3), device='cuda', generator=g) - 0.5) * 0.02
    r = torch.rand(400, device='cuda', generator=g) * 0.05
    for rr in (None, r):
        got = cmap.min_dist(a, b, rr).double()
        ref = torch.cdist(a.double(), b.double(), compute_mode='donot_use_mm_for_euclid_dist')
        ref = (ref if rr is None else ref - rr.double()).min(dim=1).values
        err = (got - ref).abs()
        near = ref.abs() < 0.01
        assert bool(near.any())
        assert float(err.max()) < 1e-5, float(err.max())        # 0.01 mm (float32 차의 제곱합)


def test_min_dist_return_index():
    torch = pytest.importorskip('torch')
    cmap = pytest.importorskip('cr3a_mapper')
    if not torch.cuda.is_available():
        pytest.skip('GPU 없음')
    g = torch.Generator(device='cuda').manual_seed(5)
    a = torch.rand((3000, 3), device='cuda', generator=g)
    b = torch.rand((52, 3), device='cuda', generator=g)
    r = torch.rand(52, device='cuda', generator=g) * 0.05
    d, i = cmap.min_dist(a, b, r, chunk=1000, return_index=True)          # (chunk 를 나눠도 번호가 맞게)
    assert torch.equal(d, cmap.min_dist(a, b, r, chunk=1000))
    assert torch.equal(d, cmap.min_dist(a, b, r))                           # 나누는 줄 수와 상관없이 같은 값 (줄마다 따로 계산)
    at = (a.double() - b.double()[i]).norm(dim=1) - r.double()[i]        # 돌려준 구까지 거리 = 최소 거리
    assert float((at - d.double()).abs().max()) < 1e-5
