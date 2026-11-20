"""CR3A 실시간 회피의 카메라 지도 (cuRoboV2 Mapper, ROS 없음, 카메라 1대).
Mirobot 계획 노드의 지도 모듈(mirobot_cumotion/curobo_v2_mapper.py, 10-08 판)에서 시작해 CR3A 실시간 회피에 쓰는 것만 남겼다
(2026-10-13 정리: 계획기용 세 번째 지도·가상 상자·무시 상자·스테레오 그림자·노드 상태 함수는 뺌. 경위는 Mirobot 작업기록).

영상 한 장:  깊이 → 팔 지우기(RobotSegmenter, 로봇 표면 mask 안) → 깜빡임 거르기 → FilterDepth → TSDF Mapper (mapper_mpc) 에 쌓음
  보이는 모든 것 (바닥 띠 z < mpc_floor_z_m · 받침 자리 · 받침 둘레 케이블 자리는 뺌 — 바닥은 rt_world 가 MPC·계획기에 상자로 따로 줌).
  빈 공간 지우기(carve)·가려진 칸 잊기(forget_occluded)·전체 감쇠 0.7 로 치운 물체가 ~1 초 안에 빠짐.
[2026-10-16 사용자 결정 "정지 및 움직이는 물체 모두 움직이는 물체처럼 … 안 보이는 뒷면 처리도 따로 하지말고 보이는 부분만"]
  정지·새 물체를 나누지 않음: 보이는 모든 칸 = 새 물체 지도 = 실제 거리 지도. 켤 때 1 초 (mpc_background_frames) 는 지도가 차도록 기다리는
  데만. [2026-10-27 R-1] 그 전 방식 (1 초 배경 = 정지 물체, 가려진 뒷면 채우기, 정지 거리 변환, 새 물체 지도 따로) 되돌리기는 뺌.
거리 지도 (esdf_mpc, 영상마다):
  raw  = 지도 ESDF                        — 실제 거리. 안전 검사·계획 검증 (rt_world)
  dyn  = raw 와 같은 값 (이름은 예전 '새 물체' 지도 — 기록 newvox·보고서·도구가 씀)
  MPC  = raw + 경계 채우기 (팔 지우기 사각지대) — MPC 와 계획기가 보는 지도
ESDF 값: 안쪽 음수. 칸 i 중심 = origin + (i + 0.5)·칸, 칸 순서 [x, y, z].
"""
import math
from dataclasses import dataclass
from typing import List, Optional

import torch
import warp as wp

from curobo._src.types.robot import RobotCfg  # 공개 API 에 아직 없음
from curobo._src.util.warp import get_warp_device_stream
from curobo.kinematics import Kinematics
from curobo.perception import FilterDepth, Mapper, MapperCfg, RobotSegmenter
from curobo.scene import VoxelGrid
from curobo.types import CameraObservation, JointState, Pose


FILL_BAND_VOX = 1.5         # 경계 채우기: 로봇 표면에서 mask + 이만큼 (칸) 안의 새 물체 칸 = 지우기 경계에 닿은 칸 B
FILL_BOX_M = 0.15           # 경계 채우기: B 둘레 이 상자 안 칸만 (m)
FILL_MAX_B = 400            # 경계 채우기: B 가 이보다 많으면 고르게 이만큼만 (cdist 크기)
MIN_DIST_PAIRS = 1 << 22    # min_dist: 한 번에 계산하는 (a 점, b 점) 쌍 수 (임시 float32 [줄, m] ~17 MB — GPU 메모리)


def min_dist(a: torch.Tensor, b: torch.Tensor, r: Optional[torch.Tensor] = None, chunk: Optional[int] = None,
             return_index: bool = False):
    """점 a [n, 3] 마다 점 b [m, 3] (r [m] 을 주면 반지름 r 인 구의 겉면) 까지의 최소 거리 [n] (float32).
    return_index 면 (거리 [n], 가장 가까운 b 의 번호 [n] long).
    [2026-10-27 R-1] torch.cdist 는 행이 25 개를 넘으면 행렬곱 (|a|² + |b|² − 2a·b) 으로 계산하는데, cuRobo 를 불러오면 행렬곱이 TF32
    (가수 10 비트) 라 거리 0 근처에서 mm~cm 수준까지 틀림 (Thor 실측 — tf32 재기). 여기서는 좌표마다 차를 제곱해 더함 (float32, 행렬곱
    없음) — 전역 TF32 설정과 상관없음 (계획기 스레드가 cuRobo 를 같이 돌려 전역 설정은 안 바꿈). float64 기준과 0.0001 mm 안.
    [2026-10-28] cdist(compute_mode='donot_use_mm_for_euclid_dist') 는 같은 계산인데 Thor 에서 50~80 배 느렸음 (영상마다 경계 채우기
    +44 ms, plan_prep 'start' +167 ms). 이렇게 하면 예전 행렬곱 길과 비슷 (r1tools/bench_min_dist2 — 경계 채우기 B 거리 0.21 ms ·
    지우기 영역 0.39 ms · plan_prep 3.7 ms, 행렬곱 길 0.22 · 0.35 · 2.5 ms).
    chunk = 한 번에 a 몇 줄 (None = (a, b) 쌍 MIN_DIST_PAIRS 개 — 임시 메모리 [줄, m] 을 m 에 따라 묶음)."""
    n = a.shape[0]
    out = torch.empty(n, device=a.device, dtype=torch.float32)
    idx = torch.empty(n, device=a.device, dtype=torch.long) if return_index else None
    step = chunk or max(1, MIN_DIST_PAIRS // max(b.shape[0], 1))
    bt = b.t().contiguous()                     # [3, m]
    for k in range(0, n, step):
        ak = a[k:k + step]
        d = (ak[:, 0:1] - bt[0]).square_()      # [줄, m] — 좌표마다 차의 제곱 (n·m·3 임시를 안 만듦)
        d += (ak[:, 1:2] - bt[1]).square_()
        d += (ak[:, 2:3] - bt[2]).square_()
        d.sqrt_()
        if r is not None:
            d -= r
        m = d.min(dim=1)
        out[k:k + step] = m.values
        if return_index:
            idx[k:k + step] = m.indices
    return (out, idx) if return_index else out


@wp.kernel
def _carve_free_space(
    block_coords: wp.array(dtype=wp.int32),          # 블록 키 (x, y, z), 격자 가운데 기준
    block_to_hash_slot: wp.array(dtype=wp.int32),    # < 0 = 빈 블록
    num_allocated: wp.array(dtype=wp.int32),
    block_data: wp.array3d(dtype=wp.float16),        # [블록, 칸, (sdf×무게 합, 무게)]
    depth: wp.array2d(dtype=wp.float32),             # 깜빡임만 거른 원래 깊이 (팔 지우기 전, m, 0 = 없음)
    fxy: wp.vec2, cxy: wp.vec2,
    cam_pos: wp.vec3, cam_rot_inv: wp.quat,
    block_offset: wp.vec3i,                          # 블록 키 → 0 부터 세는 블록 번호
    grid_half: wp.vec3,                              # TSDF 칸 수 / 2 (x, y, z)
    origin: wp.vec3, voxel: float, bs: int,
    gap: float, trunc: float, keep: float, dmin: float, dmax: float,
    occl_gap: float,                                 # 가려진 칸 잊기: 3x3 이 모두 칸보다 이만큼 넘게 앞이면 무게를 줄임 (0 = 끔)
):
    """빈 공간 지우기: 카메라가 칸 너머의 표면을 확실히 보면 (3x3 픽셀 중 6개 이상 깊이, 가장 가까운 것도 칸보다 gap 넘게 뒤)
    그 칸을 빈 공간 쪽으로 덮어씀. 3x3 이 모두 칸보다 occl_gap 넘게 앞이면 (다른 것에 가려짐) 무게만 줄여 ~1 초 뒤 '못 본 칸' 으로."""
    b, l = wp.tid()
    if b >= num_allocated[0]:
        return
    if block_to_hash_slot[b] < 0:
        return
    w = wp.float32(block_data[b, l, 1])
    if w <= 0.0:
        return
    if wp.float32(block_data[b, l, 0]) >= 0.9 * trunc * w:
        return                                       # 이미 빈 공간 (다시 안 덮어씀 → 감쇠로 무게가 줄면 블록이 반납된다)
    gx = (block_coords[b * 3 + 0] + block_offset[0]) * bs + l % bs
    gy = (block_coords[b * 3 + 1] + block_offset[1]) * bs + (l // bs) % bs
    gz = (block_coords[b * 3 + 2] + block_offset[2]) * bs + l // (bs * bs)
    p = wp.vec3((wp.float32(gx) + 0.5 - grid_half[0]) * voxel + origin[0],
                (wp.float32(gy) + 0.5 - grid_half[1]) * voxel + origin[1],
                (wp.float32(gz) + 0.5 - grid_half[2]) * voxel + origin[2])
    pc = wp.quat_rotate(cam_rot_inv, p - cam_pos)
    z = pc[2]
    if not (z >= dmin):                              # NaN 도 여기서 걸러짐
        return
    u = fxy[0] * pc[0] / z + cxy[0]
    v = fxy[1] * pc[1] / z + cxy[1]
    if not (u >= 1.0 and v >= 1.0 and u < wp.float32(depth.shape[1] - 1) and v < wp.float32(depth.shape[0] - 1)):
        return
    px = wp.int32(u)
    py = wp.int32(v)
    n_ok = int(0)
    dm = float(1.0e9)
    dmx = float(0.0)
    for dy in range(-1, 2):
        for dx in range(-1, 2):
            d = depth[py + dy, px + dx]
            if d >= dmin and d <= dmax:
                n_ok += 1
                dm = wp.min(dm, d)
                dmx = wp.max(dmx, d)
    from3 = n_ok >= 6
    if n_ok >= 6 and dm - z > gap:
        sw = wp.float32(block_data[b, l, 0])
        block_data[b, l, 0] = wp.float16(keep * sw + wp.min(dm - z, trunc))   # 관측 1장 (sdf = 측정 - 칸, 절단), 무게 1
        block_data[b, l, 1] = wp.float16(keep * w + 1.0)
    elif occl_gap > 0.0 and from3 and dmx < z - occl_gap:
        sw = wp.float32(block_data[b, l, 0])
        block_data[b, l, 0] = wp.float16(keep * sw)
        block_data[b, l, 1] = wp.float16(keep * w)


@dataclass
class MapSettings:
    center_m: List[float]
    size_m: List[float]                     # ESDF 영역. 칸 수 = 1 + floor(크기 / 칸)
    esdf_voxel_size: float = 0.01
    tsdf_voxel_size: float = 0.005          # 깊이를 쌓는 칸 (ESDF 의 절반: gather 씨앗에 맞는 비율)
    truncation_voxels: float = 4.0          # 절단 거리 = TSDF 칸 x 4
    tsdf_margin_m: float = 0.1              # TSDF 영역 = ESDF 영역 + 사방 여유 (격자 가장자리 표면도 씨앗이 되게)
    image_width: int = 640                  # R2 realsense 640x480
    image_height: int = 480
    depth_min_m: float = 0.1
    depth_max_m: float = 2.0                # 카메라 → 격자 먼 모서리 약 1.6 m + 여유
    flying_pixel_threshold: float = 0.5     # FilterDepth 기본값
    bilateral_kernel_size: int = 5          # FilterDepth 기본값
    decay: float = 0.7                      # 모든 칸 감쇠 (쌓을 때마다): 다시 확인 안 되는 칸은 ~1 초 뒤 빠짐 (Mirobot 사용자 결정)
    frustum_decay: float = 1.0              # 시야 안 칸 추가 감쇠 (끔)
    segment_distance_threshold: float = 0.05    # 팔 지우기 (rt_config.Distances.mask 가 넣음)
    segmenter_cuda_graph: bool = False
    min_valid_ratio: float = 0.05           # 깊이가 있는 픽셀이 이 비율보다 적은 영상은 안 쌓음 (가려진 카메라)
    confirm_frames: int = 15                # 깜빡임 거르기: 같은 픽셀의 지난 측정(이 장 수 안)과 값이 비슷해야 쌓음
    confirm_tol_m: float = 0.015            # 지난 측정과의 차 허용 = 이 값 + confirm_tol_rel × 깊이
    confirm_tol_rel: float = 0.02
    carve: bool = True                      # 빈 공간 지우기 (커널 설명)
    carve_gap_m: float = 0.01
    carve_keep: float = 0.5                 # 빈 공간으로 보인 장마다 예전 값을 이 비율만 남김
    carve_max_depth_m: float = 4.0          # 이보다 먼 깊이는 뚫림 증거로 안 씀
    forget_occluded: bool = True            # 가려진 칸 잊기 (커널 설명). ⚠️ 가려진 실제 물체도 ~1 초 뒤 빠짐 (보이는 면만 — 머리말 사용자 결정)
    occluded_gap_m: float = 0.03            # 절단 거리(2 cm)보다 커야 보이는 물체의 뒷면이 남음
    min_tsdf_weight: float = 0.1
    mpc_floor_z_m: float = 0.015            # 이 높이 (받침 기준 m) 보다 낮은 점은 지도에 안 넣음 — rt_world 가 rt_config.MapCfg table_z + floor_band_m 을 넣음
    mpc_background_frames: int = 15         # 켤 때 지도가 차도록 기다리는 장 수 (1 초)
    # 팔 지우기 안으로 들어온 물체는 안 보여서 다가오는 물체가 지우기 경계(~4 cm)에 멈춰 보였다 (2026-10-13 실측: 실제 41 → −36 mm,
    # 지도 계속 ~42 mm). 사용자 결정 "경계 = 막힘": 새 물체 칸이 경계에 닿아 있으면 그 근처 지우기 영역을 새 물체로 채워 MPC 지도에 넣음
    # (값에 방향 — _fill_mask_boundary 설명)
    mpc_boundary_fill: bool = True
    # [2026-10-16] 받침 자리 빼기 (x0, y0, z0, x1, y1, z1, base 기준 m) — 이 상자 안의 깊이 점은 지도에 안 넣음 (rt_config.MapCfg.base_exclude)
    base_exclude: Optional[tuple] = None
    # [2026-10-26] 받침 둘레 책상 높이 빼기 (반지름 m, 높이 m) — 받침 축에서 반지름 안 · 높이 아래 깊이 점은 지도에 안 넣음 (실제 카메라의
    #  받침 옆 케이블, rt_config.MapCfg.base_ring)
    base_ring: Optional[tuple] = None

    def __post_init__(self):
        assert int(self.confirm_frames) >= 0, 'confirm_frames >= 0'
        assert 0.0 <= self.carve_keep < 1.0, 'carve_keep 은 0 이상 1 미만'
        assert self.carve_gap_m >= self.tsdf_voxel_size, 'carve_gap_m 은 TSDF 칸 이상'
        assert self.carve_max_depth_m > self.depth_min_m
        assert self.occluded_gap_m > self.truncation_voxels * self.tsdf_voxel_size, 'occluded_gap_m 은 절단 거리보다 커야 함'

    @property
    def shape(self) -> List[int]:
        return [1 + math.floor(s / self.esdf_voxel_size + 1e-6) for s in self.size_m]

    @property
    def origin_m(self) -> List[float]:
        """0번 ESDF 칸의 아래 모서리."""
        return [c - n * self.esdf_voxel_size / 2 for c, n in zip(self.center_m, self.shape)]


class CameraMap:
    def __init__(self, s: MapSettings, robot: dict, device: str = 'cuda:0'):
        self.s = s
        self.device = torch.device(device)
        h, w = s.image_height, s.image_width
        self.segmenter = RobotSegmenter(Kinematics(RobotCfg.create(robot).kinematics),
                                        distance_threshold=s.segment_distance_threshold, ops_dtype=torch.float32,
                                        use_cuda_graph=s.segmenter_cuda_graph)
        self.filter = FilterDepth(image_shape=(h, w), depth_minimum_distance=s.depth_min_m,
                                  depth_maximum_distance=s.depth_max_m,
                                  flying_pixel_threshold=s.flying_pixel_threshold,
                                  bilateral_kernel_size=s.bilateral_kernel_size or None)
        n = s.shape
        self.mapper_mpc = Mapper(MapperCfg(     # 보이는 모든 것 (바닥 띠 뺌)
            extent_meters_xyz=tuple(sz + 2 * s.tsdf_margin_m for sz in s.size_m),
            voxel_size=s.tsdf_voxel_size,
            esdf_voxel_size=s.esdf_voxel_size,
            # Mapper 는 칸 수를 ceil(크기 / 칸) 로 센다 → (n - 0.5) 칸 크기를 주면 정확히 n 칸
            extent_esdf_meters_xyz=tuple((k - 0.5) * s.esdf_voxel_size for k in n),
            grid_center=torch.tensor(s.center_m, dtype=torch.float32),
            truncation_distance=s.truncation_voxels * s.tsdf_voxel_size,
            depth_minimum_distance=s.depth_min_m,
            depth_maximum_distance=s.depth_max_m,
            decay_factor=s.decay,
            frustum_decay_factor=s.frustum_decay,
            minimum_tsdf_weight=s.min_tsdf_weight,
            num_cameras=1,
            image_height=h,
            image_width=w,
            device=str(self.device),
        ))
        # 빈 공간 지우기는 Mapper 내부 저장소를 직접 쓴다 (공개 API 없음, V2 0.8.0). 모양이 다르면 지도를 망가뜨리지 말고 켤 때 멈춘다
        data = self.mapper_mpc._integrator.tsdf.data
        n_blk = int(self.mapper_mpc._integrator.tsdf.config.max_blocks)
        assert data.block_data.dtype == torch.float16 and data.block_data.shape[-1] == 2 and data.block_data.shape[0] >= n_blk, \
            f'Mapper block_data 모양이 예상과 다름: {data.block_data.dtype} {tuple(data.block_data.shape)}'
        assert data.block_coords.numel() >= n_blk * 3 and data.block_coords.dtype == torch.int32, 'Mapper block_coords 모양이 예상과 다름'
        self._bg_left = 0                       # 켤 때 지도가 차도록 더 기다릴 장 수 (start_background)
        self.bg_ready = False                   # 다 기다렸나 (rt_run 은 그 뒤에 다리를 시작)
        self._vg_comb = None                    # MPC 지도 (한 번 만든 GPU 버퍼에 다시 씀 → MPC·계획기는 복사 없이 계속 봄)
        self._raw = None                        # 실제 거리 지도 (같은 칸)
        self._vg_dyn = None                     # '새 물체' 지도 = 카메라면 지도 ESDF 그대로 (raw 와 같은 값), 참 형상이면 움직이는 물체만
        self.version = 0                        # 지도 판: raw·dyn·MPC 지도가 바뀔 때마다 +1 (esdf_mpc) — 쓰는 쪽 (rt_world) 의 변환 기억용
        self._bsph = None                       # 영상 시각의 로봇 구 [S, 4] (경계 채우기 — boundary_spheres, rt_world 가 넣음)
        self._fill_n = 0                        # 마지막으로 채운 칸 수 (boundary_fill — GPU 에 있으면 near_diag 가 진단과 함께 읽음)
        # [2026-10-27 R-1] 영상마다 한 번만: 새 물체 칸 (지도 판마다), 경계 채우기 때 구한 칸 ↔ 구 겉면 거리 (그 구·그 판, 진단이 다시 씀)
        self._occ_ver, self._occ_idx, self._occ_P = -1, None, None
        self._fill_sph = self._fill_dP = None
        self._fill_ver = -1
        self._dirty = True                      # [R-1] 지도를 다시 계산해야 하나 — 쌓기·경계 채우기 구·참 형상·초기화 뒤 (esdf_mpc)
        self._org = torch.tensor(s.origin_m, dtype=torch.float32, device=self.device)     # 0번 칸 모서리 (예전: 부를 때마다 만듦)
        self._nt = torch.tensor(n, device=self.device)                                     # 칸 수 [x, y, z]
        self._ray_z = None                      # 픽셀마다 광선의 base z 성분 [1, 높이, 너비] (깊이 1 m 당) — 바닥 띠 빼기
        self._cam_z = 0.0
        self._ray_x = self._ray_y = None        # 같은 광선의 x·y 성분 — 받침 자리·케이블 자리 빼기 (set_camera)
        self._cam_xy = (0.0, 0.0)
        self._carve_cam = None                  # 빈 공간 지우기 커널의 카메라 값 (set_camera)
        self._last_d = torch.zeros((h, w), device=self.device)      # 깜빡임 거르기: 픽셀마다 마지막 측정 깊이
        self._last_age = torch.full((h, w), 1 << 30, dtype=torch.int32, device=self.device)   # 그 뒤로 지난 장 수
        self.rgb = torch.zeros((1, h, w, 3), dtype=torch.uint8, device=self.device)   # 색은 안 씀
        self.intrinsics = None
        self.camera_pose = None                 # base_link 기준 깊이 광학 좌표
        self._oracle = None                     # [검증 전용] 참 형상 상자 (set_oracle) — 있으면 esdf_mpc 가 카메라 대신 이것으로
        self._oracle_P = None                   # 칸 중심 [nx, ny, nz, 3]
        self._oracle_far = None                 # 빈 지도의 거리 값 (참 형상 거리도 여기서 자름)

    def set_camera(self, intrinsics_3x3, pose_xyz_wxyz: List[float]):
        """카메라 내부값(3x3)과 base_link 기준 깊이 광학 좌표의 자세 [x y z qw qx qy qz]."""
        k = torch.as_tensor(intrinsics_3x3, dtype=torch.float32, device=self.device).view(1, 3, 3)
        self.intrinsics = k
        self.camera_pose = Pose.from_list(list(pose_xyz_wxyz))
        self.segmenter.update_camera_projection(CameraObservation(
            depth_image=torch.zeros((1, self.s.image_height, self.s.image_width), device=self.device),
            intrinsics=k, depth_to_meter=1.0))
        self._last_age.fill_(1 << 30)           # 다른 위치에서 찍은 지난 측정과는 비교하지 않는다
        kc = k[0].cpu()
        x, y, z, qw, qx, qy, qz = (float(v) for v in pose_xyz_wxyz)
        r20, r21, r22 = 2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)
        hh, ww = self.s.image_height, self.s.image_width
        vv, uu = torch.meshgrid(torch.arange(hh, device=self.device, dtype=torch.float32),
                                torch.arange(ww, device=self.device, dtype=torch.float32), indexing='ij')
        a, b = (uu - float(kc[0, 2])) / float(kc[0, 0]), (vv - float(kc[1, 2])) / float(kc[1, 1])
        self._ray_z = (r20 * a + r21 * b + r22).view(1, hh, ww)
        self._cam_z = z
        r00, r01, r02 = 1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)
        r10, r11, r12 = 2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)
        self._ray_x = (r00 * a + r01 * b + r02).view(1, hh, ww)     # 받침 자리 빼기 (base_exclude) 용 광선 x·y 성분
        self._ray_y = (r10 * a + r11 * b + r12).view(1, hh, ww)
        self._cam_xy = (x, y)
        self._carve_cam = (wp.vec2(float(kc[0, 0]), float(kc[1, 1])), wp.vec2(float(kc[0, 2]), float(kc[1, 2])),
                           wp.vec3(x, y, z), wp.quat_inverse(wp.quat(qx, qy, qz, qw)))

    def _confirm(self, raw: torch.Tensor) -> Optional[torch.Tensor]:
        """깜빡임 거르기: 지금 깊이가 있고 같은 픽셀의 지난 측정(confirm_frames 장 안)과 차가 허용 안인 픽셀 = True (끔이면 None).
        FilterDepth **뒤에** 곱한다 (앞에서 구멍을 내면 FilterDepth 가 이웃을 튀는 점으로 지워 실제 표면까지 깎임)."""
        if int(self.s.confirm_frames) <= 0:
            return None
        cur = raw[0]
        valid = cur > 0
        tol = self.s.confirm_tol_m + self.s.confirm_tol_rel * cur
        ok = valid & (self._last_age <= int(self.s.confirm_frames)) & ((cur - self._last_d).abs() <= tol)
        self._last_d.copy_(torch.where(valid, cur, self._last_d))
        self._last_age.copy_(torch.where(valid, torch.zeros_like(self._last_age), (self._last_age + 1).clamp(max=1 << 30)))
        return ok.view_as(raw)

    def _carve(self, depth: torch.Tensor, mapper):
        """빈 공간 지우기 (커널 설명). depth = 깜빡임을 거른 원래 깊이 [높이, 너비] (팔 지우기 전)."""
        tsdf = mapper._integrator.tsdf     # 공개 API 없음 (V2 0.8.0): BlockSparseTSDF
        c, data = tsdf.config, tsdf.data
        bs = int(c.block_size)
        nz, ny, nx = (int(v) for v in c.grid_shape)
        org = [float(v) for v in c.origin]
        n_alloc = min(int(data.num_allocated.item()), int(c.max_blocks))
        if n_alloc <= 0:
            return
        fxy, cxy, cam_p, cam_qi = self._carve_cam
        trunc = self.s.truncation_voxels * self.s.tsdf_voxel_size
        device, stream = get_warp_device_stream(data.block_data)
        wp.launch(_carve_free_space, dim=(n_alloc, bs ** 3), inputs=[
            wp.from_torch(data.block_coords, dtype=wp.int32), wp.from_torch(data.block_to_hash_slot, dtype=wp.int32),
            wp.from_torch(data.num_allocated, dtype=wp.int32), wp.from_torch(data.block_data, dtype=wp.float16),
            wp.from_torch(depth.contiguous(), dtype=wp.float32), fxy, cxy, cam_p, cam_qi,
            wp.vec3i(((nx + bs - 1) // bs) // 2, ((ny + bs - 1) // bs) // 2, ((nz + bs - 1) // bs) // 2),
            wp.vec3(nx * 0.5, ny * 0.5, nz * 0.5), wp.vec3(org[0], org[1], org[2]), float(c.voxel_size), bs,
            float(self.s.carve_gap_m), trunc, float(self.s.carve_keep),
            float(self.s.depth_min_m), float(self.s.carve_max_depth_m),
            float(self.s.occluded_gap_m) if self.s.forget_occluded else 0.0], device=device, stream=stream)

    def integrate(self, depth_m: torch.Tensor, q: JointState) -> bool:
        """깊이 한 장(m, [높이, 너비], GPU float32)을 쌓는다. q = 영상 시각의 관절각 (active joints).
        깊이가 남은 픽셀이 min_valid_ratio 보다 적으면 안 쌓고 False."""
        self._dirty = True
        hw = (self.s.image_height, self.s.image_width)
        raw = depth_m.view(1, *hw).to(torch.float32, copy=True)   # 팔 지우기 전 깊이 (빈 공간 지우기·배경용)
        _, world = self.segmenter.get_robot_mask_from_active_js(CameraObservation(depth_image=raw.clone(), pose=self.camera_pose), q)
        ok = self._confirm(raw)                          # 깜빡임 거르기 (판단은 원래 깊이로)
        depth, valid = self.filter(world.contiguous())   # 팔을 지운 뒤에 거른다 (지운 칸 = 0 = 깊이 없음)
        if ok is not None:
            depth = depth * ok
            valid = valid.bool() & ok
            raw = raw * ok                               # 빈 공간 지우기도 깜빡이지 않는 깊이만 증거로
        if float(valid.float().mean()) < self.s.min_valid_ratio:
            return False
        above = (self._cam_z + depth * self._ray_z) > self.s.mpc_floor_z_m     # 바닥 띠 빼기
        if self.s.base_exclude is not None or self.s.base_ring is not None:
            px = self._cam_xy[0] + depth * self._ray_x
            py = self._cam_xy[1] + depth * self._ray_y
            pz = self._cam_z + depth * self._ray_z
            if self.s.base_exclude is not None:          # 받침 자리 빼기 (움직이지 않는 로봇 자기 몸)
                x0, y0, z0, x1, y1, z1 = self.s.base_exclude
                above = above & ~((px > x0) & (px < x1) & (py > y0) & (py < y1) & (pz > z0) & (pz < z1))
            if self.s.base_ring is not None:             # 받침 둘레 책상 높이 빼기 (케이블)
                rr, zz = self.s.base_ring
                above = above & ~((px * px + py * py < rr * rr) & (pz < zz))
        self.mapper_mpc.integrate(CameraObservation(depth_image=depth * above, rgb_image=self.rgb, pose=self.camera_pose,
                                                    intrinsics=self.intrinsics))
        if self.s.carve:
            self._carve(raw.view(*hw), self.mapper_mpc)
        if self._bg_left > 0:            # 켤 때: 지도가 차도록 기다리는 장 수 (머리말)
            self._bg_left -= 1
            self.bg_ready = self._bg_left == 0
        return True

    def esdf_mpc(self) -> VoxelGrid:
        """MPC 지도와 실제 거리 지도를 다시 계산 (머리말). 돌려준 VoxelGrid 는 늘 같은 GPU 버퍼 → MPC 장면에 한 번 넣어 두면
        다시 계산할 때마다 MPC 가 바로 본다 (V2 create_from_voxel_grids: 복사 없이 참조).
        [2026-10-27 R-1] 그 뒤로 쌓기·경계 채우기 구·참 형상·초기화가 없었으면 다시 계산 안 함 (멈춰 계획·다시 계획 전 rt_world.plan_prep —
        같은 TSDF·같은 구 → 같은 지도: ESDF 는 TSDF 만으로 정해짐, 씨앗을 매번 지우고 다시 구함)."""
        if not self._dirty and self._vg_comb is not None:
            return self._vg_comb
        self._dirty = False
        if self._oracle is not None:
            return self._esdf_oracle()
        full = self.mapper_mpc.compute_esdf()
        self._vg_dyn = full                                                # 보이는 모든 칸 = 새 물체 지도 (머리말)
        self.version += 1
        if self._vg_comb is None:
            self._vg_comb = VoxelGrid(name='rt_split_esdf', pose=list(full.pose), dims=list(full.dims), voxel_size=full.voxel_size,
                                      feature_tensor=full.feature_tensor.clone())
            self._raw = full.feature_tensor.clone()
        comb = self._vg_comb.feature_tensor
        comb.copy_(full.feature_tensor)
        self._raw.copy_(comb)
        self.boundary_fill = self._fill_mask_boundary(comb.view(*self.s.shape)) if self.s.mpc_boundary_fill else 0
        return self._vg_comb

    # ---------------- [검증 전용] 참 형상 지도 (2026-10-15 사용자 결정: 검증에만, 실제 회피는 카메라만) ----------------
    def set_oracle(self, static_boxes, dyn_boxes):
        """다음 esdf_mpc 부터 카메라 대신 시뮬레이터가 아는 참 형상으로 지도를 만든다. static_boxes·dyn_boxes = [(중심 xyz, 크기 xyz)]
        (축 정렬 상자). 정지 = 상자, 새 물체 = 상자 전체 (카메라 1 대가 못 보는 면·팔 지우기 사각지대 없음). 목적: 성형 때문에 생긴
        닿음을 카메라의 기존 한계와 섞지 않고 가리기 (작업기록 5-11). 바닥은 넣지 않음 — MPC·계획기는 바닥 상자, 안전 검사는 바닥 면
        (rt_world._sample) 으로 따로 봄. [2026-10-15 시험] 바닥 아래 반공간을 넣었더니 바닥에 붙은 받침 구가 늘 '닿음' (−10 mm) 으로 잡힘
        (카메라 지도는 켤 때 팔 밑을 '모름' 으로 비워 생기지 않던 일). 카메라 지도와 다른 점: 안 보이는 면도 막힘, 지도 늦음·칸 오차 없음."""
        self._oracle = ([(tuple(b[0]), tuple(b[1])) for b in static_boxes], [(tuple(b[0]), tuple(b[1])) for b in dyn_boxes])
        self.bg_ready = True
        self._dirty = True

    @staticmethod
    def _box_sdf(P: torch.Tensor, center, dims) -> torch.Tensor:
        """칸 중심 P [..., 3] 에서 축 정렬 상자까지 부호 있는 거리 (안쪽 음수, m)."""
        c = torch.tensor(center, dtype=torch.float32, device=P.device)
        h = torch.tensor(dims, dtype=torch.float32, device=P.device) / 2
        d = (P - c).abs() - h
        return torch.clamp(d, min=0).norm(dim=-1) + torch.clamp(d.max(dim=-1).values, max=0)

    def _esdf_oracle(self) -> VoxelGrid:
        """esdf_mpc 의 참 형상판: 같은 GPU 버퍼 (MPC·계획기가 복사 없이 보는 것) 에 실제 거리 = min(정지, 새 물체), MPC 지도 = 실제
        거리 (경계 채우기는 없음 — 팔 지우기 사각지대가 없으므로), 새 물체 지도 = 움직이는 물체만 (기록 newvox) 으로 쓴다.
        [2026-10-27 R-1] 정지 거리 변환은 뺌 (구분 없는 지도에서는 반응 거리가 같아 늘 아무것도 안 바꿨음)."""
        if self._vg_comb is None:               # 버퍼 모양·자료형은 카메라 지도와 같게 (빈 지도의 ESDF 로 한 번 만듦)
            full = self.mapper_mpc.compute_esdf()
            self._vg_dyn = full
            self._vg_comb = VoxelGrid(name='rt_split_esdf', pose=list(full.pose), dims=list(full.dims), voxel_size=full.voxel_size,
                                      feature_tensor=full.feature_tensor.clone())
            self._raw = full.feature_tensor.clone()
        if self._oracle_P is None:
            n = self.s.shape
            axes = [torch.arange(n[k], device=self.device) for k in range(3)]
            self._oracle_P = self._voxel_centers(torch.stack(torch.meshgrid(*axes, indexing='ij'), dim=-1))
            # 빈 지도의 값 (참 형상이면 카메라 지도는 안 쌓아 비어 있음 — 이 계산이 새 물체 버퍼를 덮어도 바로 아래에서 다시 씀)
            self._oracle_far = float(self.mapper_mpc.compute_esdf().feature_tensor.float().max())
        P = self._oracle_P
        static_boxes, dyn_boxes = self._oracle
        static = torch.full_like(P[..., 0], self._oracle_far)
        for c, d in static_boxes:
            static = torch.minimum(static, self._box_sdf(P, c, d))
        dyn = torch.full_like(static, self._oracle_far)
        for c, d in dyn_boxes:
            dyn = torch.minimum(dyn, self._box_sdf(P, c, d))
        comb, vdyn = self._vg_comb.feature_tensor, self._vg_dyn.feature_tensor
        self.version += 1
        comb.copy_(static.clamp(max=self._oracle_far).reshape(comb.shape).to(comb.dtype))
        vdyn.copy_(dyn.reshape(vdyn.shape).to(vdyn.dtype))
        self._raw.copy_(comb)                   # 실제 거리 = min(정지, 새 물체)
        torch.minimum(self._raw, vdyn.view(self._raw.shape).to(self._raw.dtype), out=self._raw)
        torch.minimum(comb, vdyn.view(comb.shape).to(comb.dtype), out=comb)
        self.boundary_fill = 0
        return self._vg_comb

    @property
    def raw_is_dyn(self) -> bool:
        """실제 거리 지도와 새 물체 지도가 같은 값인가 (카메라 지도 — 머리말. 참 형상 지도는 아님: 새 물체 = 움직이는 물체만)."""
        return self._oracle is None

    def raw_grid(self) -> torch.Tensor:
        """실제 거리 지도 [x, y, z] (esdf_mpc 뒤에 유효)."""
        return self._raw.view(*self.s.shape)

    def dyn_grid(self) -> torch.Tensor:
        """새 물체 거리 지도 [x, y, z] (esdf_mpc 뒤에 유효)."""
        return self._vg_dyn.feature_tensor.view(*self.s.shape)

    def mpc_grid(self) -> torch.Tensor:
        """MPC 가 보는 지도 [x, y, z] (정지 거리 변환·경계 채우기 포함, esdf_mpc 뒤에 유효)."""
        return self._vg_comb.feature_tensor.view(*self.s.shape)

    @property
    def boundary_spheres(self):
        """영상 시각의 로봇 구 [S, 4] (경계 채우기, rt_world 가 넣음)."""
        return self._bsph

    @boundary_spheres.setter
    def boundary_spheres(self, v):
        self._bsph = v
        self._dirty = True                      # 경계 채우기가 바뀜

    @property
    def boundary_fill(self) -> int:
        """마지막으로 채운 칸 수 (진단). [2026-10-27 R-1] 아직 GPU 에 있으면 여기서 읽음 (rt_world 는 near_diag 가 진단 거리와 함께 한 번에)."""
        if torch.is_tensor(self._fill_n):
            self._fill_n = int(self._fill_n)
        return self._fill_n

    @boundary_fill.setter
    def boundary_fill(self, v):
        self._fill_n = v

    def _voxel_centers(self, idx: torch.Tensor) -> torch.Tensor:
        return self._org + (idx.float() + 0.5) * self.s.esdf_voxel_size

    def _occupied(self):
        """새 물체 칸 (거리 ≤ 칸/2) 의 번호 [N, 3] (long, 한 줄 번호 오름차순) 과 중심 [N, 3] (m) — [2026-10-27 R-1] 지도 판마다 한 번
        (경계 채우기·진단·기록이 같이 씀. 예전: 영상마다 세 번)."""
        if self._occ_ver != self.version:
            idx = torch.nonzero(self.dyn_grid().float() <= 0.5 * self.s.esdf_voxel_size)
            self._occ_ver, self._occ_idx, self._occ_P = self.version, idx, self._voxel_centers(idx)
        return self._occ_idx, self._occ_P

    def new_voxel_centers(self) -> torch.Tensor:
        """새 물체 칸 중심 [N, 3] (m)."""
        if self._vg_dyn is None:
            return torch.zeros((0, 3), device=self.device)
        return self._occupied()[1]

    def occupied_flat(self) -> torch.Tensor:
        """새 물체 칸의 한 줄 번호 ((ix·ny + iy)·nz + iz, 오름차순) [N] (GPU long) — 기록 (rt_world.voxel_code 'dyn')."""
        idx = self._occupied()[0]
        _, ny, nz = self.s.shape
        return (idx[:, 0] * ny + idx[:, 1]) * nz + idx[:, 2]

    def _fill_mask_boundary(self, comb_view):
        """경계 채우기 (MapSettings.mpc_boundary_fill): 로봇 표면에서 mask + 1.5 칸 안의 새 물체 칸 B 를 찾고, B 둘레 15 cm 상자 안의
        지우기 영역 (로봇 표면 mask 안 = 안 보이는 곳) 칸마다 값 = B 까지 거리 − 채우기 폭 (mask + 1 칸) 을 MPC 지도에 넣음: 물체가 B 에서
        로봇 쪽으로 폭만큼 들어와 있을 수 있다고 보고, B 에서 멀어지는 쪽으로 거리가 늘게 (구 중심에서도 기울기가 있어 MPC 가 빠질 방향을 앎).
        [2026-10-16 사용자 결정 (나) "좋다"] 그 전 (10-13) 은 B 근처 지우기 영역을 칸으로 채우고 그 칸까지 거리 → 안쪽은 −반 칸으로 납작해 구
        중심이 그 안이면 기울기 없음 → WMX 실행에서 MPC 지도 −27 mm 고정, 당근이 안 움직이고 물체가 와서 닿음 (작업기록 5-21).
        [2026-10-27 R-1] 그 방식 되돌리기 (mpc_boundary_fill_grad False) 는 뺌.
        실제 거리 지도 (안전 검사·계획 검증) 에는 안 넣음: 넣어 본 시험 (2026-10-13 이상적 모드) 에서 로봇이 지우기 영역 안 '닿음' 으로
        멈춰 다리 2/17 (계획 검증·안전 검사가 손이 5 cm 안에 오면 아무 데도 못 가게 함).
        반환 = 지우기 영역 중 값이 0 이하인 칸 수 (GPU 에 둔 수 — boundary_fill·near_diag 가 읽음).
        [2026-10-27 R-1] 새 물체 칸은 지도 판마다 한 번 (_occupied), 칸 ↔ 구 겉면 거리는 진단 (near_diag) 이 다시 씀, 상자 범위는 GPU → CPU
        한 번 (예전 int() 12 번)."""
        self._fill_sph = self._fill_dP = None
        sph = self._bsph
        if sph is None or self._vg_dyn is None:
            return 0
        vs = self.s.esdf_voxel_size
        mask = float(self.s.segment_distance_threshold)
        P = self.new_voxel_centers()
        if P.shape[0] == 0:
            return 0
        c, r = sph[:, :3].float(), sph[:, 3].float()
        dP = min_dist(P, c, r)                                                   # 새 물체 칸마다 로봇 구 겉면까지 최소
        self._fill_sph, self._fill_dP, self._fill_ver = sph, dP, self.version
        B = P[dP <= mask + FILL_BAND_VOX * vs]
        if B.shape[0] == 0:
            return 0
        if B.shape[0] > FILL_MAX_B:
            B = B[torch.linspace(0, B.shape[0] - 1, FILL_MAX_B, device=B.device).long()]
        lo = torch.clamp(torch.floor((B.min(0).values - FILL_BOX_M - self._org) / vs).long(), min=0)
        hi = torch.minimum(torch.floor((B.max(0).values + FILL_BOX_M - self._org) / vs).long() + 1, self._nt)
        (l0, l1, l2), (h0, h1, h2) = torch.stack([lo, hi]).tolist()
        axes = [torch.arange(a, b, device=self.device) for a, b in ((l0, h0), (l1, h1), (l2, h2))]
        V = self._voxel_centers(torch.stack(torch.meshgrid(*axes, indexing='ij'), dim=-1).view(-1, 3))
        zone = min_dist(V, c, r) <= mask                                         # 지우기 영역 (안 보이는 곳)
        # B 까지 거리는 지우기 영역 칸만 ([2026-10-27 R-1] 예전: 상자 칸 전부 — 대부분 지우기 영역 밖)
        val = torch.full((V.shape[0],), float('inf'), device=self.device)
        val[zone] = min_dist(V[zone], B) - (mask + vs)
        box = comb_view[l0:h0, l1:h1, l2:h2]
        box.copy_(torch.minimum(box, val.view(box.shape).to(box.dtype)))
        return (val <= 0).sum()

    def near_diag(self, spheres: torch.Tensor) -> Optional[dict]:
        """(진단) 로봇 구 [N, 4] 에서 MPC 지도 (near_mm, 칸 중심 값이라 ±1 칸) 와 새 물체 칸 (near_new_mm, 칸이 있을 때만) 까지의 거리.
        [2026-10-27 R-1] 경계 채우기가 같은 구 (같은 텐서) 로 이 지도 판에 구한 칸 거리를 다시 쓰고, 채운 칸 수 (boundary_fill) 와 함께
        GPU → CPU 한 번 (예전: 새 물체 칸·거리를 다시 구하고 따로따로 읽음)."""
        if self._vg_comb is None or self._vg_dyn is None:
            return None
        n, vs = self.s.shape, self.s.esdf_voxel_size
        c, r = spheres[:, :3].float(), spheres[:, 3].float()
        idx = ((c - self._org) / vs).floor().long()
        for k in range(3):
            idx[:, k].clamp_(0, n[k] - 1)
        comb = self._vg_comb.feature_tensor.view(*n)
        vals = [(comb[idx[:, 0], idx[:, 1], idx[:, 2]].float() - r).min()]
        P = self.new_voxel_centers()
        if P.shape[0]:
            same = self._fill_sph is spheres and self._fill_ver == self.version
            vals.append((self._fill_dP if same else min_dist(P, c, r)).min())
        fill = torch.is_tensor(self._fill_n)
        if fill:
            vals.append(self._fill_n)
        host = torch.stack([v.double() for v in vals]).tolist()
        if fill:
            self._fill_n = int(host[-1])
        out = {'near_mm': round(host[0] * 1000, 1)}
        if P.shape[0]:
            out['near_new_mm'] = round(host[1] * 1000, 1)
        return out

    def start_background(self):
        """켤 때 (또는 지도 새로 쌓기): 다음 mpc_background_frames 장 동안 지도가 차기를 기다림 (bg_ready). 지도는 그대로 쌓임 (머리말)."""
        self._bg_left = max(1, int(self.s.mpc_background_frames))
        self.bg_ready = False

    def warmup(self):
        """GPU 코드 컴파일과 ESDF CUDA 그래프 녹화를 미리 한다 (첫 영상이 느려지지 않게). 끝나면 비운 상태."""
        h, w = self.s.image_height, self.s.image_width
        f = 0.5 * w
        k = torch.tensor([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], dtype=torch.float32, device=self.device)
        cam = Pose.from_list([0.0, 0.0, max(self.s.size_m) / 2 + 0.3, 0.0, 1.0, 0.0, 0.0])   # 위에서 아래를 봄
        self.set_camera(k, cam.tolist())
        depth = torch.full((h, w), float(cam.position[0, 2]), device=self.device)           # 높이 0 의 바닥
        q = self.segmenter.kinematics.get_active_js(JointState.from_position(
            torch.zeros((1, len(self.segmenter.kinematics.joint_names)), device=self.device),
            joint_names=self.segmenter.kinematics.joint_names))
        self.start_background()                 # 켤 때 기다리기 (1·2장째) 와 그 뒤 (3·4장째) 길도 한 번씩
        self._bg_left = 2
        for _ in range(4):
            self.integrate(depth, q)
            self.esdf_mpc()
        torch.cuda.synchronize(self.device)
        self.mapper_mpc.reset()
        self._dirty = True
        self._bg_left, self.bg_ready = 0, False
        self._last_age.fill_(1 << 30)
        self.intrinsics = self.camera_pose = None
