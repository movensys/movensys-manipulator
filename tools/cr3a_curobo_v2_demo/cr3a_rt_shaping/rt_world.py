"""장면·지도·전역 계획기 (GPU). 합성 깊이 카메라 1대 (depth_sim) → cr3a_mapper.CameraMap → MPC 지도·실제 거리 지도,
cuRoboV2 MotionPlanner (MPC 와 같은 지도·같은 반응 거리), 실제 거리 검사 (안전 검사·계획 검증).
장면 (로봇 받침 기준, m): 바닥 z = World.floor_z (= max(MapCfg.table_z, 0) — 시뮬레이션 0, 실제 카메라는 카메라 보정이 잰 테이블 높이.
MPC·계획기에는 상자로), 정지 기둥 (rt_config.TaskCfg.pillar), 움직이는 물체 (rt_scenarios).
"""
import base64
import time

import numpy as np
import torch
import yaml

import cr3a_mapper as cmap
import depth_sim as ds
import rt_model
from curobo.scene import Cuboid, Scene, VoxelGrid
from curobo.types import JointState
from rt_config import Config, Distances

JOINTS = [f'joint{i}' for i in range(1, 7)]
FAR_M = 9.0                 # '멀다' (m): 격자 밖 지도 값·쓰지 않는 구 (반지름 0)·움직이지 않는 구의 바닥 면 거리 — 최소 거리에서 안 뽑힘


def robot_cfg(model=None) -> dict:
    """cuRoboV2 로봇 설정 (rt_model: None = 환경 변수 CR3A_MODEL 또는 기본 'tool')"""
    return yaml.safe_load(open(rt_model.get(model).yml))


def pack_voxels(idx) -> str:
    """칸 번호들 (지도 [x, y, z] 를 한 줄로 편 번호 = (ix·ny + iy)·nz + iz) → 정렬 → 앞 번호와의 차이 (첫 값은 −1 에서) → varint
    (7 비트씩, 위 비트 = 다음 바이트 이어짐) → base64. 보고서 (rt_viewer_template.html unpackVox) 가 푼다. 칸 3000 개 ≈ 4 KB."""
    d = np.diff(np.concatenate([[-1], np.sort(np.asarray(idx, np.int64))])).astype(np.uint32)
    n = 1 + sum((d >= (1 << (7 * k))).astype(np.int64) for k in range(1, 5))
    pos = np.concatenate([[0], np.cumsum(n)[:-1]]).astype(np.int64)
    out = np.zeros(int(n.sum()), np.uint8)
    for k in range(5):
        m = n > k
        out[pos[m] + k] = ((d[m] >> np.uint32(7 * k)) & 0x7F).astype(np.uint8) | ((n[m] > k + 1).astype(np.uint8) << 7)
    return base64.b64encode(out.tobytes()).decode()


def unpack_voxels(code: str) -> np.ndarray:
    """pack_voxels 의 반대 (시험·분석용)."""
    out, cur, val, sh = [], -1, 0, 0
    for b in base64.b64decode(code):
        val |= (b & 0x7F) << sh
        if b & 0x80:
            sh += 7
        else:
            cur += val
            out.append(cur)
            val, sh = 0, 0
    return np.array(out, np.int64)


def floor_cuboid(d: Distances, z: float = 0.0) -> Cuboid:
    """바닥 (윗면 z): MPC·계획기에는 지도 대신 상자로 준다 (지도에서는 바닥 띠를 뺌, cr3a_mapper mpc_floor_z_m).
    반응 거리는 하나 (새 물체) 라서 반응 거리 차만큼 낮춰 바닥 반응 거리 (Distances.act_floor, 2 cm) 에서 반응하게 함.
    [2026-10-16] 전에는 정지 물체 반응 거리 (act_static, 같은 2 cm) 를 썼음 — 정지 물체가 6 cm 가 되면서 바닥은 따로 (act_floor 설명).
    [2026-10-27 D-2] z = 충돌 바닥 높이 (World.floor_z, 받침 원점 기준)."""
    return Cuboid(name='floor', pose=[0.0, 0.0, z - 0.01 - (d.act_new - d.act_floor), 1, 0, 0, 0], dims=[2.0, 2.0, 0.02])


class World:
    def __init__(self, robot: dict, c: Config, log=print, oracle: bool = False, ik_tol=None):
        """oracle = [검증 전용, 2026-10-15 사용자 결정] 카메라 대신 참 형상 지도 (cr3a_mapper.CameraMap.set_oracle). 실제 회피는 카메라만.
        ik_tol = (위치 m, 방향 rad): 계획기 IK (planner.ik_solver — 공구 끝 목표) 성공 기준. None = cuRobo 기본 (5 mm · 0.05 rad, rt_run).
        [2026-10-28 R-1] 계획기를 만들기 전에 IK 설정에 넣음 (예전: rt_server 가 warmup 뒤 planner.ik_solver.config 를 바꿈).
        MotionPlannerCfg.create 의 position_tolerance·orientation_tolerance 는 TrajOpt 성공 기준 (경로 끝 공구 자세) 도 같이 바꿔서 안 씀
        — IK 만 (서버가 바꾸던 범위 그대로)."""
        t0 = time.time()
        self.oracle = oracle
        s = cmap.MapSettings(center_m=list(c.map.center), size_m=list(c.map.size), esdf_voxel_size=c.map.voxel,
                             segment_distance_threshold=c.dist.mask,
                             base_exclude=tuple(c.map.base_exclude),
                             base_ring=tuple(c.map.base_ring) if c.map.base_ring else None,
                             mpc_floor_z_m=c.map.table_z + c.map.floor_band_m)     # [D-2] 바닥 띠 = 테이블 윗면 + 1.5 cm
        self.cm = cmap.CameraMap(s, robot)
        self.cm.warmup()
        self.cam = ds.DepthSim(model=c.model)
        T = self.cam.cam_T[0]
        self.cm.set_camera(ds.intrinsics(), ds.pose7(T))
        # [2026-10-27 D-2] 충돌 바닥 높이 = max(테이블 윗면, 0): 테이블이 받침 원점보다 낮게 재지면 예전처럼 0 (안전 쪽), 높으면 올림 (MapCfg.table_z)
        self.floor_z = max(float(c.map.table_z), 0.0)
        self.floor = floor_cuboid(c.dist, self.floor_z)
        from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
        pcfg = MotionPlannerCfg.create(
            robot=robot, scene_model=Scene(cuboid=[self.floor], voxel=[self._empty_grid()]),
            collision_cache={'cuboid': 4, 'voxel': 1}, num_ik_seeds=c.plan.ik_seeds, num_trajopt_seeds=c.plan.trajopt_seeds,
            optimizer_collision_activation_distance=c.dist.act_new)
        if ik_tol is not None:
            pcfg.ik_solver_config.position_tolerance, pcfg.ik_solver_config.orientation_tolerance = (float(x) for x in ik_tol)
        self.planner = MotionPlanner(pcfg)
        self.planner.warmup(enable_graph=True)
        self.plan_cfg = c.plan
        assert list(self.planner.joint_names) == JOINTS, self.planner.joint_names
        home = self.planner.default_joint_state.clone().unsqueeze(0)
        goal = home.clone()
        goal.position[..., 0] += 0.2
        for _ in range(2):                      # 첫 계획이 느리지 않게
            self.planner.plan_cspace(goal, home, max_attempts=1)
        self.attempts = c.plan.attempts
        self.kin = self.cm.segmenter.kinematics
        # 움직이는 구 (받침 구는 바닥 위에 고정 → 바닥 면 검사에서 뺌): 두 자세에서 중심이 같은 구 = 고정.
        # [2026-10-14 고침] clone 필수: 기구학 결과는 같은 GPU 버퍼를 다시 써서, clone 없이 두 번 계산하면 두 결과가 같아져 60 개 모두
        #  '고정' → 바닥 면 검사가 처음부터 꺼져 있었다 (지도 격자 밖 로봇 뒤쪽에서 손목이 바닥 −26 mm, 이상적 모드 h_s3)
        s0, s1 = (self.kin.compute_kinematics(self.js(q)).robot_spheres.view(-1, 4).clone() for q in (np.zeros(6), np.full(6, 0.5)))
        self.moving_sph = (s0[:, :3] - s1[:, :3]).norm(dim=-1) > 1e-6
        assert int(self.moving_sph.sum()) > len(self.moving_sph) // 2, f'움직이는 구가 {int(self.moving_sph.sum())} 개뿐 (바닥 면 검사 고장)'
        self._sph_links = None                  # 구마다 링크 이름 (sphere_link_names, 처음 부를 때 읽음)
        # [2026-10-27 R-1] 거리 계산 상수 (예전: 부를 때마다 CPU → GPU 로 새로 만듦) + float32 지도 기억 (지도 판마다 한 번 바꿈)
        self._org_c = torch.tensor(self.cm.s.origin_m, device='cuda') + 0.5 * self.cm.s.esdf_voxel_size     # 0번 칸 중심
        self._nm1 = torch.tensor(self.cm.s.shape, device='cuda', dtype=torch.float32) - 1
        self._vol = None                        # (지도 판, {이름: 몇 번째}, float32 [지도, x, y, z])
        self.last_near = {}                     # 마지막 영상 (frame) 시각 관절값의 진단 거리 (near) — rt_run 영상 기록
        self.voxel_cut = 0                      # 마지막 voxel_code 가 maxn 을 넘어 버린 칸 수 (기록에 남김)
        log(f'지도·계획기 준비 {time.time() - t0:.1f}초 (격자 {s.shape}, 카메라 받침 기준 {np.round(T[:3, 3], 3)})')

    def _empty_grid(self) -> VoxelGrid:
        s = self.cm.s
        n, vs = s.shape, s.esdf_voxel_size
        center = [o + k * vs / 2 for o, k in zip(s.origin_m, n)]
        return VoxelGrid(name='world_voxel', pose=center + [1.0, 0.0, 0.0, 0.0], dims=[k * vs for k in n], voxel_size=vs,
                         feature_tensor=torch.full((int(np.prod(n)), 1), 1.0, device='cuda', dtype=torch.float16))

    def js(self, q) -> JointState:
        return self.kin.get_active_js(JointState.from_position(
            torch.tensor(np.asarray(q, np.float32).reshape(1, -1), device='cuda'), joint_names=JOINTS))

    def _spheres_js(self, js):
        """기구학 입력 js 의 쓰는 충돌 구 (반지름 > 0) [S', 4] (새 텐서 — 기구학 버퍼와 따로)."""
        sph = self.kin.compute_kinematics(js).robot_spheres.view(-1, 4)
        return sph[sph[:, 3] > 0]

    def _spheres(self, q):
        return self._spheres_js(self.js(q))

    def frame(self, q, boxes):
        """영상 한 장: 관절 q 의 로봇 + 상자들 [(중심, 크기, wxyz)] 을 렌더 → 지도에 쌓기 → MPC 지도 다시 계산 (integrate_depth) + q 의 진단
        거리 (last_near). (쌓였나, ms)
        [검증 전용] oracle 이면 렌더·쌓기 대신 참 형상: boxes[0] = 정지 기둥, 나머지 = 움직이는 물체 (rt_run.Controller._boxes 순서)."""
        t0 = time.perf_counter()
        if self.oracle:
            assert all(list(b[2]) == [1, 0, 0, 0] for b in boxes), '참 형상 지도는 축 정렬 상자만'
            self.cm.set_oracle(boxes[:1], boxes[1:])
            self.cm.esdf_mpc()
            ok, self.last_near = True, self.near(q)
        else:
            ok, self.last_near = self.integrate_depth(self.cam.render(q, boxes)[0], q, q)
        torch.cuda.synchronize()                # [2026-10-27 R-1] GPU 일을 다 기다린 뒤 시간 (진단을 한 번에 읽어 그 전에 덜 기다리므로)
        return ok, (time.perf_counter() - t0) * 1000

    def integrate_depth(self, depth_m, q_mask, q_ctl):
        """[2026-10-27 R-1] 깊이 한 장 (m, [높이, 너비] GPU float32) → 지도: 팔 지우기 = 관절값 q_mask (영상이 찍힌 순간의 팔), 경계 채우기
        (cr3a_mapper mpc_boundary_fill) = q_ctl (움직이는 = 제어하는 로봇), MPC 지도·실제 거리 지도 다시 계산, q_ctl 의 진단 거리.
        반환 (쌓였나 — 깊이 있는 픽셀이 너무 적으면 안 쌓음, near dict). 가상 카메라 (frame) 와 GUI 서버의 실제 카메라가 같이 씀.
        [2026-10-27 R-1] 같은 자세면 기구학 입력 한 번, 경계 채우기 구 = 진단 구 (경계 채우기가 구한 칸 거리를 진단이 다시 씀 — near_diag)."""
        js = self.js(q_mask)
        ok = self.cm.integrate(depth_m, js)
        sph = self._spheres_js(js if q_ctl is q_mask else self.js(q_ctl))     # 쌓은 뒤에 (팔 지우기가 같은 기구학 버퍼를 씀)
        self.cm.boundary_spheres = sph
        self.cm.esdf_mpc()
        return ok, self.cm.near_diag(sph) or {}

    def near(self, q) -> dict:
        """(진단) 관절 q 의 로봇 구에서 MPC 지도·새 물체 칸까지 거리 (cr3a_mapper.CameraMap.near_diag)."""
        return self.cm.near_diag(self._spheres(q)) or {}

    def spheres(self, Q):
        """관절값 [n, 6] → 충돌 구 중심 [n, S, 3]·반지름 [n, S] (GPU, cuRobo 차례 — 반지름 0 = 안 쓰는 구 포함)."""
        js = JointState.from_position(torch.as_tensor(np.asarray(Q, np.float32), device='cuda'), joint_names=JOINTS)
        sph = self.kin.compute_kinematics(self.kin.get_active_js(js)).robot_spheres.view(len(Q), -1, 4)
        return sph[..., :3], sph[..., 3]

    def sphere_link_names(self) -> list:
        """충돌 구 (spheres 차례, 반지름 0 포함) 마다 링크 이름 — 진단 글의 '가장 가까운 링크'. 못 읽으면 빈 목록."""
        if self._sph_links is None:
            try:
                kp = self.kin.kinematics_config
                names = {int(i): n for n, i in kp.link_name_to_idx_map.items()}
                self._sph_links = [names.get(int(i), '?') for i in kp.link_sphere_idx_map.cpu().numpy()]
            except Exception:
                self._sph_links = []
        return self._sph_links

    def _grid_pts(self, c):
        """구 중심 c [n, S, 3] → grid_sample 좌표 [1, 1, 1, n·S, 3] ((z, y, x) 차례, [-1, 1]) 와 격자 안 [n, S]."""
        g = (c - self._org_c) / self._nm1 / self.cm.s.esdf_voxel_size * 2 - 1    # [-1, 1] (x, y, z)
        return g.flip(-1).view(1, 1, 1, -1, 3), (g.abs() <= 1).all(-1)

    def _sample(self, c, r, vol, floor):
        """구 (중심 c [n, S, 3], 반지름 r [n, S]) 와 거리 지도 vol [x, y, z] (+ floor 면 바닥 면 z = floor_z) 의 최소 거리 [n] (GPU).
        지도는 칸 중심 사이 3선형 보간 (MPC 와 같은 방식), 격자 밖은 바닥 면만. (지도 하나 — 여러 지도는 clearances 가 한 번에)"""
        gp, inside = self._grid_pts(c)
        d = torch.nn.functional.grid_sample(vol[None, None], gp, mode='bilinear', padding_mode='border', align_corners=True).view(c.shape[:-1])
        d = torch.where(inside, d, torch.full_like(d, FAR_M)) - r
        if floor:
            d = torch.minimum(d, torch.where(self.moving_sph, c[..., 2] - r - self.floor_z, torch.full_like(d, FAR_M)))   # 바닥 면 (움직이는 구만)
        d = torch.where(r > 0, d, torch.full_like(d, FAR_M))
        return d.min(dim=1).values

    def _vols(self):
        """float32 지도 묶음 [지도, x, y, z] 와 이름 → 몇 번째 — 지도 판 (CameraMap.version) 마다 한 번만 바꿔 둠 (예전: 거리 계산마다 지도마다
        float16 → float32). 카메라 지도는 실제 거리 = 새 물체 지도 (같은 값) 라 'dyn' 은 'raw' 를 씀."""
        cm = self.cm
        if self._vol is None or self._vol[0] != cm.version:
            names = ('raw', 'mpc') if cm.raw_is_dyn else ('raw', 'dyn', 'mpc')
            src = {'raw': cm.raw_grid, 'dyn': cm.dyn_grid, 'mpc': cm.mpc_grid}
            V = self._vol[2] if (self._vol is not None and self._vol[2].shape[0] == len(names)) else \
                torch.empty((len(names), *cm.s.shape), device='cuda', dtype=torch.float32)
            for i, g in enumerate(names):
                V[i].copy_(src[g]())
            slot = {g: i for i, g in enumerate(names)}
            slot.setdefault('dyn', slot['raw'])
            self._vol = (cm.version, slot, V)
        return self._vol[1], self._vol[2]

    def clearance(self, Q, grid='raw') -> np.ndarray:
        """관절값 [n, 6] 마다 충돌 구의 최소 거리 [n] (m, 0 아래 = 충돌). grid = 'raw' 실제 거리 지도 + 바닥 면 z = floor_z,
        'dyn' 새 물체 지도 (카메라 = 실제 거리 지도와 같은 값) 만 (바닥 면 없음)."""
        g = 'raw' if grid == 'raw' else 'dyn'
        return self.clearances(Q, (g,))[g]

    def clearances(self, Q, grids=('raw', 'dyn'), floor=True) -> dict:
        """관절값 [n, 6] 마다 지도별 최소 거리 {이름: [n]} — 기구학·GPU 기다림 한 번. 'raw' 실제 거리 + 바닥 면, 'dyn' 새 물체 (바닥 면
        없음), 'mpc' MPC 가 보는 지도 (팔 지우기 경계 채우기 포함) + 바닥 면. floor = False 면 어느 지도도 바닥 면 없이 (지도만 — GUI 지도 점검:
        바닥은 MPC 장면에 따로 있고 책상 가까이 일하는 자세를 막지 않게).
        [2026-10-27 R-1] 한 번에: 격자 좌표·격자 안·바닥 면·쓰는 구는 한 번, 3선형 보간은 지도들을 채널로 묶어 grid_sample 한 번 (지도마다
        _sample 과 같은 계산 — 값 같음)."""
        c, r = self.spheres(Q)
        slot, V = self._vols()
        gp, inside = self._grid_pts(c)
        D = torch.nn.functional.grid_sample(V[None], gp, mode='bilinear', padding_mode='border',
                                            align_corners=True).view(V.shape[0], *c.shape[:-1])
        D = torch.where(inside, D, FAR_M) - r                                     # [지도, n, S]
        fl = torch.where(self.moving_sph, c[..., 2] - r - self.floor_z, FAR_M) if floor else None    # 바닥 면 (움직이는 구만)
        used = r > 0
        out = []
        for g in grids:
            d = D[slot[g]]
            if fl is not None and g != 'dyn':
                d = torch.minimum(d, fl)
            out.append(torch.where(used, d, FAR_M).min(dim=1).values)
        d = torch.stack(out).cpu().numpy()
        return dict(zip(grids, d))

    def grid_info(self) -> dict:
        """(기록용) 지도 칸 번호를 위치로 풀 때 쓰는 값: 0 번 칸 모서리 (m), 칸 수 [x, y, z], 칸 크기 (m)."""
        s = self.cm.s
        return {'origin': [round(float(v), 5) for v in s.origin_m], 'shape': [int(v) for v in s.shape], 'voxel': float(s.esdf_voxel_size)}

    def voxel_code(self, grid='dyn', zmin=None, maxn=30000) -> str:
        """(기록용) 지도에서 막힌 칸 (거리 ≤ 칸/2) 을 칸 번호로 묶은 글자 (pack_voxels). grid = 'dyn' 새 물체 (켤 때 기다림이 끝난 뒤),
        'raw' 실제 거리 지도. zmin (m) 보다 중심이 낮은 칸은 뺌 (바닥 아래 = 바닥 뒤로 가려진 칸). 칸이 maxn 넘으면 앞 (번호 작은) maxn 개만 —
        [2026-10-27 R-1] 버린 칸 수는 voxel_cut 에 (예전: 말없이 잘림 — 실제 카메라는 책상·벽까지 보여 넘을 수 있음)."""
        s, vs = self.cm.s, self.cm.s.esdf_voxel_size
        self.voxel_cut = 0
        if grid == 'dyn' and not self.cm.bg_ready:
            return ''
        if grid == 'dyn' and zmin is None:      # [R-1] 영상마다 구해 둔 새 물체 칸 (CameraMap._occupied) 을 다시 씀
            flat = self.cm.occupied_flat()
        else:
            occ = (self.cm.dyn_grid() if grid == 'dyn' else self.cm.raw_grid()).float() <= 0.5 * vs
            if zmin is not None:
                occ[:, :, :max(0, int(np.ceil((zmin - s.origin_m[2]) / vs - 0.5)))] = False
            flat = torch.nonzero(occ.reshape(-1)).view(-1)
        self.voxel_cut = max(0, int(flat.shape[0]) - maxn)
        return pack_voxels(flat[:maxn].cpu().numpy())

    def plan(self, q0, q1, solve=None):
        """MPC 지도로 q0 → q1 전역 계획, 실제 거리 지도로 검증. ((시각 [N], 관절 [N, 6]) 또는 None, 걸린 시간 s).
        계획기는 MPC 와 같은 지도 (경계 채우기 포함) 와 같은 반응 거리로 풀고, 구 근사로 '충돌 없음' 이어도 실제 거리 지도에서 닿으면 실패
        (출발 자세가 이미 닿아 있으면 그대로 받음: 빠져나오는 경로).
        [2026-10-16] 세 단계로 나눔 — 멈추지 않는 다시 계획 (rt_run, rt_config.PlanCfg (나)) 은 plan_solve 만 다른 스레드에서 부름.
        solve = 풀기 함수 (q0, q1) → 경로 또는 None (기본 self.plan_solve) — [2026-10-27 R-1] rt_run.Controller._solve (계획 뒤 고치기 자리
        _post_solve 를 거침)."""
        t0 = time.perf_counter()
        self.plan_prep()
        path = (solve or self.plan_solve)(q0, q1)
        wall = time.perf_counter() - t0
        if path is None:
            return None, wall
        if self.plan_check(path) is None:
            return None, time.perf_counter() - t0
        return path, wall

    def plan_prep(self, grid='mpc', q0=None, clear_m=0.015):
        """계획기 장면에 지금 지도를 넣음 — 계획기 자기 버퍼로 복사됨 (cuRoboV2 SceneVoxelData.load_batch: features[...] = …)
        → 계획이 도는 동안 지도가 다시 계산돼도 계획은 이때 지도로 풂. grid = 'mpc' (MPC 지도, 경계 채우기 포함 — 멈춰서 계획 plan)
        | 'start' (MPC 지도에서 관절값 q0 의 충돌 구 겉면 clear_m 안 칸만 실제 거리 — 멈추지 않는 다시 계획·GUI 공구 끝 IK,
        rt_config.PlanCfg.live_clear_m 설명). [2026-10-27 R-1] 'raw' (실제 거리 지도, 되돌리기 전용) 는 뺌"""
        assert grid in ('mpc', 'start'), f'plan_prep grid = {grid!r}'
        vg = self.cm.esdf_mpc()
        feat = None
        if grid == 'start':
            feat = vg.feature_tensor.clone()
            c, r = self.spheres(np.asarray(q0, float)[None])
            c, r = c[0][r[0] > 0], r[0][r[0] > 0]
            s = self.cm.s
            n, vs = s.shape, s.esdf_voxel_size
            org = torch.tensor(s.origin_m, device='cuda', dtype=torch.float32)
            lo = torch.clamp(torch.floor(((c - r[:, None]).min(0).values - clear_m - org) / vs).long(), min=0)
            hi = torch.minimum(torch.floor(((c + r[:, None]).max(0).values + clear_m - org) / vs).long() + 1,
                               torch.tensor(n, device='cuda'))
            axes = [torch.arange(int(lo[k]), int(hi[k]), device='cuda') for k in range(3)]
            idx = torch.stack(torch.meshgrid(*axes, indexing='ij'), dim=-1).view(-1, 3)
            P = org + (idx.float() + 0.5) * vs
            near = cmap.min_dist(P, c, r) < clear_m        # [R-1] TF32 와 상관없는 거리 (cr3a_mapper.min_dist)
            flat = (idx[near, 0] * n[1] + idx[near, 1]) * n[2] + idx[near, 2]
            fv = feat.view(-1)
            fv[flat] = self.cm._raw.view(-1)[flat].to(fv.dtype)
        if feat is not None:
            vg = VoxelGrid(name=vg.name, pose=list(vg.pose), dims=list(vg.dims), voxel_size=vg.voxel_size, feature_tensor=feat)
        self.planner.update_world(Scene(cuboid=[self.floor], voxel=[vg]).get_collision_check_world())

    def plan_solve(self, q0, q1, seed_path=None):
        """(계획기만 씀 — 다른 스레드에서 불러도 됨) q0 → q1 → (시각 [N], 관절 [N, 6]) 또는 None. 먼저 고른 시작값으로 (_solve_seeded,
        rt_config.PlanCfg ③), 모두 실패면 cuRobo 기본 (plan_cspace). seed_path = 지금 따라가는 경로의 남은 부분 [M, 6] (멈추지 않는 다시
        계획의 시작값으로)."""
        start = JointState.from_position(torch.tensor([list(q0)], device='cuda', dtype=torch.float32), joint_names=JOINTS)
        goal = JointState.from_position(torch.tensor([list(q1)], device='cuda', dtype=torch.float32), joint_names=JOINTS)
        r = self._solve_seeded(start, goal, np.asarray(q0, float), np.asarray(q1, float), seed_path)
        if r is not None:
            return r
        r = self.planner.plan_cspace(goal, start, max_attempts=self.attempts)
        if r is None or not bool(r.success.any()):
            return None
        sol = r.js_solution
        pos = sol.position.reshape(-1, 6).cpu().numpy().astype(float)
        dt = float(sol.dt.view(-1)[0])
        return np.arange(len(pos)) * dt, pos

    def _solve_seeded(self, start, goal, q0, q1, seed_path):
        """[2026-10-16 ③ rt_config.PlanCfg, 작업기록 5-22] 시작값을 골라 cuRobo trajopt 를 한 번 풀고 (후보 = 시작값 수, 모자라면 cuRobo 가 곧은
        선으로 채움), 성공한 후보 중 관절 움직임 (rad 합) 이 가장 적은 것 → (시각, 관절) 또는 None (모두 실패: 부르는 쪽이 cuRobo 기본으로).
        시작값 [H 점, q0 → q1]: 곧은 선, 지금 경로의 남은 부분 (있으면, 관절 거리로 고르게 다시 나눔), J2 ± / J1 ± 가운데 볼록 (sin, plan_bump_rad).
        cuRobo 기본 첫 시도는 시작값 6 개가 모두 같은 곧은 선이라 (archive/dev/plan_seeds_probe.py) 막히면 그래프 씨앗 (무작위 길) 로 넘어갔음."""
        ts = self.planner.trajopt_solver
        H, N = ts.action_horizon, ts.config.num_seeds
        s = np.linspace(0.0, 1.0, H)[:, None]
        line = q0 + s * (q1 - q0)
        seeds = [line]
        if seed_path is not None and len(seed_path) >= 2:
            P = np.vstack([q0, np.asarray(seed_path, float), q1])
            step = np.abs(np.diff(P, axis=0)).max(1)
            P = np.vstack([P[:1], P[1:][step > 1e-6]])                 # 겹친 점 빼기 (np.interp 는 늘어나는 거리만)
            d = np.concatenate([[0.0], np.cumsum(np.abs(np.diff(P, axis=0)).max(1))])
            if len(P) >= 2 and d[-1] > 1e-6:
                seeds.append(np.stack([np.interp(s[:, 0] * d[-1], d, P[:, j]) for j in range(6)], 1))
        bump = np.sin(np.pi * s[:, 0])
        for j, sign in ((1, -1.0), (1, 1.0), (0, 1.0), (0, -1.0)):     # J2 넘어가기 위·아래, J1 돌아가기 양쪽
            if len(seeds) >= N:
                break
            v = line.copy()
            v[:, j] += sign * self.plan_cfg.plan_bump_rad * bump
            seeds.append(v)
        S = torch.tensor(np.stack(seeds), device='cuda', dtype=torch.float32).view(1, len(seeds), H, 6)
        r = ts.solve_cspace(goal, start, seed_traj=S, return_seeds=N, finetune_attempts=3, finetune_dt_scale=0.75)
        ok = r.success.view(-1).cpu().numpy().astype(bool)
        if not ok.any():
            return None
        P = r.js_solution.position.view(N, -1, 6).cpu().numpy().astype(float)
        dt = r.js_solution.dt.view(-1).cpu().numpy().astype(float)
        travel = np.abs(np.diff(P, axis=1)).sum((1, 2))
        travel[~ok] = np.inf
        k = int(np.argmin(travel))
        return np.arange(P.shape[1]) * float(dt[min(k, len(dt) - 1)]), P[k]

    def plan_check(self, path):
        """실제 거리 지도 (지금) 로 경로 검증: 닿으면 None (출발 자세가 이미 닿아 있으면 그대로: 빠져나오는 경로)."""
        clr = self.clearance(path[1])
        if clr.min() < 0.0 and float(clr[0]) >= 0.0:
            return None
        return path
