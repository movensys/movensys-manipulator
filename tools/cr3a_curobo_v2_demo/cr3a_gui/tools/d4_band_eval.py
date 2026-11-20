#!/usr/bin/env python3
"""[2026-10-28 디버깅 D-4] 가림 띠 거르기 (depth_filters) 오프라인 평가 — 이미지 기록 (snap) 의 깊이로. ★ wmx_r2 컨테이너 (cuRobo·GPU) ★

  python3 tools/d4_band_eval.py side  <snap 폴더> …             가림 띠가 로봇 어느 쪽인가 (깊이 없음 비율·유령 점 위치)
  python3 tools/d4_band_eval.py eval  <결과 폴더> <snap 폴더> …   거르기마다: 팔 옆 점 (영상마다) + 지도 (cr3a_mapper 그대로) 점검
  python3 tools/d4_band_eval.py delay <결과 폴더> <snap 폴더>      가짜 물체를 넣어 지도에 잡히는 늦음 (정지·옆으로·다가옴)
  python3 tools/d4_band_eval.py bench <snap 폴더>                 서버에서 한 장 거르는 시간 (GPU, segmenter 팔 지우기 포함)
eval 은 <결과 폴더>/eval.json · 그림 (전·후 나란히) · summary.png 를 씀.
'팔 거리' = cuRobo 충돌 구 겉면까지 (팔 지우기·지도 점검과 같은 기준). 지도 점검 걸림 = 지도에서 구 겉면 5.5 cm 안 칸이 있거나 MPC 지도
거리 < 0 (GuiController._map_check 와 같음 — 앞 SKIP 장은 뺌).
"""
import glob
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(HERE), str(HERE.parent / 'cr3a_rt_shaping')]
import cv2  # noqa: E402
import torch  # noqa: E402

import depth_filters as dfl  # noqa: E402
import snap_log  # noqa: E402

SKIP = 5                    # 앞 장 수 (시간 확인 n 장이 차고 지도가 쌓이는 시간) — 그 뒤 장으로 판단
NEAR_M, MASK_M, FAR_M = 0.06, 0.04, 0.15
DEV = 'cuda'
VARIANTS = {                # 이름: (시간 확인 (k, n, tol) | None, 가림 띠 side | None). A = 깊이 가운데값 ±20 mm 로 셈 (lead 안),
    #  A' = 깊이 있음만 셈 (깊이가 바뀌는 물체 — 다가오는 손 — 를 안 지움)
    'none': (None, None),
    'A 3/4': ((3, 4, 0.02), None),
    'A 4/5': ((4, 5, 0.02), None),
    "A' 3/4": ((3, 4, None), None),
    "A' 4/5": ((4, 5, None), None),
    "A' 4/4": ((4, 4, None), None),
    'B': (None, 'left'),
    'A 3/4+B': ((3, 4, 0.02), 'left'),
    'A 4/5+B': ((4, 5, 0.02), 'left'),
    "A' 3/4+B": ((3, 4, None), 'left'),
    "A' 4/5+B": ((4, 5, None), 'left'),
    "A' 4/4+B": ((4, 4, None), 'left'),
    'B+': (None, 'left+'),
    "A' 4/5+B+": ((4, 5, None), 'left+'),
    'B+a': (None, 'left+a'),
    "A' 4/5+B+a": ((4, 5, None), 'left+a'),
    "A' 5/5+B+a": ((5, 5, None), 'left+a'),
    "A' 4/5+B+a2D15": ((4, 5, None), 'left+a2d:15'),
    "A' 4/5+B+a2D30": ((4, 5, None), 'left+a2d:30'),
    "A' 4/5+B+a2D45": ((4, 5, None), 'left+a2d:45'),
}


def load(d):
    d = Path(d)
    job = json.loads((d / '_job.json').read_text())
    z = np.load(d / '_job.npz')
    fr = [json.loads(Path(f).read_text()) for f in sorted(glob.glob(str(d / 'frame_*.json')))]
    D = [cv2.imread(str(d / f['depth']), cv2.IMREAD_UNCHANGED) for f in fr]
    return {'dir': d, 'name': d.name, 'job': job, 'cfg': job['cfg'], 'K': np.array(job['cam']['K'], float),
            'T': np.array(job['cam']['T_base_depth'], float), 'cam': job['cam'], 'frames': fr, 'D': D,
            'Q': np.radians([f['q_mask_deg'] for f in fr]), 'C': z['C'][:len(fr)], 'R': z['R'][:len(fr)],
            'snap': json.loads((d / 'snap.json').read_text())}


def points(depth_m, K, T):
    v, u = torch.nonzero(depth_m > 0, as_tuple=True)
    zz = depth_m[v, u]
    Kt = torch.as_tensor(K, dtype=torch.float32, device=DEV)
    Tt = torch.as_tensor(T, dtype=torch.float32, device=DEV)
    pc = torch.stack([(u - Kt[0, 2]) * zz / Kt[0, 0], (v - Kt[1, 2]) * zz / Kt[1, 1], zz], 1)
    return v, u, pc @ Tt[:3, :3].T + Tt[:3, 3]


def sph_dist(P, c, r, chunk=20000):
    c = torch.as_tensor(c, dtype=torch.float32, device=DEV)
    r = torch.as_tensor(r, dtype=torch.float32, device=DEV)
    d = torch.empty(len(P), device=DEV)
    k = torch.empty(len(P), dtype=torch.long, device=DEV)
    for i in range(0, len(P), chunk):
        D = ((P[i:i + chunk, None, :] - c[None]) ** 2).sum(-1).sqrt() - r
        d[i:i + chunk], k[i:i + chunk] = D.min(1)
    return d, k


def excluded(P, cfg):
    """지도에서 빼는 점 (바닥 띠·받침 자리·받침 둘레) — cr3a_mapper.integrate 와 같음."""
    ex = P[:, 2] <= cfg['floor_z']
    if cfg.get('base_exclude'):
        x0, y0, z0, x1, y1, z1 = cfg['base_exclude']
        ex |= (P[:, 0] > x0) & (P[:, 0] < x1) & (P[:, 1] > y0) & (P[:, 1] < y1) & (P[:, 2] > z0) & (P[:, 2] < z1)
    if cfg.get('base_ring'):
        rr, zz = cfg['base_ring']
        ex |= (P[:, 0] ** 2 + P[:, 1] ** 2 < rr * rr) & (P[:, 2] < zz)
    return ex


def speck_m(D, cfg):
    d, _ = snap_log.drop_specks(D, *cfg['speck'])
    return torch.from_numpy(d.astype(np.float32)).to(DEV).mul_(0.001)


def filtered(cap, i, variant, D_list=None, robot_cache=None):
    """i 번째 장을 거른 깊이 (m, GPU) — 서버 순서: 외딴 조각 지우기 → 시간 확인 (원래 깊이 n 장) → 가림 띠 (팔 지우기 픽셀 기준)."""
    D_list = cap['D'] if D_list is None else D_list
    temporal, side = VARIANTS[variant] if isinstance(variant, str) else variant
    d = speck_m(D_list[i], cap['cfg'])
    if temporal is not None:
        k, n, tol = temporal
        st = torch.stack([torch.from_numpy(D_list[j].astype(np.float32)).to(DEV).mul_(0.001) if j >= 0 else torch.zeros_like(d)
                          for j in range(i - n + 1, i + 1)])
        d = d * dfl.temporal_keep(st, k, tol)
    if side is not None:
        robot = robot_mask(cap, i % len(cap["R"]), d, robot_cache)
        rows = int(side.split('2d:')[1]) if '2d:' in side else 0
        d = d * ~dfl.shadow_band(d, robot, float(cap['K'][0, 0]), side.split('+')[0], behind='+' in side,
                                 adaptive='+a' in side, ref_rows=rows)
    return d


def robot_mask(cap, i, d, cache=None):
    keep = cap['R'][i] > 0
    return dfl.robot_mask_from_spheres(d, cap['K'], cap['T'], cap['C'][i][keep], cap['R'][i][keep], MASK_M)


def frame_stats(cap, i, d):
    """거른 깊이 → 팔 옆 점 (지우기 밖·지도 빼기 밖·구 겉면 6 cm 안) 수·최소 거리, 먼 실제 점 (15 cm 밖) 수."""
    v, u, P = points(d, cap['K'], cap['T'])
    keep = cap['R'][i] > 0
    dist, _ = sph_dist(P, cap['C'][i][keep], cap['R'][i][keep])
    ok = ~excluded(P, cap['cfg'])
    near = ok & (dist >= MASK_M) & (dist < NEAR_M)
    return {'near': int(near.sum()), 'near_min_mm': round(float(dist[near].min()) * 1000, 1) if bool(near.any()) else None,
            'far': int((ok & (dist > FAR_M)).sum()), 'near_uv': (v[near], u[near]), 'near_P': P[near]}


# ---------------- 지도 (cr3a_mapper 그대로) ----------------
class MapRig:
    def __init__(self, cap0):
        import rt_run  # noqa: F401  (cuRobo 실행 설정 먼저 — gui_controller 와 같음)
        import cr3a_mapper as cmap
        import depth_sim as ds
        from rt_config import Config, TaskCfg
        from rt_world import robot_cfg
        c = Config(task=TaskCfg(), style='carrot', model='tool')
        cfg = cap0['cfg']
        self.s = cmap.MapSettings(center_m=list(c.map.center), size_m=list(c.map.size), esdf_voxel_size=c.map.voxel,
                                  segment_distance_threshold=c.dist.mask, base_exclude=tuple(cfg['base_exclude']),
                                  base_ring=tuple(cfg['base_ring']), mpc_floor_z_m=cfg['floor_z'])
        self.cm = cmap.CameraMap(self.s, robot_cfg(c.model))
        self.cm.warmup()
        self.kin = self.cm.segmenter.kinematics
        self.cmap, self.ds = cmap, ds
        self.fill_b = self.s.segment_distance_threshold + 1.5 * self.s.esdf_voxel_size

    def reset(self, cap):
        cm = self.cm
        cm.mapper_mpc.reset()
        cm.set_camera(cap['K'].tolist(), self.ds.pose7(cap['T']))
        cm._dirty = True
        cm.boundary_spheres = None
        cm._bg_left, cm.bg_ready = 0, True

    def js(self, q):
        from curobo.types import JointState
        from rt_world import JOINTS
        return self.kin.get_active_js(JointState.from_position(torch.tensor(np.asarray(q, np.float32)[None], device=DEV),
                                                               joint_names=JOINTS))

    def step(self, d, q):
        """한 장 쌓기 → (지도 점검 칸 B [n, 3] (구 겉면 5.5 cm 안), MPC 지도 거리 mm, 먼 칸 수)."""
        cm = self.cm
        js = self.js(q)
        cm.integrate(d, js)
        sph = self.kin.compute_kinematics(js).robot_spheres.view(-1, 4)
        sph = sph[sph[:, 3] > 0].clone()
        cm.boundary_spheres = sph
        cm.esdf_mpc()
        nd = cm.near_diag(sph) or {}
        P = cm.new_voxel_centers()
        if P.shape[0] == 0:
            return torch.zeros((0, 3), device=DEV), nd.get('near_mm'), 0, P
        dm = self.cmap.min_dist(P, sph[:, :3].float(), sph[:, 3].float())
        return P[dm <= self.fill_b], nd.get('near_mm'), int((dm > FAR_M).sum()), P


def run_map(rig, cap, variant, D_list=None, Q=None):
    D_list = cap['D'] if D_list is None else D_list
    Q = cap['Q'] if Q is None else Q
    rig.reset(cap)
    out = []
    for i in range(len(D_list)):
        d = filtered(cap, i, variant, D_list)
        B, mpc, far, P = rig.step(d, Q[min(i, len(Q) - 1)])
        out.append({'B': B, 'mpc_mm': mpc, 'far_vox': far, 'P': P, 'd': d})
    return out


# ---------------- 명령 ----------------
def cmd_side(dirs):
    for dd in dirs:
        cap = load(dd)
        zl = zr = 0.0
        nl = nr = 0
        ph_l = ph_r = 0
        for i, D in enumerate(cap['D']):
            d = torch.from_numpy(D.astype(np.float32)).to(DEV).mul_(0.001)
            robot = robot_mask(cap, i, d)
            rb = robot.cpu().numpy()
            dn = D
            h, w = rb.shape
            left_edge = rb & ~np.roll(rb, 1, 1)
            right_edge = rb & ~np.roll(rb, -1, 1)
            for (ys, xs), side in ((np.nonzero(left_edge), 'L'), (np.nonzero(right_edge), 'R')):
                for y, x in zip(ys[::7], xs[::7]):
                    zrm = dn[y, x] / 1000.0
                    W = int(min(25, 0.095 * cap['K'][0, 0] / max(zrm, 0.2)))
                    seg = dn[y, max(0, x - W):x] if side == 'L' else dn[y, x + 1:x + 1 + W]
                    segr = rb[y, max(0, x - W):x] if side == 'L' else rb[y, x + 1:x + 1 + W]
                    seg = seg[~segr]
                    if len(seg) == 0:
                        continue
                    if side == 'L':
                        zl += float(np.mean(seg == 0)); nl += 1
                    else:
                        zr += float(np.mean(seg == 0)); nr += 1
            st = frame_stats(cap, i, speck_m(D, cap['cfg']))
            vv, uu = (t.cpu().numpy() for t in st['near_uv'])
            for y, x in zip(vv, uu):
                cols = np.nonzero(rb[y])[0]
                if len(cols) == 0:
                    continue
                k = cols[np.argmin(np.abs(cols - x))]
                if k > x:
                    ph_l += 1
                else:
                    ph_r += 1
        print(f'{cap["name"]:22s} 로봇 윤곽 옆 깊이 없음 비율: 왼쪽 {zl / max(nl, 1):.2f} · 오른쪽 {zr / max(nr, 1):.2f} | '
              f'팔 옆 점이 로봇의 왼쪽 {ph_l} · 오른쪽 {ph_r}')


def cmd_eval(out, dirs):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    caps = [load(d) for d in dirs]
    rig = MapRig(caps[0])
    res = {}
    keep_maps = {}
    for cap in caps:
        res[cap['name']] = {}
        for vn in VARIANTS:
            t0 = time.time()
            fs = [frame_stats(cap, i, filtered(cap, i, vn)) for i in range(len(cap['D']))]
            mp = run_map(rig, cap, vn)
            ev = range(SKIP, len(cap['D']))
            nB = [int(mp[i]['B'].shape[0]) for i in ev]
            mpc = [mp[i]['mpc_mm'] for i in ev if mp[i]['mpc_mm'] is not None]
            fail = [i for i in ev if mp[i]['B'].shape[0] > 0 or (mp[i]['mpc_mm'] is not None and mp[i]['mpc_mm'] < 0)]
            mins = [fs[i]['near_min_mm'] for i in ev if fs[i]['near_min_mm'] is not None]
            res[cap['name']][vn] = {
                'near_pts_mean': round(float(np.mean([fs[i]['near'] for i in ev])), 1), 'near_pts_max': max(fs[i]['near'] for i in ev),
                'near_frames': sum(fs[i]['near'] > 0 for i in ev), 'near_min_mm': min(mins) if mins else None,
                'far_pts_mean': round(float(np.mean([fs[i]['far'] for i in ev])), 0),
                'map_B_max': max(nB), 'map_fail_frames': len(fail), 'n_eval': len(ev), 'mpc_min_mm': min(mpc) if mpc else None,
                'far_vox_last': mp[-1]['far_vox'], 'secs': round(time.time() - t0, 1)}
            if vn in IMG_PAIR:
                keep_maps[(cap['name'], vn)] = (fs, [{'B': m['B'].cpu().numpy()} for m in mp])
            r = res[cap['name']][vn]
            print(f'{cap["name"]:22s} {vn:9s} 팔옆점 평균 {r["near_pts_mean"]:7.1f} 최대 {r["near_pts_max"]:5d} (장 {r["near_frames"]:2d}/{len(ev)}) '
                  f'최소 {r["near_min_mm"]} mm | 지도 B 최대 {r["map_B_max"]:3d} 걸림 {r["map_fail_frames"]:2d}/{len(ev)} MPC 최소 {r["mpc_min_mm"]} | '
                  f'먼 점 {r["far_pts_mean"]:.0f} 먼 칸 {r["far_vox_last"]}', flush=True)
    (out / 'eval.json').write_text(json.dumps(res, ensure_ascii=False, indent=1))
    images(out, caps, keep_maps)
    summary_png(out, res)


IMG_PAIRS = (('none', "A' 4/5+B+a"), ('none', "A' 4/5+B+a2D30"), ('none', "A' 4/5+B+a2D45"))
IMG_PAIR = {v for p in IMG_PAIRS for v in p}


def images(out, caps, keep_maps, pairs=IMG_PAIRS):
    """전·후 나란히: 빨강 = 거른 뒤 팔 옆에 남은 점, 노랑 = 그 장까지 쌓은 지도에서 팔 5.5 cm 안 칸, 파랑 = 팔 지우기."""
    for cap in caps:
        for a, b in pairs:
            if (cap['name'], a) not in keep_maps or (cap['name'], b) not in keep_maps:
                continue
            fa, ma = keep_maps[(cap['name'], a)]
            # 보여 줄 장: 'none' 에서 지도 칸이 가장 많은 장 (없으면 팔 옆 점이 가장 많은 장), SKIP 뒤
            ev = list(range(SKIP, len(cap['D'])))
            i = max(ev, key=lambda j: (len(ma[j]['B']), fa[j]['near']))
            panels = []
            for vn in (a, b):
                fs, mp = keep_maps[(cap['name'], vn)]
                st = fs[i]
                col = cap['frames'][i]['color']
                img = cv2.imread(str(cap['dir'] / col)) if col else np.zeros((480, 640, 3), np.uint8)
                d = filtered(cap, i, vn)
                v, u, P = points(d, cap['K'], cap['T'])
                keep = cap['R'][i] > 0
                dist, k = sph_dist(P, cap['C'][i][keep], cap['R'][i][keep])
                ok = ~excluded(P, cap['cfg'])
                near = ok & (dist >= MASK_M) & (dist < NEAR_M)
                a_ = {'masked_P': P[dist < MASK_M].cpu().numpy(), 'near_P': P[near].cpu().numpy(), 'near_d': dist[near].cpu().numpy(),
                      'near_s': k[near].cpu().numpy(), 'n': {}}
                order = np.argsort(a_['near_d'])
                for kk in ('near_P', 'near_d', 'near_s'):
                    a_[kk] = a_[kk][order]
                names = cap['cfg']['link_names']
                sidx = np.nonzero(keep)[0]
                title = (f'{cap["name"]} #{i} [{vn}] near {len(a_["near_d"])} pts'
                         + (f' min {a_["near_d"][0] * 1000:.0f}mm' if len(a_['near_d']) else '') + f' | map voxels {len(mp[i]["B"])}')
                panels.append(snap_log.overlay(img, a_, np.array(cap['cam']['T_base_color']), np.array(cap['cam']['Kc']),
                                               cap['cam'].get('Dc'), names, sidx, vox=mp[i]['B'], title=title))
            cv2.imwrite(str(out / f'{cap["name"]}__{a}_vs_{b}.jpg'.replace(' ', '').replace('/', '-').replace("'", 'p')),
                        np.hstack(panels), [cv2.IMWRITE_JPEG_QUALITY, 90])


def summary_png(out, res):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = list(res)
    vns = [v for v in ('none', "A' 4/5", 'B', 'B+', 'B+a', "A' 4/5+B", "A' 4/5+B+", "A' 4/5+B+a", "A' 5/5+B+a", "A' 4/5+B+a2D15",
                       "A' 4/5+B+a2D30", "A' 4/5+B+a2D45") if v in res[names[0]]]
    vn_en = {'none': 'none', 'A 3/4': 'A 3/4 (+-20mm)', 'A 4/5': 'A 4/5 (+-20mm)', "A' 4/5": "A' 4/5 (valid only)", 'B': 'B band',
             'A 3/4+B': 'A 3/4 + B', "A' 4/5+B": "A' 4/5 + B", 'B+': 'B+ band (at/behind)', "A' 4/5+B+": "A' 4/5 + B+",
             'B+a': 'B+ adaptive width', "A' 4/5+B+a": "A' 4/5 + B+ adaptive (deployed)", "A' 5/5+B+a": "A' 5/5 + B+ adaptive",
             "A' 4/5+B+a2D15": "A' 4/5 + B+a + 2D ref 15 rows", "A' 4/5+B+a2D30": "A' 4/5 + B+a + 2D ref 30 rows",
             "A' 4/5+B+a2D45": "A' 4/5 + B+a + 2D ref 45 rows"}
    fig, ax = plt.subplots(3, 1, figsize=(14, 11))
    x = np.arange(len(names))
    wd = 0.8 / len(vns)
    for j, vn in enumerate(vns):
        ax[0].bar(x + j * wd, [res[n][vn]['near_pts_mean'] for n in names], wd, label=vn_en[vn])
        ax[1].bar(x + j * wd, [res[n][vn]['map_fail_frames'] for n in names], wd, label=vn_en[vn])
        base = [max(res[n]['none']['far_pts_mean'], 1) for n in names]
        ax[2].bar(x + j * wd, [100 * res[n][vn]['far_pts_mean'] / b for n, b in zip(names, base)], wd, label=vn_en[vn])
    ax[0].set_ylabel('points 4-6 cm from arm spheres\n(mean per frame)')
    ax[1].set_ylabel(f'map-check fail frames\n(of {res[names[0]]["none"]["n_eval"]})')
    ax[2].set_ylabel('real scene points >15 cm\n(% of none)')
    ax[2].set_ylim(80, 102)
    for a in ax:
        a.set_xticks(x + 0.4 - wd / 2)
        a.set_xticklabels([n.split('_', 1)[0] + '\n' + n.split('_', 1)[1] for n in names], fontsize=8)
        a.grid(axis='y', alpha=0.3)
    ax[0].legend(ncol=3, fontsize=8)
    ax[0].set_title('D-4 stereo shadow band filters on 13 snap captures (gui_real_20261028_150253)')
    fig.tight_layout()
    fig.savefig(out / 'summary.png', dpi=110)


DELAY_VARIANTS = ('none', "A' 4/5+B+a", "A' 5/5+B+a", "A' 4/5+B+a2D15", "A' 4/5+B+a2D30", "A' 4/5+B+a2D45")


def cmd_delay(out, d0):
    """가짜 물체: 15 장 기록을 두 번 이어 30 장 (로봇 정지 기록), 8 번째 장부터 물체 (50x50 px 네모, 배경보다 앞). 시나리오:
    정지 (1.0 m) · 옆으로 0.5 m/s · 카메라 쪽으로 0.5 m/s·1.0 m/s (1.4 m 에서). 거르기마다 물체 픽셀 남은 비율·지도에 처음 잡힌 장."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    cap = load(d0)
    D = cap['D'] + cap['D']
    Q = np.concatenate([cap['Q'], cap['Q']])
    fx = cap['K'][0, 0]
    rig = MapRig(cap)
    # 물체 자리: 로봇·바닥에서 먼 (구 겉면 25 cm 밖) 깊이가 1.6 m 넘는 곳 — 지도 격자 안
    d = torch.from_numpy(D[0].astype(np.float32)).to(DEV) / 1000
    v, u, P = points(d, cap['K'], cap['T'])
    keep = cap['R'][0] > 0
    dist, _ = sph_dist(P, cap['C'][0][keep], cap['R'][0][keep])
    t0 = 7
    # 이름: (깊이·옆 이동 함수, 나타나기 전 그 자리 깊이 없음 (검은·먼 배경 — 시간 확인 A' 는 이때만 늦음))
    scen = {'정지 1.0 m': (lambda f: (1.0, 0.0), False), '정지 1.0 m (배경 깊이 없음)': (lambda f: (1.0, 0.0), True),
            '옆으로 0.5 m/s': (lambda f: (1.0, 0.5 / 15 * (f - t0)), False), '옆으로 1.0 m/s': (lambda f: (1.0, 1.0 / 15 * (f - t0)), False),
            '다가옴 0.5 m/s': (lambda f: (1.4 - 0.5 / 15 * (f - t0), 0.0), False),
            '다가옴 1.0 m/s': (lambda f: (1.4 - 1.0 / 15 * (f - t0), 0.0), False)}
    # 물체 중심 픽셀: 깊이 > 1.6 m 이고 로봇에서 먼 곳 중 영상 가운데에 가까운 곳
    cand = (d[v, u] > 1.6) & (dist > 0.25)
    vv, uu = v[cand].cpu().numpy(), u[cand].cpu().numpy()
    j = np.argmin((vv - 240) ** 2 + (uu - 320) ** 2 * 0.3)
    v0, u0 = int(vv[j]), int(uu[j])
    T = cap['T']
    res = {}
    for sn, (fn, hole) in scen.items():
        frames = []
        boxes = []
        for f in range(len(D)):
            Df = D[f].copy()
            box = None
            if hole and f < t0:                        # 나타나기 전: 그 자리 깊이 없음
                half = int(round(0.06 * fx / 1.0))
                Df[v0 - half:v0 + half, u0 - half:u0 + half] = 0
            if f >= t0 and fn(f)[0] > 0.3:
                z, lat = fn(f)
                du = int(round(lat * fx / z))
                half = int(round(0.06 * fx / z))           # 12 cm 네모
                a0, a1, b0, b1 = v0 - half, v0 + half, u0 + du - half, u0 + du + half
                if 0 <= a0 and a1 < 480 and 0 <= b0 and b1 < 640:
                    # 가림 지킴: 물체보다 앞 (로봇 등) 은 그대로 → 로봇 뒤를 지나는 물체는 로봇에 가려짐 (보이는 픽셀만 셈)
                    sub = Df[a0:a1, b0:b1]
                    vis = (sub == 0) | (sub > np.uint16(z * 1000))
                    sub[vis] = np.uint16(z * 1000)
                    box = (a0, a1, b0, b1, z, torch.from_numpy(vis).to(DEV))
            frames.append(Df)
            boxes.append(box)
        res[sn] = {}
        for vn in DELAY_VARIANTS:
            mp = run_map(rig, cap, vn, frames, Q)
            first, kept = None, []
            for f, (m, box) in enumerate(zip(mp, boxes)):
                if box is None:
                    continue
                a0, a1, b0, b1, z, vis = box
                dd = m['d']
                if bool(vis.any()):
                    kept.append(float((dd[a0:a1, b0:b1][vis] > 0).float().mean()))
                # 물체 3D 범위 (픽셀 네모 → 받침 점) 안 지도 칸
                sub = torch.zeros_like(dd)
                sub[a0:a1, b0:b1] = z * vis
                _, _, Pb = points(sub, cap['K'], T)
                lo, hi = Pb.min(0).values - 0.03, Pb.max(0).values + 0.03
                Pm = m['P']
                n_in = int(((Pm >= lo) & (Pm <= hi)).all(1).sum()) if Pm.shape[0] else 0
                if first is None and n_in > 0:
                    first = f
            res[sn][vn] = {'first_frame': first, 'delay_ms': None if first is None else round((first - t0) * 1000 / 15, 0),
                           'kept_frac_mean': round(float(np.mean(kept)), 3) if kept else None,
                           'kept_frac_min': round(float(np.min(kept)), 3) if kept else None}
            r = res[sn][vn]
            print(f'{sn:14s} {vn:9s} 지도에 처음 잡힘 {r["first_frame"]} (나타난 뒤 {r["delay_ms"]} ms) · 물체 픽셀 남음 평균 '
                  f'{r["kept_frac_mean"]} 최소 {r["kept_frac_min"]}', flush=True)
    (out / 'delay.json').write_text(json.dumps({'base': str(d0), 'pixel': [v0, u0], 'appear_frame': t0, 'res': res}, ensure_ascii=False, indent=1))


def cmd_bench(d0):
    """서버 한 장: segmenter 팔 지우기 픽셀 (cuRobo, integrate 안에서도 한 번 더 함) + 시간 확인 + 가림 띠 — GPU 시간."""
    cap = load(d0)
    rig = MapRig(cap)
    rig.reset(cap)
    from curobo.types import CameraObservation
    cm = rig.cm
    hist = dfl.TemporalFilter(4, 5)
    res = {}
    fx = float(cap['K'][0, 0])

    def timeit(name, fn, n=300):
        ts = []
        for it in range(n):
            torch.cuda.synchronize()
            t = time.perf_counter()
            fn(it)
            torch.cuda.synchronize()
            ts.append((time.perf_counter() - t) * 1000)
        ts = np.array(ts[20:])
        res[name] = {'p50': round(float(np.percentile(ts, 50)), 3), 'p99': round(float(np.percentile(ts, 99)), 3)}
        print(f'{name:40s} p50 {res[name]["p50"]:.2f} ms · p99 {res[name]["p99"]:.2f} ms', flush=True)
    Ds = [torch.from_numpy(D.astype(np.float32)).to(DEV).mul_(0.001) for D in cap['D']]
    js = [rig.js(q) for q in cap['Q']]

    def seg(it):
        i = it % len(Ds)
        m, _ = cm.segmenter.get_robot_mask_from_active_js(CameraObservation(depth_image=Ds[i].view(1, 480, 640).clone(),
                                                                            pose=cm.camera_pose), js[i])
        return m
    timeit('팔 지우기 픽셀 (segmenter, 한 번 더)', seg)
    timeit('가림 띠 (B)', lambda it: dfl.shadow_band(Ds[it % 15], seg(it).view(480, 640), fx, 'left'))
    timeit('  B 만 (팔 픽셀 있을 때)', lambda it, m=seg(0).view(480, 640): dfl.shadow_band(Ds[it % 15], m, fx, 'left'))
    timeit('  B+ 적응 폭 만 (팔 픽셀 있을 때)', lambda it, m=seg(0).view(480, 640): dfl.shadow_band(Ds[it % 15], m, fx, 'left',
                                                                                      behind=True, adaptive=True))
    for k, n, tol in ((3, 4, None), (4, 5, None), (4, 5, 0.02)):
        tf = dfl.TemporalFilter(k, n, tol)
        timeit(f'시간 확인 A {k}/{n} tol {tol}', lambda it, tf=tf: tf.keep(it, Ds[it % 15]))

    def full(it):
        i = it % 15
        d = Ds[i]
        keep = hist.keep(it, d)
        m = seg(it).view(480, 640)
        return d * keep * ~dfl.shadow_band(d, m, fx, 'left', behind=True, adaptive=True)
    timeit("서버 한 장 전체 (A' 4/5 + B+ 적응 폭)", full)
    for rows in (15, 30, 45):
        timeit(f'  B+ 적응 폭 + 2D 기준 {rows} 줄 만', lambda it, m=seg(0).view(480, 640), rows=rows: dfl.shadow_band(
            Ds[it % 15], m, fx, 'left', behind=True, adaptive=True, ref_rows=rows))
    timeit('비교: 지도 쌓기 integrate (지금 서버)', lambda it: cm.integrate(Ds[it % 15], js[it % 15]), n=100)
    print(json.dumps(res, ensure_ascii=False))


if __name__ == '__main__':
    cmd = sys.argv[1]
    if cmd == 'side':
        cmd_side(sys.argv[2:])
    elif cmd == 'eval':
        if len(sys.argv) > 3 and sys.argv[3].startswith('--only='):     # 거르기 일부만 (예: --only=none,B+)
            only = sys.argv[3].split('=', 1)[1].split(',')
            for k in [k for k in VARIANTS if k not in only]:
                del VARIANTS[k]
            cmd_eval(sys.argv[2], sys.argv[4:])
        else:
            cmd_eval(sys.argv[2], sys.argv[3:])
    elif cmd == 'delay':
        cmd_delay(sys.argv[2], sys.argv[3])
    elif cmd == 'summary':                      # eval.json 에서 summary.png 만 다시
        summary_png(Path(sys.argv[2]), json.loads((Path(sys.argv[2]) / 'eval.json').read_text()))
    elif cmd == 'bench':
        cmd_bench(sys.argv[2])
    else:
        print(__doc__)
