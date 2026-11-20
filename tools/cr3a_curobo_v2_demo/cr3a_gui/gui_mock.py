#!/usr/bin/env python3
"""CR3A GUI 화면 시안 (--mock): 서버 없이 예시 숫자로 채운 창 + 탭마다 PNG 저장 (--shot).

[2026-10-27 R-1 리팩터링] cr3a_gui.MainWindow 안에 있던 시안 코드 (if self.mock 갈래 11 곳, 예시 경로·지도·깊이·그래프, rt_shaping) 를
  여기로 — MockWindow 가 MainWindow 에서 필요한 것만 바꿈 (서버 연결·카메라·보정 창·명령은 '[UI 시안] 기능 없음' 로그만).
실행: python cr3a_gui.py --mock [--shot <폴더>]
"""
import time

import cv2
import numpy as np
from PyQt5 import QtGui, QtWidgets

import cr3a_gui as G
import cr3a_model as cm
from common import to_qimage
from shaper_def import build, ms

PATH_BUMP = np.array([0.0, 20.0, 15.0, 0.0, 0.0, 0.0])   # (시안) 예시 경로: A → B 를 기둥 위로


class MockWindow(G.MainWindow):
    """화면 시안: 서버에 안 붙고 예시 숫자로 채움. 명령은 보내지 않고 '[UI 시안] … 기능 없음' 로그만."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle(self.windowTitle() + '   [UI 시안: 기능 없음, 숫자는 예시]')

    def _start(self):
        self._mock()

    def _cmd(self, m, quiet=False) -> bool:
        self._log(30, f'[UI 시안] "{m.get("cmd")}" — 기능 없음')
        return False

    def on_toggle_stack(self):
        return self._cmd({'cmd': 'stack'})

    def on_cam_on(self):
        return self._cmd({'cmd': 'camera'})

    def on_cam_off(self):
        return self._cmd({'cmd': 'camera'})

    def on_calib(self):
        return self._cmd({'cmd': 'calib'})

    def on_realtime(self, on):
        return

    def on_shaper_apply(self, cfg):
        self.shaping.mark_applied(cfg)
        self._save_shaper(cfg)

    def _fill_pose(self):
        super()._fill_pose()
        self.scene.ghost = [sp.value() for sp in self.sp_target]
        self.scene.target_T = self.robot.tip(self.scene.ghost)
        self.request_render()

    def _mock(self):
        C = G.COLORS
        A = np.array((56.31, -40.67, -12.99, -29.16, 83.72, 55.72))       # (시안 예시 경로의 두 끝 — 자세 목록과 무관)
        B = np.array((-28.78, -42.61, -10.82, -27.41, 96.28, -26.31))
        s = np.linspace(0, 1, 80)[:, None]
        path = A + (B - A) * s + np.sin(np.pi * s) * PATH_BUMP
        q_now, q_u = path[30], path[36]
        self._set_pill(self.chips['robot'], '서보 켜짐', C['ok'], '6 축 서보 켜짐, 알람 없음')
        self._set_pill(self.chips['wmx'], '실제 로봇 (EtherCAT)', C['real'], 'WMX 엔진이 EtherCAT 으로 CR3A 에 붙어 있음')
        self._set_pill(self.chips['sys'], '켜짐', C['ok'], 'WMX 엔진 · R2 · 계획·회피 노드')
        self._set_pill(self.chips['cam'], '15 fps', C['ok'])
        self._set_pill(self.chips['obst'], '켜짐', C['ok'], '지도 물체 칸 1,284 개')
        self.btn_stack.setText('■ 시스템 종료')
        self.cb_target.setEnabled(False)
        self.shaping.set_config('ETM4', [(True, 8.0), (True, 15.0), (True, 23.0)])
        res = build(*self.shaping.config())
        sc = self.scene
        sc.q, sc.u_q, sc.ghost, sc.path = q_now, q_u, B, path
        sc.target_T = self.robot.tip(B)
        sc.cam_T, Tbc = G.default_cam_T()
        sc.obst = (mock_voxels(Tbc[:3, 3]), 0.01)
        self._update_pose(q_now)
        for sp, val in zip(self.sp_target, B):
            sp.setValue(val)
        tB = self.robot.tip(B)
        for k in range(3):
            self.sp_xyz[k].setValue(tB[k, 3] * 1000)
        for sp, a in zip(self.sp_rpy, cm.rpy_from_matrix(self.robot.tip(q_now)[:3, :3])):
            sp.setValue(a)
        self.chk_rt.setChecked(True)
        self.lb_preview.setPixmap(mock_depth_pixmap(212, 120))
        self.lb_caminfo.setText('<b>RealSense D455</b> (고정)<br>깊이 640 × 480 · 15 fps<br>'
                                f'<span style="color:{C["ok"]}">● 영상 들어옴</span> (마지막 0.03 s 전)<br>'
                                f'<span style="color:{C["off"]}">깊이 영상 (예시)</span>')
        self._show_cam_pose(Tbc)
        self._set_pill(self.st_chips['cam'], '실제 카메라 15 fps', C['ok'])
        self._set_pill(self.st_chips['map'], '지도 쌓는 중', C['ok'])
        self.lb_map.setText(f'지도: 물체 칸 <b>{len(sc.obst[0]):,}</b> 개 · 1 cm 칸 · 영상마다 갱신 (15 Hz)')
        self.lb_reason.setText(f'<b>A → B</b> <span style="color:{C["ok"]}">계획 성공</span> (0.21 s) — 실행 중 '
                               '<b>1.1 s</b> / 3.2 s')
        self.lb_plan.setText('경로 1.65 s · 공구 끝 0.92 m · 장애물까지 최소 48 mm · 관절 이동 214° · '
                             f'입력성형 지연 {ms(res["t"][-1])} ms 포함')
        self.lb_exec.setText('마지막 실행: B → A 도착 3.24 s (0.1° 안) · 14:03:31')
        self.scope.btn_run.setChecked(True)                # (켜기가 그림을 비우므로 그림보다 먼저)
        self.scope.set_data(mock_scope(path, res))
        for st, lv, text in (
                ('14:02:10', 20, '시스템 시작: 대상 = 실제 로봇 (EtherCAT) — WMX 엔진 · R2 · 계획·회피 노드'),
                ('14:02:18', 20, 'WMX 통신 시작 (1 ms) · 축 6 개 정상'),
                ('14:02:25', 20, '서보 켜짐 (J1 ~ J6)'),
                ('14:02:31', 20, '카메라 켜짐: RealSense D455 640 × 480 · 15 fps'),
                ('14:02:52', 20, f'장애물 회피 켜짐: 지도 물체 칸 {len(sc.obst[0]):,} 개'),
                ('14:03:05', 20, f'입력성형 적용: {self.shaping.label()} — 임펄스 {len(res["t"])} 개, '
                                 f'지연 시간 {ms(res["t"][-1])} ms'),
                ('14:03:20', 20, '실시간 회피 켜짐 (20 Hz)'),
                ('14:03:28', 20, '목표 이동 B → A: 계획 0.19 s, 경로 1.62 s → 도착 3.24 s'),
                ('14:03:40', 20, '목표 이동 A → B: 계획 0.21 s, 경로 1.65 s')):
            self._log(lv, text, st)
        self.request_render()


def mock_voxels(cam_pos, vs=0.01):
    """(시안) 예시 지도: 기둥 (0.42, 0, 0.15) 10 x 10 x 30 cm + 상자 8 cm — 카메라에서 보이는 면만 1 cm 칸."""
    out = []
    for c, size in (((0.42, 0.0, 0.15), (0.10, 0.10, 0.30)), ((0.22, -0.26, 0.46), (0.08, 0.08, 0.08))):
        c, size = np.array(c), np.array(size)
        lo, hi = c - size / 2, c + size / 2
        axes = [np.arange(lo[i] + vs / 2, hi[i], vs) for i in range(3)]
        X, Y, Z = np.meshgrid(*axes, indexing='ij')
        P = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
        keep = np.zeros(len(P), bool)
        for i in range(3):
            for side, bound in ((-1, lo[i]), (1, hi[i])):
                if side * (cam_pos[i] - bound) > 0:                  # 카메라 쪽 면
                    keep |= np.abs(P[:, i] - (bound - side * vs / 2)) < vs * 0.6
        out.append(P[keep])
    return np.concatenate(out)


def mock_depth_pixmap(w, h):
    """(시안) 예시 깊이 영상 (가짜): 바닥 기울기 + 기둥 + 상자 + 팔 → 색."""
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    d = 1.6 - 0.8 * yy / h
    d[(xx > 0.46 * w) & (xx < 0.58 * w) & (yy > 0.32 * h)] = 0.78
    cv2.circle(d, (int(0.25 * w), int(0.35 * h)), 13, 0.62, -1)
    cv2.line(d, (int(0.80 * w), int(0.95 * h)), (int(0.70 * w), int(0.40 * h)), 0.70, 14)
    cv2.line(d, (int(0.70 * w), int(0.40 * h)), (int(0.52 * w), int(0.22 * h)), 0.66, 11)
    img = cv2.applyColorMap(np.clip((d - 0.5) / 1.2 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)    # BGR
    return QtGui.QPixmap.fromImage(to_qimage(img))


def mock_scope(path, res):
    """(시안) 예시 관절 그래프: B → A 이동, 멈춤, A → B 이동 중 (지금). 1 ms 칸. 성형 후 = 성형기로 컨볼루션, 실제 = 6 ms 늦게."""
    import rt_shaping  # (shaper_def 가 cr3a_rt_shaping 경로를 넣음 — 시안에서만 써서 여기서 가져옴)
    t = np.arange(0, 10.0, 0.001)
    rev = path[::-1]

    def along(p, t0, dur):
        x = np.clip((t - t0) / dur, 0, 1)
        sv = 10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5
        idx = sv * (len(p) - 1)
        i = np.minimum(idx.astype(int), len(p) - 2)
        f = (idx - i)[:, None]
        return p[i] * (1 - f) + p[i + 1] * f
    u = along(rev, 1.0, 1.65)
    u2 = along(path, 8.9, 1.65)
    u[t >= 8.9] = u2[t >= 8.9]
    h = rt_shaping.to_grid(res['A'], res['t'], 0.001)
    y = rt_shaping.shape_fir(u, h)
    fb = np.vstack([np.repeat(y[:1], 6, 0), y[:-6]])
    return {'t': t, 'u': u, 'y': y, 'fb': fb}


def shots(win, out):
    """시안 화면을 탭마다 PNG 로 (out 폴더) + 입력성형 패널 몇 가지 + 3D 옆 보기."""
    out.mkdir(parents=True, exist_ok=True)
    app = QtWidgets.QApplication.instance()
    for name, lt, rt in (('shot1_jog.png', 0, 0), ('shot2_target.png', 0, 1), ('shot3_camera.png', 0, 2),
                         ('shot4_graph.png', 1, 1)):
        win.view_tabs.setCurrentIndex(lt)
        win.tabs.setCurrentIndex(rt)
        app.processEvents()
        t0 = time.perf_counter()
        win.scene._sig = None                        # (지금 scene3d: 같은 그림이면 안 그리는 표식 — 다시 그리게)
        win.scene.refresh()
        print(f'{name}: 3D 그리기 {(time.perf_counter() - t0) * 1000:.0f} ms')
        app.processEvents()
        win.grab().save(str(out / name))
    win.view_tabs.setCurrentIndex(0)
    for name, method, rows in (('shaping_ZV_8Hz.png', 'ZV', [(True, 8.0), (False, 15.0), (False, 23.0)]),
                               ('shaping_ETM5_3freq.png', 'ETM5', [(True, 8.0), (True, 15.0), (True, 23.0)]),
                               ('shaping_ETM4_5_6.4Hz.png', 'ETM4', [(True, 5.0), (True, 6.4), (True, 23.0)]),
                               ('shaping_ZV_150Hz_warn.png', 'ZV', [(True, 150.0), (False, 15.0), (False, 23.0)])):
        win.shaping.set_config(method, rows, apply=False)
        app.processEvents()
        win.shaping.grab().save(str(out / name))
    win.scene.flags['u'] = True
    win.scene.set_preset('옆')
    app.processEvents()
    win.scene.grab().save(str(out / 'scene_side_u.png'))
    print('저장:', out)
