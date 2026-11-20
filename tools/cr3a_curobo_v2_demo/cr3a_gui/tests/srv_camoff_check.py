"""[2026-11-02 디버깅 D-7] 고정 카메라 꺼짐 → 장애물 (카메라 지도) 없이 이동 — 서버 통합 시험 (GUI 대신 클라이언트, 실물 포트 47861 안 씀).
  python3 tests/srv_camoff_check.py <결과 폴더> [ideal|wmx] [포트 47878]
  ★ CR3A_TMP_DIR (카메라 PID 파일 — 실물 /tmp 아님)·ROS_DOMAIN_ID (시험 도메인) 를 따로 두고 부름 — 실물 카메라 노드·토픽을 안 봄 ★
서버 = --camera real --mask-q-file last_encoders.json --realtime off --speed 1.0 (기본 성형기 ETM4 8 Hz). 카메라 노드 없이 시작:
  ⓪ 준비 끝 (지도 배경을 안 기다림)  ① 실시간 회피 켜기 거절  ② 관절 목표 B: '주의' 한 번 + 도착, 3D 지도 칸 '안 씀'
  ③ 공구 끝 목표 (지금 −40 mm): '주의' + 도착  ④ [반복] 한 바퀴: '주의' 는 명령마다 한 번
  ⑤ 카메라 노드는 도는데 영상 없음 (가짜 노드 PID 파일) → 전처럼 거절, 안 움직임  ⑤' 노드를 끄면 다시 지도 없이 (B)
  ⑥ 가짜 깊이 (평면 PLANE_MM, 로봇을 가로지름) 를 냄 = 카메라 켜짐 → 서 있는 동안 지도 다시 씀, 목표 A: '주의' 없음·지도 칸 있음·평면에 막힘
  ⑦ 가짜 깊이 끔 = 카메라 꺼짐 → 같은 목표 A: '주의' + 도착 (멈춘 지도에 평면이 남아 있어도)  ⑧ 끄기 → 기록의 다리마다 map_off
"""
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path[:0] = [str(Path(__file__).resolve().parent), str(Path(__file__).resolve().parents[1])]
import camera_io as cio  # noqa: E402
import cr3a_model as cm  # noqa: E402
from srv_client import GUI, Gui, start_server  # noqa: E402

ROOT = Path(sys.argv[1])
ROBOT = sys.argv[2] if len(sys.argv) > 2 else 'ideal'
PORT = int(sys.argv[3]) if len(sys.argv) > 3 else 47878
PLANE_MM = 850              # 가짜 깊이: 받침 위 30 cm (0, 0, 0.3) 가 광축 깊이 ~0.85 m (등록한 카메라 위치) → 팔을 가로지르는 평면
ROOT.mkdir(parents=True, exist_ok=True)
assert ROBOT in ('ideal', 'wmx'), ROBOT
assert str(cio.CAM_PIDF.parent) != '/tmp', 'CR3A_TMP_DIR 을 시험 폴더로 (실물 카메라 PID 파일을 안 보게)'
assert os.environ.get('ROS_DOMAIN_ID', '') not in ('', '55', '57', '70'), 'ROS_DOMAIN_ID 를 시험 도메인으로'
assert cio.CameraProcess.pid() is None, '시험 폴더에 카메라 PID 파일이 있음'
RES = []
T0 = time.time()


def check(name, ok, detail=''):
    RES.append((name, bool(ok)))
    print(f'[{time.time() - T0:6.1f}s]', ('통과' if ok else '실패'), name, detail, flush=True)


class G2(Gui):
    """srv_client.Gui + 지도 칸 메시지 (시각, 칸 수, 'off')."""

    def __init__(self, port, timeout):
        self.vox = []
        super().__init__(port, timeout)

    def _rx(self):
        while self.alive:
            try:
                m = self.c.recv()
            except (EOFError, OSError):
                self.alive = False
                return
            t = m.get('type')
            if t == 'hello':
                self.hello.update(m)
                self.logs += m.get('logs', [])
            elif t == 'state':
                self.st = m
            elif t == 'voxels':
                self.vox.append((time.time(), len(m['P']), bool(m.get('off'))))
            elif t == 'log':
                self.logs.append(m['line'])
                print('   [서버]', m['line'][:230], flush=True)


def n(text):
    return len(g.seen(text))


def wait_new(text, n0, timeout):
    return g.wait(lambda: n(text) > n0, timeout)


def vox_after(t):
    """시각 t 뒤 지도 칸 메시지 (2 s 기다림) — 마지막 것 또는 None."""
    g.wait(lambda: any(v[0] > t for v in g.vox), 3.0)
    late = [v for v in g.vox if v[0] > t]
    return late[-1] if late else None


def deg(q):
    return np.degrees(np.asarray(q, float)).tolist()


def goto(q, label='목표 이동', timeout=90.0):
    """관절 목표 → (도착?, 그 명령의 '주의' 수)."""
    a0, w0 = n(f'{label} 도착'), n('주의 — 고정 카메라가 꺼져 있음')
    g.send({'cmd': 'goto', 'q_deg': deg(q)})
    ok = wait_new(f'{label} 도착', a0, timeout)
    time.sleep(0.5)
    return ok, n('주의 — 고정 카메라가 꺼져 있음') - w0


def wait_still(timeout=30.0):
    def still():
        u0 = g.u()
        time.sleep(0.3)
        return g.st.get('job') is None and np.max(np.abs(g.u() - u0)) < 1e-6
    return g.wait(still, timeout, step=0.0)


srv_out = ROOT / f'srv_{ROBOT}'
srv = start_server(PORT, srv_out, '--robot', ROBOT, '--camera', 'real', '--mask-q-file', str(GUI / 'last_encoders.json'),
                   '--realtime', 'off', '--speed', '1.0', log_path=ROOT / f'server_{ROBOT}.log')
pub = fake = None
try:
    g = G2(PORT, timeout=400.0)
    # ⓪ 준비
    check('⓪ 준비 끝 (카메라 꺼짐 — 지도 배경 안 기다림)', g.ready(400.0) and n('고정 카메라 꺼짐: 지도 없이') == 1)
    qA, qB = np.array(g.hello['qA']), np.array(g.hello['qB'])
    # ① 실시간 회피 켜기
    r0 = n('고정 카메라가 꺼져 있음 — 실시간 회피는 카메라를 켠 뒤에')
    g.send({'cmd': 'realtime', 'on': True})
    check('① 실시간 회피 켜기 거절 (카메라 꺼짐)', wait_new('고정 카메라가 꺼져 있음 — 실시간 회피는 카메라를 켠 뒤에', r0, 10.0)
          and g.wait(lambda: g.st.get('realtime') is False, 3.0))
    # ② 관절 목표 B
    t = time.time()
    ok, w = goto(qB)
    check('② 카메라 꺼짐 → 관절 목표 B 도착', ok)
    check("② '주의' 알림 한 번", w == 1, f'{w} 번')
    v = vox_after(t + 0.6)
    check("② 3D 지도 칸 = '안 씀' (칸 0)", v is not None and v[2] and v[1] == 0, str(v))
    check('② 도착 자리 = B', np.degrees(np.max(np.abs(np.array(g.st['q']) - qB))) < 0.2,
          f'{np.degrees(np.max(np.abs(np.array(g.st["q"]) - qB))):.3f}°')
    # ③ 공구 끝 목표
    wait_still()
    tip = cm.Robot().tip(np.degrees(g.st['q']))
    xyz = (tip[:3, 3] * 1000 + [0.0, 0.0, -40.0]).tolist()
    a0, w0 = n('공구 끝 이동 도착'), n('공구 끝 이동: 주의')
    g.send({'cmd': 'goto_pose', 'xyz_mm': xyz, 'rpy_deg': None})
    check('③ 카메라 꺼짐 → 공구 끝 목표 (−40 mm) 도착', wait_new('공구 끝 이동 도착', a0, 90.0))
    check("③ '공구 끝 이동: 주의' 한 번", n('공구 끝 이동: 주의') - w0 == 1)
    # ④ [반복] 한 바퀴
    wait_still()
    w0 = n('주의 — 고정 카메라가 꺼져 있음')
    a0 = n('반복 1: → 처음 자리 도착')
    g.send({'cmd': 'repeat', 'q_deg': deg(qB)})
    ok = wait_new('반복 1: → 처음 자리 도착', a0, 120.0)
    g.send({'cmd': 'stop'})
    check('④ [반복] 한 바퀴 (목표 → 처음 자리) 도착', ok)
    check("④ '주의' 는 명령마다 한 번", n('주의 — 고정 카메라가 꺼져 있음') - w0 == 1, f'{n("주의 — 고정 카메라가 꺼져 있음") - w0} 번')
    check('④ [정지] 뒤 섬', wait_still(40.0))
    # ⑤ 노드는 도는데 영상 없음 (고장)
    fake = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(900)', cio.CAM_NS])
    cio.CAM_PIDF.write_text(str(fake.pid))
    # (갓 띄운 자식은 exec 전 잠깐 부모 명령 줄 — /proc/<pid>/cmdline 에 /cam_fixed 가 나올 때까지)
    check('⑤ 가짜 카메라 노드 PID 가 보임', g.wait(lambda: cio.CameraProcess.pid() == fake.pid, 5.0))
    u0 = g.u()
    r0 = n('지도를 새로 못 봐 안 움직임')
    g.send({'cmd': 'goto', 'q_deg': deg(qA)})
    check('⑤ 노드는 도는데 영상 없음 → 거절 (전과 같음)', wait_new('지도를 새로 못 봐 안 움직임', r0, 15.0))
    time.sleep(1.0)
    # (서 있을 때 상태 u = 명령 그대로, 주기 기록 u = 소수 6 자리 반올림 → 1e-5 rad 안이면 같은 자리)
    du = np.max(np.abs(g.u() - u0))
    check('⑤ 안 움직임', du < 1e-5, f'{np.degrees(du):.6f}°')
    fake.terminate()
    fake.wait(5)
    cio.CAM_PIDF.unlink(missing_ok=True)
    fake = None
    # ⑤' 다시 카메라 꺼짐 (노드 없음) → B: ⑤ 에서 '노드 켜짐' 으로 지도를 다시 쓰게 됐으므로, ⑥ 의 '영상이 다시 옴' 을 보려고 지도 끔으로
    time.sleep(0.5)
    ok, w = goto(qB)
    check("⑤' 노드 끈 뒤 → 다시 카메라 꺼짐 이동 (B 도착, '주의' 한 번)", ok and w == 1, f'{w} 번')
    wait_still()
    # ⑥ 카메라 켜짐 (가짜 깊이 — 평면)
    f0 = n('고정 카메라 영상이 다시 옴')
    pub = subprocess.Popen([sys.executable, str(GUI / 'tests' / 'fake_depth_pub.py'), str(PLANE_MM)])
    check('⑥ 영상이 오면 서 있는 동안 지도를 다시 씀', wait_new('고정 카메라 영상이 다시 옴', f0, 20.0))
    v = vox_after(time.time())
    check("⑥ 3D 지도 칸 '안 씀' 풀림", v is not None and not v[2], str(v))
    c0 = n('지도 점검 통과') + n('지도 점검 걸림')
    g.send({'cmd': 'map_check'})                    # 지도를 1.4 s 쌓음 (실시간 회피 끔에서는 서 있는 동안 지도가 멈춤)
    check('⑥ [지도 점검] 이 지도를 쌓음 (평면이 로봇 둘레면 걸림)', g.wait(lambda: n('지도 점검 통과') + n('지도 점검 걸림') > c0, 20.0))
    w0 = n('주의 — 고정 카메라가 꺼져 있음')
    e0, a0 = n('목표 이동 시간 초과 (계획 실패)'), n('목표 이동 도착')
    t = time.time()
    g.send({'cmd': 'goto', 'q_deg': deg(qA)})       # 막히면 다리 시간 초과 (25 s) 까지 다시 계획 — 기록에 그 다리가 남게 그대로 둠
    g.wait(lambda: n('목표 이동 시간 초과 (계획 실패)') > e0 or n('목표 이동 도착') > a0, 90.0)
    blocked = n('목표 이동 시간 초과 (계획 실패)') > e0
    v = vox_after(t + 0.6)
    check("⑥ 카메라 켜짐 → '주의' 없음 (지도를 씀)", n('주의 — 고정 카메라가 꺼져 있음') == w0)
    check('⑥ 지도에 평면 칸이 쌓임', v is not None and not v[2] and v[1] > 100, str(v))
    check('⑥ 평면에 막혀 계획 실패 (지도 켬)', blocked)
    check('⑥ 섬', wait_still(40.0))
    # ⑦ 카메라 꺼짐 (가짜 깊이 끔) — 멈춘 지도에는 평면이 남음
    pub.terminate()
    pub.wait(5)
    pub = None
    time.sleep(1.5)
    ok, w = goto(qA)
    check('⑦ 카메라 꺼짐 → 같은 목표 A 도착 (평면 안 봄)', ok)
    check("⑦ '주의' 한 번", w == 1, f'{w} 번')
    check('⑦ 도착 자리 = A', np.degrees(np.max(np.abs(np.array(g.st['q']) - qA))) < 0.2)
    # ⑧ 끄기
    wait_still()
    g.send({'cmd': 'quit'})
    try:
        code = srv.wait(240)
    except subprocess.TimeoutExpired:
        code = None
    check('⑧ 서버 끝 코드 0', code == 0, str(code))
finally:
    for p in (pub, fake):
        if p is not None:
            p.terminate()
    cio.CAM_PIDF.unlink(missing_ok=True)
    if srv.poll() is None:
        srv.terminate()
        try:
            srv.wait(120)
        except subprocess.TimeoutExpired:
            srv.kill()

try:
    legs = json.loads((srv_out / 'rt_run.json').read_text())['legs']
except (OSError, ValueError, KeyError) as e:
    legs = []
    print('기록 못 읽음', e)
for L in legs:
    print('  다리', L['label'], '도착' if L.get('ok') else f'못 감 ({L.get("why")})', 'map_off', L.get('map_off'),
          '계획', L.get('plans'), '경로', [p['kind'] for p in L.get('paths', [])])
moved = [L for L in legs if L.get('ok')]
check('⑧ 기록: 도착한 다리 = 모두 카메라 꺼짐 (map_off) 6 개', len(moved) == 6 and all(L.get('map_off') for L in moved),
      f'{sum(1 for L in moved if L.get("map_off"))}/{len(moved)}')
check('⑧ 기록: 카메라 켜짐 다리 (⑥) 는 map_off 아님', any(L.get('map_off') is False and not L.get('ok') for L in legs) or not blocked)
check('⑧ 기록: 카메라 꺼짐 다리도 cuRoboV2 계획 경로 (rest)', all(L.get('paths') and L['paths'][0]['kind'] == 'rest'
                                                           for L in moved if L.get('map_off')))
n_bad = sum(1 for _, ok in RES if not ok)
print(f'RESULT {ROBOT} 통과 {len(RES) - n_bad}/{len(RES)}', flush=True)
sys.exit(1 if n_bad else 0)
