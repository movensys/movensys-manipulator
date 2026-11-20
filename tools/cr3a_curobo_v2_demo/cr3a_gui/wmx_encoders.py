#!/usr/bin/env python3
"""실제 로봇 엔코더 위치 읽기 — R2 축 상태 (/wmx/axes/status, wmx_core_motion_node) 1 s 를 모아 저장.  ★ wmx_r2 컨테이너 안 ★
  python3 wmx_encoders.py <저장 JSON>        (wmx_r2.sh encoders 가 부름)
실제 위치 (actual_pos) 는 CR3A 파라미터 XML 의 기어비로 rad (R2 ros2_control 의 /joint_states 와 같은 값). 서보가 켜진 축이 하나라도
보이면 바로 실패 (코드 3) — 이 읽기는 서보를 켜지 않는 것이 전제.
"""
import json
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from wmx_r2_message.msg import AxesStatus

AXES = 6
WAIT_S = 60.0               # 엔진이 통신을 시작하고 축 상태가 나올 때까지 기다리는 최대 시간
COLLECT_S = 1.0


def main():
    out = sys.argv[1]
    rclpy.init()
    node = Node('cr3a_read_encoders')
    msgs = []
    node.create_subscription(AxesStatus, '/wmx/axes/status', msgs.append, 10)
    t0, t_first = time.time(), None
    try:
        while time.time() - t0 < WAIT_S:
            rclpy.spin_once(node, timeout_sec=0.05)
            ok = [m for m in msgs if len(m.actual_pos) >= AXES]
            if ok and any(any(m.servo_on[:AXES]) for m in ok):
                print('✗ 서보가 켜진 축이 있음 → 읽기 멈춤', [list(m.servo_on[:AXES]) for m in ok[-1:]], flush=True)
                sys.exit(3)
            if ok and t_first is None:
                t_first = time.time()
            if t_first is not None and time.time() - t_first >= COLLECT_S:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    ok = [m for m in msgs if len(m.actual_pos) >= AXES]
    if not ok:
        print(f'✗ {WAIT_S:.0f} s 안에 축 상태를 못 받음', flush=True)
        sys.exit(1)
    P = np.array([m.actual_pos[:AXES] for m in ok], float)
    C = np.array([m.pos_cmd[:AXES] for m in ok], float)
    q = np.median(P, axis=0)
    res = {'t': time.time(), 'n': len(ok), 'q_rad': q.tolist(), 'q_deg': np.degrees(q).round(4).tolist(),
           'spread_deg': np.degrees(P.max(0) - P.min(0)).round(5).tolist(), 'pos_cmd_rad': np.median(C, axis=0).tolist(),
           'servo_on': [bool(v) for v in ok[-1].servo_on[:AXES]], 'amp_alarm': [bool(v) for v in ok[-1].amp_alarm[:AXES]],
           'n_axes_reported': len(ok[-1].actual_pos)}
    with open(out, 'w') as f:
        json.dump(res, f, indent=1)
    print('엔코더 (°):', ' '.join(f'{v:9.3f}' for v in res['q_deg']), flush=True)
    print('1 s 동안 흔들림 (°):', ' '.join(f'{v:.4f}' for v in res['spread_deg']), flush=True)
    print('서보:', res['servo_on'], ' 알람:', res['amp_alarm'], f' 메시지 {res["n"]} 개 · 축 {res["n_axes_reported"]} 개', flush=True)
    if not all(math.isfinite(v) for v in q):
        sys.exit(1)


if __name__ == '__main__':
    main()
