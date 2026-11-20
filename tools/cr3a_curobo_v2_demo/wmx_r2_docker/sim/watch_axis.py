#!/usr/bin/env python3
# [만듦 2026-10-13] /wmx/axis/state 를 받아 한 축의 위치를 0.25초마다 찍는다 (읽기만). sim/axis_demo.sh 가 컨테이너 안에서 실행.
#   python3 watch_axis.py <축 번호> <몇 초>
#   찍는 값: 서보 ON 여부, 명령 위치(pos_cmd), 실제 위치(actual_pos), 실제 속도, 동작 완료(motion_complete)
#   단위는 WMX 사용자 단위 (시뮬레이터 기본 설정이면 펄스)
import sys
import time

import rclpy
from rclpy.node import Node
from wmx_r2_message.msg import AxisState

AXIS = int(sys.argv[1]) if len(sys.argv) > 1 else 0
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0


class Watch(Node):
    def __init__(self):
        super().__init__('watch_axis')
        self.last = None
        self.first = None
        self.create_subscription(AxisState, '/wmx/axis/state', self.on_state, 10)

    def on_state(self, m):
        if AXIS < len(m.actual_pos):
            self.last = m
            if self.first is None:
                self.first = m


def main():
    rclpy.init()
    n = Watch()
    t0 = time.time()
    next_print = t0
    print(f'  {"시간":>5} {"서보":>4} {"명령위치":>10} {"실제위치":>10} {"실제속도":>9} {"완료":>4}')
    while time.time() - t0 < DURATION:
        rclpy.spin_once(n, timeout_sec=0.05)
        if n.last is not None and time.time() >= next_print:
            m = n.last
            print(f'  {time.time() - t0:5.2f} {"ON" if m.servo_on[AXIS] else "off":>4} {m.pos_cmd[AXIS]:10.1f} '
                  f'{m.actual_pos[AXIS]:10.1f} {m.actual_velocity[AXIS]:9.1f} {"예" if m.motion_complete[AXIS] else "-":>4}')
            next_print += 0.25
    if n.first is None:
        print(f'  /wmx/axis/state 를 못 받음 (축 {AXIS})')
        sys.exit(1)
    d = n.last.actual_pos[AXIS] - n.first.actual_pos[AXIS]
    print(f'  축 {AXIS}: 실제 위치 {n.first.actual_pos[AXIS]:.1f} → {n.last.actual_pos[AXIS]:.1f} (변화 {d:+.1f})')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
