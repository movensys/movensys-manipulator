#!/usr/bin/env python3
"""CR3A GUI 서버 — 실시간 회피 (cr3a_rt_shaping 의 rt_run.Controller) 를 정해진 시나리오 대신 GUI 명령으로 돌린다.

GUI (cr3a_gui.py) 와는 같은 PC 안 연결 (multiprocessing.connection, 127.0.0.1:PORT): GUI → 명령 dict, 서버 → 상태 (주기마다)·로그·
경로·지도 칸. GUI 를 닫아도 서버는 계속 돌고, 다시 열면 이어 붙음 (미로봇 GUI 와 같음). [■ 시스템 종료] = quit.
[2026-10-27 사용자 결정 ①] GUI 연결이 끊기면 (창 닫기·GUI 가 GUI_LOST_S 동안 ping 없음) 로봇을 세우고 실시간 회피 끔·대기 일 취소
(서보는 그대로, 다시 열면 이어서 조작).

로봇 (--robot)
  ideal  [2026-10-26 1 차] 소프트웨어 로봇 (보낸 명령 그대로, rt_robot.IdealRobot). WMX·ROS·실제 로봇 없음.
  wmx    [2026-10-26 2 차] R2 → WMX 시뮬레이터. 켤 때 wmx_r2.sh up (안전장치: Module.ini·엔진이 시뮬레이터뿐인지 → R2 → 축 로그),
         R2 의 /joint_states (WMX 실제 위치 = 실물에서는 엔코더) 를 읽고 JTC 로 궤적 (rt_robot.RosRobot — tools/rt_demo.sh 와 같은 길),
         끌 때 서보 OFF → wmx_r2.sh down. R2 로그의 오류·/joint_states 끊김 → 비상정지. 기록 (--out, 기본
         ~/workspaces/wmx_r2_logs/gui_wmx_<시각>) = rt_run.json·u_exec*.npz + WMX 축 로그 (python3 -m tools.wmx_check 가 검증).
  real   [2026-10-26 사용자 "자 그럼 실물 로봇으로 움직여보자"] R2 → WMX (EtherCAT) → 실제 CR3A. 켤 때 wmx_r2.sh up-real (Module.ini 를
         EtherCAT 으로, 끌 때 다시 시뮬레이터로), 나머지는 wmx 와 같음. 실시간 회피는 꺼진 채로 시작 (사용자 "꺼진 채로 시작"). 기록 gui_real_<시각>.
  카메라 (--camera): virtual = 가상 (rt_world: 기둥 + 시험 물체 moving3t), real = 실제 고정 카메라 (camera_io, 등록한 위치).
속도 배율 (--speed, 명령 speed {scale}) [2026-11-02 디버깅 D-5, 사용자 11-01 "속도 세팅을 하면 urdf의 속도 및 가속도 값을 낮춘것과
  같은 효과"]: 배율 s = 로봇 명령의 속도·가속 한계 × s (배율 1 한계 = 로봇 한계의 LIMIT_FRAC, 저크 한계 × JERK_MULT — 아래. 저크
  한계는 배율과 상관없음). 목표 이동·반복: cuRobo 가 배율 1 한계로 낸 경로를 한계 × s 안에 드는 만큼만 고르게 늘림 (같은 길 —
  speed_stretch, cuRobo 가 계획 시간을 정하는 식과 같은 꼴 → 낮은 배율에서는 최고 속도가 한계 × s 에 닿음, 배율 1 은 그대로. 늘린 점
  사이가 PATH_GAP_MAX_S 를 넘는 아주 긴 이동만 같은 곡선 위에 점을 다시 찍음 — scale_path).
  실시간 회피 (MPC 당근·비키기) 는 추종기 한계 × s (gui_controller._track_lim) → 목표 이동과 같은 한계. 한 칸 조그 = 최고 속도 × s
  (아래 조그). 입력성형은 그 뒤 (실제 시간 1 ms 칸) 라 성형기는 안 늘어남 → 성형된 명령 시간 = 늘린 시간 + 성형 지연. 실시간 회피
  끔의 지키기·누르는 동안 조그·안전 검사 감속·비상정지는 한계 그대로. 바꾸면 다음 일 (이동) 부터.
  [옛 뜻 2026-10-26 ~ 11-01 "1초 모션이 2초가 되고"] 시각을 1/s 로 늘림 (속도 × s · 가속 × s²) — 배율 1 경로가 속도 한계의 ~53 % 만
  써서 같은 배율에서 목표 이동은 한계 × s 의 ~53 %, 회피는 100 % → 회피가 더 빨라 보였음 (D-5).
명령
  goto {q_deg}         관절 목표로 (rt_run.Controller.leg: 멈춰서 계획 → MPC 가 경로를 따라감 → 0.1° 안 0.3 s 면 도착)
                       [2026-11-02 D-7 사용자 "카메라가 끄기 상태라면 장애물 고려하지 않는 모션을 생성하도록"] 실제 카메라 지도 + 실시간 회피
                       끔에서 고정 카메라가 꺼져 있으면 (노드 없음 + 영상 없음) 카메라 지도 없이 계획·이동 (책상·자기 충돌·관절 한계만 —
                       gui_controller.GuiWorld). 노드가 도는데 영상이 안 오면 (고장) 전처럼 거절. goto_pose·repeat 도 같음
  repeat {q_deg}       지금 자리 ↔ 목표 왕복 (멈출 때까지)
  stop                 성형 거쳐 감속 정지 (지금 상태에서 저크 한계로 서는 자리를 지킴)
  estop                즉시 멈춤: 성형을 건너뛰고 로봇이 따라가는 명령에서 가속·저크 한계로 바로 감속, 성형 기억 = 선 자세, 실시간 회피도 끔
  realtime {on}        끔 = 지도를 그 순간에 멈춤 (새로 다가오는 물체를 안 봄, 계획 직전에만 새로 봄) + 서 있을 때 명령을 안 보냄 (안 피함)
  shaper {method, rows}    입력성형기 바꾸기 (shaper_def — GUI 패널과 같은 정의). 명령이 서 있을 때만
  objects {go}         (시험) 가상 물체 움직이기 시작 / 멈춤 · map_reset 지도 새로 쌓기 · quit
  speed {scale}        속도 배율 (0.01~1, 서 있을 때만 — 위)
  ping                 GUI 살아 있음 (0.5 s 마다, 명령 줄에는 안 넣음 — 끊김 감시만)
  [2026-10-27 사용자 "카테시안 조그나 카테시안 목표 이동 … 구현해줘. 특히 카테시안 조그도 조인트 조그처럼 누르는 동안 이동", "역기구학은
   … curobo 나 무브잇 라이브러리로"] 공구 끝 (그리퍼 손가락 끝 = Link6 + TIP_M) 목표 — 역기구학 = cuRobo IKSolver (계획기 안, 관절 한계·
   자기 충돌·지도 충돌 함께), 정기구학 = cuRobo 운동학 (같은 모델):
  goto_pose {xyz_mm, rpy_deg|None}   공구 끝 목표 (받침 기준 mm, Rx Ry Rz ° = 고정 축 x→y→z — GUI 와 같음, None = 지금 방향 그대로)
                       → IK (지금 관절에 가장 가까운 해) → goto 와 같음 (계획 → 속도 배율 → 입력성형)
  repeat_pose {…}      지금 자리 ↔ 그 공구 끝 목표 왕복
  조그 [2026-10-27 사용자 "조그에서는 입력성형 빼고 무브잇만으로 조그하도록 하자. 목표 이동만 입력성형 적용"] = MoveIt Servo (servo/
   cr3a_servo.launch.py, R2 와 함께 켬 — 이상적 모드에는 없음). 서버는 조그 동안 R2 에 명령을 안 보내고 GUI 명령을 Servo 로 넘김 →
   Servo 가 R2 팔 제어기로 바로 (입력성형 없음, 카메라 지도 안 봄 — MoveIt 장면 = 로봇·테이블·Thor 상자). Servo 가 멈추면 서버 명령
   시간표·성형 기억을 그 자리에서 새로 (_servo_exit):
  jog {joint, delta_deg}   관절 한 칸 — 그 관절만, 속도 STEP_V × 속도 배율, 남은 거리로 줄여 도착 (Servo 가 낸 최신 명령으로 잼)
  jog_cart {axis, delta, frame}      공구 끝 한 칸 (axis X Y Z mm / Rx Ry Rz °, frame base = 받침 축 · tool = 공구 축) — 곧은 선, 돌기는
                       공구 끝 점 둘레
  jog_hold {joint, v_deg} · jog_cart_hold {axis, v, frame}   누르는 동안 (°/s · mm/s, GUI 가 50 ms 마다, JOG_TIMEOUT_S 안 오면 뗀 것)
  jog_release          뗌 → Servo 에 0 속도 SERVO_HALT_S 동안 (부드럽게 섬)
실행: rt_server.py --robot ideal|wmx|real [--port 47861] [--shaper JSON] [--speed 0.2] [--q0 A|B|zero] [--out 폴더] [--camera virtual|real]
      (보통 GUI 의 [▶ 시스템 시작] 이 띄움). 시험용: --test-comm-left-s S (WMX 통신 남은 시간), --test-rec-max-s S (기록 줄이기 시작)
      SIGTERM·SIGINT [2026-10-27 R-1 ⑥]: 켜는 중에는 바로 끝 (finally 정리), 명령을 받는 중이면 첫 번째 = [■ 시스템 종료] 와 같은 끄기
      (성형 거쳐 정지 → 기록 저장 → R2 끄기), 두 번째 = 바로 끝.
"""
import argparse
import atexit
import collections
import dataclasses
import json
import logging
import logging.handlers
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from multiprocessing.connection import Listener
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RT = HERE.parent / 'cr3a_rt_shaping'
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(RT))
import cr3a_model as cm  # noqa: E402  (공구 끝 = Link6 + TIP_M, 각도 표기 — GUI 와 같은 정의)
# [2026-10-27 R-1 S2] 포트·인증·경로는 GUI 와 한곳 (common.py — PyQt 없음): PORT (CR3A_GUI_PORT, 기본 47861) · AUTH · LOG_ROOT
#  (CR3A_LOG_ROOT) · TMP_DIR (CR3A_TMP_DIR)
from common import AUTH, LOG_ROOT, PORT, TMP_DIR  # noqa: E402
JOG_TIMEOUT_S = 0.4         # 누르는 동안 조그: 이 시간 안에 다음 명령이 안 오면 뗀 것 (GUI 는 50 ms 마다, 스레드에서).
                            #  [2026-10-26 Thor 시험] 0.25 s 에서 GUI 가 3D 를 그리느라 늦어 한 번 '뗌' 으로 끊겼음
# [2026-10-27] 조그 = MoveIt Servo (servo/cr3a_servo.launch.py · cr3a_servo.yaml)
SERVO_LAUNCH = HERE / 'servo' / 'cr3a_servo.launch.py'
SERVO_NODE = '/servo_node'
SERVO_BASE = 'Link0'        # Servo 로봇 형상의 받침 (= IK 사슬 시작 = Servo 기준 좌표). 우리 base_link 와 방향 같음 (J1 높이만 6.5 mm 다름)
SERVO_TIP = 'gui_tip'       # Servo 로봇 형상의 공구 끝 (Link6 + TIP_M — launch 파일이 붙임)
SERVO_HALT_S = 0.3          # 조그를 멈출 때 0 속도를 보내는 시간 (Servo 의 부드럽게 하기 필터가 줄임), 그 뒤 명령을 끊음
SERVO_QUIET_S = 0.3         # 멈춘 뒤 Servo 가 이만큼 아무 궤적도 안 내고 관절이 서 있으면 조그 끝 → 서버 명령 시간표를 그 자리에서 새로
SERVO_EXIT_MAX_S = 4.0      # 멈추기 시작해서 이만큼 지나도 안 서면 경고하고 넘어감
STEP_V = (math.radians(10.0), 0.05, math.radians(10.0))    # 한 칸 조그 최고 속도 (관절 rad/s · 공구 끝 m/s · 회전 rad/s) × 속도 배율
STEP_K = 5.0                # 한 칸: 남은 거리 × 이것 = 속도 (1/s) — 끝에서 부드럽게 줄며 도착
STEP_ACC = 4.0              # 한 칸: 명령 속도를 1 s 에 최고 속도의 이 배까지만 바꿈 (0.25 s 에 최고 속도)
STEP_VMIN = (0.006, 0.003, math.radians(0.5))     # 한 칸 끝 속도 최소 (관절 rad/s · m/s · rad/s). Servo 의 가속도 제한 필터는 한 주기
                            #  (25 ms) 관절 변화·속도 변화가 둘 다 1e-4 rad 아래면 명령을 무시함 (COMMAND_DIFFERENCE_THRESHOLD) → 끝에서
                            #  계속 줄이면 0.008° 앞에서 섬 (2026-10-27 WMX 시뮬레이터). 이 속도면 한 주기 1.5e-4 rad (관절)
STEP_TOL = (math.radians(0.003), 0.00003, math.radians(0.005))   # 한 칸 도착: 남은 거리 ≤ 미끄럼 (아래) + 이것 (관절 rad · 위치 m · 방향 rad)
SERVO_COAST_S = 0.0375      # 한 칸: 멈춤을 정한 뒤에도 Servo 가 마지막 속도 명령을 쓰는 시간 (25 ms 주기 1~2 번) — 그만큼 미리 멈춤.
                            #  [2026-10-27 WMX] 보정 전 관절 한 칸이 0.012~0.014° 넘어감 (끝 최소 속도 0.006 rad/s × 1~2 주기)
SERVO_LIMIT_MARGIN = 0.1    # Servo 가 관절 한계 이만큼 (rad) 앞에서 그 관절을 세움 (cr3a_servo.yaml joint_limit_margins) — 한 칸 목표가 넘으면 안 함
SERVO_CODES = {-1: '명령 거절', 1: '특이점 가까움 — 느려짐', 2: '특이점 — 멈춤', 3: '특이점에서 나오는 중 — 느림',
               4: '충돌 가까움 (MoveIt 장면: 로봇·테이블·Thor 상자) — 느려짐', 5: '충돌 (MoveIt 장면) — 멈춤', 6: '관절 한계 — 그 관절 멈춤'}
SERVO_STILL_S = 0.2         # 조그 끝 '관절이 섬' = 지금 엔코더와 이만큼 앞 엔코더 (기록 보간) 의 차가 SERVO_STILL_RAD 아래
SERVO_STILL_RAD = 2e-5
SERVO_START_PAD_S = 0.05    # 조그 시작: 서버 마지막 메시지 (멈춤 꼬리·비상정지 감속) 가 끝나고 이만큼 뒤에 Servo 명령 (팔 제어기가 튀지 않게)
SERVO_WAIT_MAX_S = 3.0      #  그 기다림이 이보다 길면 조그 안 함
STEP_TIME_PAD_S = 3.0       # 한 칸 조그 시간 한도 = 거리 ÷ 최고 속도 + 이것
SERVO_STOP_WAIT_S = (6.0, 2.0)  # Servo 끄기: SIGINT 뒤 기다림 · SIGKILL 뒤 기다림
SERVO_READY_S = 30.0        # Servo 서비스가 뜨기를 기다리는 시간 (지도·계획기 준비 뒤)
SERVO_COAST_STOP_S = 0.8    # (오류·신호로 조그 중에 끝남) 명령이 끊긴 Servo 가 가속 한계로 설 때까지 (끊김 0.2 s + 감속)
JTC_REF_AGE_S = 0.3         # 팔 제어기 기준 위치 (controller_state) 가 이보다 오래면 안 씀 (엔코더로)
JS_WAIT_S = 20.0            # 켤 때 R2 /joint_states 첫 메시지를 기다리는 시간
REPEAT_PAUSE_S = 0.5        # [반복] 끝마다 쉼 (MPC 는 그 자리를 지킴)
# [2026-10-27 R-1 ⑫] 서기·끄기 시간 여유 (전엔 코드 안 숫자) — 명령 꼬리·성형 길이 뒤로 주기를 조금 더 돌려 '서 있음' 을 확인
SETTLE_PAD_S = 0.4          # 정지·일 끝·실시간 회피 끔 (정지점 계산 뒤): 감속 + 성형 길이 + 이것 동안 주기를 더 돌림
SETTLE_OFF_S = 0.6          # 실시간 회피 끔·끄기: 성형 길이 + 이것 동안 주기를 더 돌림 (정지점 계산 전 기본값)
TAIL_PAD_S = 0.2            # 끄기: 마지막 메시지 꼬리 (또는 비상정지 감속) 가 끝나고 이만큼 더 기다린 뒤 R2 끄기
TAIL_WAIT_MAX_S = 3.0       #  (오류로 끝남) 그 기다림의 최대
REBASE_WAIT_S = 0.2         # 서보를 켠 뒤 R2 가 명령을 지금 위치로 맞출 때까지 기다림 (그 뒤 엔코더로 시간표 새로)
IDLE_TAIL_PAD_S = 0.05      # 서 있음: 마지막 메시지 꼬리가 끝나고 이만큼 지나면 명령 상태를 선 자리에서 새로 (_idle)
IDLE_S = 0.05               # 서 있음 (명령을 안 보냄) 주기 — 이 시간 동안 GUI 명령을 기다림 (오면 바로 처리)
STAND_V, STAND_A, STAND_Q = 1e-4, 1e-3, 1e-5   # '서 있나': 성형 전 명령 속도 (rad/s)·가속 (rad/s²) 이 이 아래이고 앞 성형 길이 동안 u 가
                            #  같은 자리 (rad) 안
REFRESH_S = 0.4             # (실시간 회피 끔) 계획 직전 지도 새로 보기: 가상 카메라 = 이 시간만큼의 영상을 지금 렌더
REFRESH_REAL_S = 0.45       #  실제 카메라 = 이 시간 동안 들어오는 영상을 쌓음 (15 fps 의 ~6 장)
SAY_ONCE_S = 2.0            # 같은 거절 글은 이 간격에 한 번 (누르는 동안 조그는 50 ms 마다 옴)
R2_WATCH_S = 0.3            # R2 로그 읽기 간격
R2_SAME_S = 2.0             #  같은 (숫자만 다른) 줄은 이 간격에 한 번만 GUI 로
EXIT_FLUSH_S = 0.5          # 끝내기 전 GUI 로 마지막 글이 갈 시간
PLAN_END_SNAP_RAD = 1e-3    # 계획 경로 끝점이 목표에서 이 안이면 끝점 = 목표 (_post_solve)
PUB_S = 0.045               # 상태 보내기 최소 간격 (주기마다 = 20 Hz)
VOX_S = 0.5                 # 지도 칸 보내기 간격
VOX_MAX = 20000
TRIM_S = 5.0                # 기록 줄이기 간격 (오래 돌아도 메모리가 안 늘게)
# [2026-10-27 R-1 ③] 줄일 때 남기는 수: JTC 기준 기록 (rt_robot.RosRobot.jtc — jtc_ref 는 마지막만 씀, 100~1000 Hz 로 쌓임), 지금 세그먼트의
#  실행된 u 조각 (Stream.trim_executed — 주기마다 하나 = 5 s 분)
TRIM_JTC = 200
TRIM_CHUNKS = 100
OUT_MAX = 400               # GUI 로 보낼 줄 (넘치면 가장 오래된 것을 버림 — 제어 주기가 GUI 를 기다리지 않게)
R2_SH = HERE / 'wmx_r2.sh'
SERVO_LOG_TMP = TMP_DIR / 'cr3a_servo_launch.log'     # 기록 폴더가 없을 때 Servo launch 출력 ([R-1 ⑫] CR3A_TMP_DIR — common.py)
JS_STALE_S = 0.5            # (WMX) /joint_states 가 이보다 오래 안 오면 비상정지
GC_IDLE_S = 30.0            # (WMX) 파이썬 GC 를 끄고 (rt_run.run 과 같음), 명령을 안 보내며 서 있을 때 이 간격으로 모음
REC_MAX_S = 2400.0          # 기록 (--out) 은 켜고 이만큼은 다 남김 (WMX 축 로그 최대 길이 wmx_r2.sh LOG_MAX_S 와 같음), 그 뒤는 GUI 용으로
                            #  줄임. [2026-10-27 사용자 "제한을 40분으로"] 20 분이던 때 실물 실행 (12:08~12:50) 의 실시간 회피 1·2 번째
                            #  주기 기록과 새 물체 칸 기록이 다 지워졌음 (디버깅 D-1)
FRAME_BUDGET_S = 0.025      # (WMX) 계산 시작 이만큼 전까지 온 영상만 그때 처리, 나머지는 메시지를 보낸 뒤 빈 시간에 (지도 처리 p90 21 ms).
                            #  [2026-10-26 WMX 시험] 늦은 메시지 112 번 중 107 번이 계산 바로 앞 영상 처리 때문 (영상 + 계산 > 50 ms)
CABLE_RING_M = (0.20, 0.12)     # (실제 카메라) 받침 둘레 케이블 자리 빼기: 반지름·높이 m. 책상 위 케이블 9~17 cm·높이 4 cm + 받침 오른쪽 뒤에서
                                #  올라오는 케이블 r 12~17 cm·높이 11 cm (깊이가 깜빡여 지키는 중 2 번 링크가 피하며 흔들림 — 5 cm 로 한 3 차 시험).
                                #  21 cm 부터 뒤 검은 상자 (실제 물체) 라 그 앞까지
# [2026-10-26 사용자 "주변에 깊이가 거의 없는 외따로 뜨는 깊이 점을 지도에서 거르는 처리"] (실제 카메라) 깊이 영상 한 장마다, 이웃 픽셀끼리
#  SPECK_DIFF_MM 안으로 이어진 깊이 덩어리가 SPECK_PX 픽셀 이하면 지움 (cv2.filterSpeckles). 팔 바로 위 허공 (뒤 벽) 에 가끔 26~48 픽셀 조각이
#  0.5 m 로 잘못 잡혀 (45 장 중 3 장) 지도에 남았고 시뮬레이션 로봇이 3 번 비킴 (작업기록 5-29). 45 장 시험: 그 조각 다 지움, 늘 보이는 장애물
#  픽셀은 하나도 안 지움, 장애물 점 0.19 % 빠짐 (100 px 부터는 늘 보이는 장애물 가장자리도 지워짐). 한 장 안에서 판단 → 새 물체 늦음 없음.
#  50 px = 1 m 에서 약 2 x 2 cm. 팔 지우기 전 원래 깊이에 함 → 팔·물체에 붙은 점은 큰 덩어리라 남음
SPECK_PX = 50
SPECK_DIFF_MM = 40
SPEED_MIN = 0.01            # 속도 배율 최소 (GUI 슬라이더 1 % — 사용자 2026-11-02 "0.05 가 아니라 0.01 을 최소로"; 전 0.05)
# [2026-10-26 사용자 결정 "모두 cuRobo 계획기·MPC 한계의 95 %로", "가속 ~90%정도가 되도록 저크 값을 좀 더 상향"] GUI 서버의 로봇 설정:
#  속도·가속·저크 한계 × LIMIT_FRAC (cspace *_scale), 저크 한계 × JERK_MULT. 까닭: cuRobo 는 경로 시간을 속도·가속·저크 한계를 다 안 넘게
#  정하는데 cr3a_tool_curobo.yml 의 저크 한계 = 가속 × 10 (10-13 에 넣은 값, Dobot 사양 아님) 이 먼저 걸려 B↔영점이 속도 45 %·가속 81 %·
#  저크 100 % 였음. 실측 (빈 장면, 95 % 한계): 저크 × 1 → 가속 75~81 %, × 1.25 → 87~94 % (B↔영점 1.53 s, 전 1.63 s), × 1.5 → 95 % (상한).
#  ⚠️ cuRoboV2 (0.8.0) 는 velocity_scale 을 두 번 곱함 (kinematics_loader + CSpaceParams — 0.95 → 0.9025 실측) → 그 값은 √LIMIT_FRAC.
#  시뮬레이션 (rt_run) 은 yml 그대로 (지금까지 검증한 값)
LIMIT_FRAC = 0.95
JERK_MULT = 1.25
# [2026-11-02 D-5] 계획 경로는 길이와 상관없이 점 81 개 → 점 사이 = 이동 시간 ÷ 80. 실시간 회피의 경로 따라가기 (rt_mpc.Mpc.advance_ref) 는
#  기준 시각 [− lookahead 0.4 s, + lookahead + 0.5 s] 창 안의 점으로만 나아가서, 점 사이가 ~1.2 s 를 넘으면 (이동 시간 ~96 s 넘게 = 배율 ~0.05 ·
#  한 관절 ~530° 넘게) 기준이 멈추고 3 s 뒤 '막힘' → 다시 계획 (섰다 가기, 코드 읽기로 찾음). 점 사이가 PATH_GAP_MAX_S 를 넘을 때만 같은
#  곡선 위에 PATH_STEP_S 마다 다시 찍음 (densify_path) — 보통 이동 (배율 1~0.1, 점 사이 0.02~0.1 s) 은 그대로
PATH_GAP_MAX_S = 0.5
PATH_STEP_S = 0.1
MOVES = ('goto', 'repeat', 'jog', 'jog_hold', 'goto_pose', 'repeat_pose', 'jog_cart', 'jog_cart_hold')     # 로봇을 움직이는 명령
CART_AXES = ('X', 'Y', 'Z', 'Rx', 'Ry', 'Rz')     # 공구 끝 조그 축 (GUI 단추 이름)
IK_SEEDS = 8                # 공구 끝 목표: cuRobo IK 해 몇 개 중 지금 관절에 가장 가까운 것 (씨앗 32 개는 어차피 다 풂)
IK_TOL_M = 0.0001           # cuRobo IK 성공 = 위치 오차 이 안. cuRobo 기본 5 mm · 0.05 rad (2.9°) 는 조여 씀
IK_TOL_RAD = math.radians(0.05)  # (씨앗 LM 풀이는 0.01 mm 까지 풂 — seed_ik_solver_cfg convergence_*_tolerance)
# [2026-10-28 사용자 "카테시안 목표 이동 하려 하면 좌표 이동 불가라고 에러만 띄우면"] 공구 끝 목표의 IK 해가 지금 자세에서 이만큼 넘게
#  돌아야 하면 (2π 감은 뒤) 거절 — 팔꿈치가 거의 펴진 자세에서 위·안쪽 목표는 어깨를 뒤집은 해 (J1 ~150°·J5 ~175°) 만 있어, 받으면 팔이
#  크게 돌거나 (감긴 값) 계획이 시간 초과까지 실패했음 (감기지 않은 값 J2 −319.6°)
TIP_MAX_J1_DEG = 45.0
TIP_MAX_JOINT_DEG = 90.0
HW_NAME = 'WmxManipulatorArmSystem'     # R2 ros2_control 하드웨어 이름 (cr3a.wmx.ros2_control.xacro 기본값)
ARM = 'movensys_manipulator_arm_controller'
JSB = 'joint_state_broadcaster'
# [2026-10-27 실물 검토 (작업기록 5-32) 뒤 사용자 결정 ①②④⑤⑥]
GUI_LOST_S = 2.0            # ① ping 을 보내던 GUI 가 이만큼 조용하면 (화면 멈춤) 끊긴 것으로 봄. 연결이 닫히면 바로
REFUSE_LOG_S = 60.0         # [R-1 S2] 지금 GUI 가 있는 동안 온 다른 연결을 거절할 때 알림 간격 (거절당한 GUI 는 1 초마다 다시 붙음)
CAM_STALE_S = 1.0           # ④ (실제 카메라) 마지막 깊이 영상이 이보다 오래면: 이동 직전 지도 새로 보기에서 이동 거절 (실시간 회피 끔),
                            #  실시간 회피 중이면 정지 + 회피 끔. 15 fps 의 15 장. [D-7] 카메라 노드도 없으면 (꺼짐) 거절 대신 지도 없이 이동
CHECK_S = 1.4               # [2026-10-27 디버깅 D-1 ㄴ] 실시간 회피 켜기 전·[지도 점검]: (실시간 회피 끔이면) 지도를 이만큼 새로 쌓으며 봄
CHECK_SKIP_S = 0.7          #  그중 앞 이만큼은 판단에서 뺌 — 멈춰 있던 옛 지도 칸이 빠지는 시간. 늘 보이던 칸은 감쇠 0.7/장에서 무게가 1/(1−0.7)
                            #  ≈ 3.3 까지 쌓여 최소 0.1 아래로 가는 데 ~10 장 (0.67 s, 15 fps). 빈 공간 지우기·가려진 칸 잊기가 보통 더 빨리 지움
                            #  (독립 검토: 0.5 s 는 무게 1 가정 → 정리한 케이블의 옛 칸이 남아 거짓 '걸림' 이 될 수 있음)
CHECK_MIN_FRAMES = 5        #  (실제 카메라) 판단에 쓴 영상이 이보다 적으면 (가려진 카메라 — 지도에 안 쌓임) '점검 못 함'
TABLE_Z_DOWN_MAX_M = 0.05   # [2026-10-27 D-2] 카메라 보정의 테이블 높이 (받침 원점 기준) 를 쓰는 범위의 아래 끝 (−5 cm) — 밖이면 잘못 잰
                            #  것으로 보고 0. [D-2 검토] 위 끝은 rt_config.FLOOR_REACT_TOP_MAX_M − dist.act_floor (+1.5 cm: 바닥을 더 올리면
                            #  받침·Link1 충돌 구가 늘 바닥 반응 거리 안 → 계획이 모두 실패)
COMM_WARN_S = (1800.0, 600.0, 120.0)    # ⑤ WMX 무료 라이선스 남은 시간 경고 (엔진 로그 "stop communicating after 6 hours")
COMM_STOP_S = 120.0         # ⑤ 이 아래면 움직이는 일 정지·실시간 회피 끔·새 이동 거절 — 움직이는 중에 통신이 끊겨 R2 가 서보를 갑자기
                            #  끄지 않게 (⑥ R2 고장 = 비상정지 Final)
WMX_LOG_DIR = Path(os.environ.get('CR3A_WMX_LOG_DIR', '/opt/wmx3/log'))     # 엔진 로그 (켤 때마다 <시각>.txt — 무료 라이선스 줄)
LIMIT_FRAC_TOL = 1e-3       # [R-1 ⑦] 켤 때 확인: 계획기·MPC 한계 ÷ 로봇 한계 (URDF 속도·yml 가속·저크) 가 LIMIT_FRAC 에서 이 안
LOG_FMT, LOG_DATEFMT = '[%(asctime)s] %(message)s', '%H:%M:%S'     # [R-1 ⑩] 서버 출력 줄 모양 (전과 같음 — tools.wmx_check 가 읽는 줄 포함)


class Link:
    """GUI 연결 하나: 받기 스레드 → 명령 줄 (cmds), 보내기 스레드 ← 내보낼 줄 (_out, 넘치면 오래된 것을 버림 — 제어 주기가 GUI 를
    기다리지 않게). GUI 가 붙으면 hello (정적인 값) 와 최근 로그부터 보냄.
    [2026-10-27 R-1 ②] 내보낼 줄 = deque(maxlen) + Condition: 전엔 크기 정한 Queue 가 차면 '하나 꺼내고 넣기' 였는데, 두 스레드 (제어·
    R2 로그 감시) 가 같이 넣으면 꺼낸 자리를 다른 스레드가 먼저 채워 넣기가 queue.Full 을 냈음 → 제어 스레드에서 나면 serve 가 Abort 만
    받아 서버가 끝남 (GUI 가 멈춰 줄이 찰 때). deque 는 차면 가장 오래된 것이 저절로 빠져 넣기가 실패하지 않음.
    [2026-10-27 R-1 S2 — GUI 쪽 제안 (b)] 새 클라이언트는 지금 GUI 가 없거나 멈췄을 때 (ping 하던 GUI 가 GUI_LOST_S 넘게 조용함) 만 받음.
    전엔 늘 새 것으로 바꿨음 → 같은 포트에 붙은 두 번째 GUI·시험 클라이언트가 운전자 GUI 를 말없이 끊어 (옛 연결은 '끊김' 도 안 알림)
    로봇이 움직이는 채로 운전자가 화면·[정지] 를 잃을 수 있었음. 거절당한 쪽에는 {'type': 'refused'} 를 보내고 닫음 (출력·운전자 GUI 로그에
    알림 — 같은 쪽이 1 초마다 다시 붙어도 REFUSE_LOG_S 에 한 번)."""

    def __init__(self, port):
        self.listener = Listener(('127.0.0.1', port), authkey=AUTH)
        self.cmds = queue.Queue()
        self._out = collections.deque(maxlen=OUT_MAX)
        self._out_cv = threading.Condition()
        self.conn = None
        self.hello = {}
        self.logs = collections.deque(maxlen=200)
        self._lock = threading.Lock()
        # 지금 GUI 의 마지막 ping 시각 (time.monotonic — [R-1 ⑤]). [R-1 S2 — GUI 쪽 제안 (a)] ping 만 셈: GUI 의 ping 은 화면 스레드가
        #  보내므로, 화면이 멈춰도 누르는 동안 조그 스레드가 보내는 명령이 '살아 있음' 으로 세어지면 'GUI 끊김' 이 안 났음
        self.t_msg = 0.0
        self.pinging = False                # 지금 GUI 가 ping 을 보냄 (= 조용하면 멈춘 것) — ping 없는 시험 클라이언트는 연결 닫힘만 봄
        self.silent_sent = False
        self._refused, self._t_refused_log = 0, -math.inf      # 지금 GUI 가 있는 동안 거절한 수 · 마지막으로 알린 시각
        threading.Thread(target=self._accept, daemon=True, name='gui_accept').start()
        threading.Thread(target=self._send_loop, daemon=True, name='gui_send').start()

    def _alive(self):
        """지금 GUI 가 있고 멈추지 않았나 (ping 을 안 보내는 클라이언트는 연결이 있으면 산 것으로)."""
        return self.conn is not None and (not self.pinging or time.monotonic() - self.t_msg <= GUI_LOST_S)

    def _accept(self):
        while True:
            try:
                c = self.listener.accept()
            except OSError:
                time.sleep(0.5)
                continue
            with self._lock:
                refuse = self._alive()
                if not refuse:
                    old, self.conn = self.conn, None      # (옛 연결의 받기·보내기 스레드는 '끊김' 을 안 알림 — _lost)
            if refuse:
                self._refuse(c)
                continue
            if old is not None:
                self._note(f'앞 GUI 가 {GUI_LOST_S:.0f} s 넘게 응답이 없어 새 GUI 연결로 바꿈')
                try:
                    old.close()
                except OSError:
                    pass
            try:
                c.send({'type': 'hello', **self.hello, 'logs': list(self.logs)})
            except OSError:
                continue
            self.t_msg, self.pinging, self.silent_sent = time.monotonic(), False, False
            if self._refused:
                self._note(f'새 GUI 연결 (앞 GUI 가 있는 동안 다른 연결 {self._refused} 번 거절했음)')
            self._refused = 0
            self.conn = c
            threading.Thread(target=self._recv, args=(c,), daemon=True, name='gui_recv').start()

    def _refuse(self, c):
        """지금 GUI 가 살아 있는 동안 온 다른 연결: 까닭을 보내고 닫음 + 알림 (REFUSE_LOG_S 에 한 번 — 거절당한 GUI 는 1 초마다 다시 붙음)."""
        why = '서버가 이미 다른 GUI 와 연결돼 있음 — 그 GUI 를 닫은 뒤 다시 (같은 포트의 두 번째 GUI·시험은 거절)'
        # [2026-10-28] 세고 알린 뒤 닫음 — 닫힘을 본 쪽 (거절당한 클라이언트·시험) 이 센 수·알림도 보게 (전엔 닫은 뒤 세어 시험이 가끔 앞섬)
        self._refused += 1
        now = time.monotonic()
        if now - self._t_refused_log >= REFUSE_LOG_S:
            self._t_refused_log = now
            text = f'다른 GUI 연결을 거절 ({self._refused} 번째) — 지금 GUI 를 그대로 둠 (운전자 GUI 를 말없이 끊지 않게)'
            self._note(text)
        try:
            c.send({'type': 'refused', 'why': why})
            c.close()
        except (OSError, ValueError):
            pass

    def _note(self, text):
        """연결 일 알림: 출력 (LOGGER) + GUI 로그 줄."""
        LOGGER.info(text)
        self.log(text)

    def _lost(self, c, why):
        """[2026-10-27 ①] 지금 GUI 연결 c 가 끊김 → 명령 줄에 gui_lost (제어 스레드가 정지·실시간 회피 끔). 새 GUI 로 바뀐 옛 연결은 안 알림."""
        with self._lock:
            if self.conn is not c:
                return
            self.conn = None
        self.cmds.put({'cmd': 'gui_lost', 'why': why})

    def check_silent(self):
        """ping 을 보내던 GUI 가 GUI_LOST_S 넘게 조용함 (연결은 열린 채 GUI 가 멈춤) → 한 번만 True."""
        if self.conn is None or not self.pinging or self.silent_sent or time.monotonic() - self.t_msg <= GUI_LOST_S:
            return False
        self.silent_sent = True
        return True

    def _recv(self, c):
        while True:
            try:
                m = c.recv()
            except (EOFError, OSError):
                self._lost(c, '연결 닫힘')
                break
            if not isinstance(m, dict):
                continue
            if m.get('cmd') == 'ping':                  # [R-1 S2 (a)] 살아 있음은 ping 으로만 (화면 스레드)
                if c is self.conn:
                    self.t_msg, self.silent_sent, self.pinging = time.monotonic(), False, True
                continue
            self.cmds.put(m)

    def send(self, m):
        """보낼 줄에 넣음 — 어느 스레드에서든, 막히지도 실패하지도 않음 (차면 가장 오래된 것이 빠짐)."""
        if self.conn is None and m.get('type') not in ('log',):
            return
        with self._out_cv:
            self._out.append(m)
            self._out_cv.notify()

    def log(self, line):
        self.logs.append(line)
        self.send({'type': 'log', 'line': line})

    def _send_loop(self):
        while True:
            with self._out_cv:
                while not self._out:
                    self._out_cv.wait()
                m = self._out.popleft()
            c = self.conn
            if c is None:
                continue
            try:
                c.send(m)
            except (OSError, EOFError, ValueError):
                self._lost(c, '연결 닫힘')


class Abort(Exception):
    """움직이는 일 (goto·repeat) 도중 정지·비상정지·끄기 — 주기 시작 전에 던져서 일을 빠져나옴 (제어 상태는 주기 사이라 그대로 이어짐)."""

    def __init__(self, kind):
        super().__init__(kind)
        self.kind = kind


@dataclasses.dataclass
class JogState:
    """[2026-10-27 R-1 ⑪] 조그 하나 (MoveIt Servo — GuiController.sj). 전엔 키 20 개 dict. 시각은 모두 time.monotonic."""
    mode: str                           # 'hold' 누르는 동안 · 'step' 한 칸
    joint: bool                         # 관절 조그 (아니면 공구 끝)
    ax: int                             # 관절 0~5 · 공구 끝 축 0~5 (X Y Z Rx Ry Rz)
    frame: str | None                   # 공구 끝 'base' 받침 축 · 'tool' 공구 축, 관절 None
    name: str                           # 글 이름 (J1 · X (받침 기준) …)
    t0: float                           # 누른 시각
    t: float                            # 마지막 누르는 동안 명령 시각 (JOG_TIMEOUT_S 안 오면 뗀 것)
    v_cmd: object = 0.0                 # 한 칸: 지난 속도 명령 (관절 rad/s · 공구 끝 (v, w))
    phase: str = 'wait'                 # wait (서버 꼬리 끝나기) → run (Servo 에 명령) → halt (멈추는 중)
    ran: bool = False                   # Servo 에 명령을 보냈음
    said: set = dataclasses.field(default_factory=set)     # 이 조그에서 알린 Servo 상태 코드
    v: float = 0.0                      # 누르는 동안 속도 (rad/s · m/s)
    d: float = 0.0                      # 한 칸 크기 (rad · m)
    q0: np.ndarray | None = None        # 조그 처음 자리 (멈추는 중에 이어 간 조그도 처음 것)
    q_start: np.ndarray | None = None   # 이번 명령 시작 자리 (팔 제어기 기준·엔코더)
    t_run: float = 0.0                  # 명령 시작 시각
    target: object = None               # 한 칸 목표 (관절 rad · 공구 끝 4x4)
    t_max: float = 0.0                  # 한 칸 시간 한도
    t_halt: float | None = None         # 멈추기 시작한 시각
    why: str = ''                       # 멈춘 까닭
    hard: bool = False                  # 정지·비상정지·끊김으로 멈춤 (멈추는 중에 새 조그로 이어 가지 않음)
    estop: bool = False                 # 비상정지로 멈춤 (선 뒤 비상정지 상태)


def _path_spline(t, q):
    """계획 경로 (시각 [N], 관절 [N, J]) → 5 차 스플라인 (점을 지나고 양 끝 속도·가속 0 — 계획 경로는 멈춤에서 멈춤)."""
    from scipy.interpolate import make_interp_spline
    t = np.asarray(t, float)
    zero = [(1, np.zeros(np.shape(q)[1])), (2, np.zeros(np.shape(q)[1]))]
    return make_interp_spline(t - t[0], np.asarray(q, float), k=5, bc_type=(zero, zero))


def _spline_states(spl, xs, end):
    """스플라인의 경로 시각들 xs (0~end 로 자름) 상태 [n, 3, J] (q, v, a). end 넘으면 끝에 섬 (v = a = 0)."""
    xs = np.clip(np.asarray(xs, float), 0.0, end)
    S = np.stack([spl(xs), spl(xs, 1), spl(xs, 2)], 1)
    S[xs >= end, 1:] = 0.0
    return S


def speed_stretch(t, q, s, vmax, amax):
    """[2026-11-02 디버깅 D-5] 속도 배율 s (머리말) — 계획 경로 (시각 [N], 관절 [N, J]) 를 고르게 늘릴 배수 k ≥ 1: 늘린 경로 (속도 ÷ k,
    가속 ÷ k²) 가 속도 한계 vmax × s · 가속 한계 amax × s 안에 드는 가장 작은 값 = max(1, 속도 사용률 / s, √(가속 사용률 / s)) — cuRobo 가
    계획 시간을 정하는 식 (calculate_dt_no_clamp) 과 같은 꼴. 저크 한계는 그대로라 안 봄 (k ≥ 1 → 저크 ÷ k³, 줄기만 함).
    사용률은 경로를 읽는 두 방식 중 큰 것: 점 사이 직선 (실시간 회피 켬 — MPC 당근 Mpc.path_at) · 점을 지나는 5 차 스플라인 1 ms 칸
    (끔 — 명령 = 경로 그대로, _path_spline. 끝 조건이 속도·가속 0 이라 늘린 점의 스플라인 = 늘린 스플라인 → 늘린 뒤 사용률 = 이 값 ÷ k, ÷ k²).
    → (k, 속도 사용률, 가속 사용률) — 사용률은 배율 1 한계 기준 (늘리기 전)."""
    t, q = np.asarray(t, float), np.asarray(q, float)
    if len(t) < 3 or t[-1] - t[0] <= 0.0:
        return 1.0, 0.0, 0.0
    vmax, amax = np.asarray(vmax, float), np.asarray(amax, float)
    dt = np.diff(t)
    v = np.diff(q, axis=0) / dt[:, None]
    a = np.diff(v, axis=0) / (0.5 * (dt[1:] + dt[:-1]))[:, None]
    spl = _path_spline(t, q)
    xs = np.append(np.arange(0.0, t[-1] - t[0], 0.001), t[-1] - t[0])
    vu = max(float(np.max(np.abs(v) / vmax)), float(np.max(np.abs(spl(xs, 1)) / vmax)))
    au = max(float(np.max(np.abs(a) / amax)), float(np.max(np.abs(spl(xs, 2)) / amax)))
    return max(1.0, vu / s, math.sqrt(au / s)), vu, au


def densify_path(t, q, gap_max=None, step=None):
    """[2026-11-02 D-5 사용자 "조건부로 넣는 로직을 추가해줘"] 늘린 경로의 점 사이가 gap_max (PATH_GAP_MAX_S) 를 넘을 때만 같은 5 차
    스플라인 (_path_spline) 위에 step (PATH_STEP_S) 이하 고른 간격으로 점을 다시 찍음 — 넘지 않으면 받은 그대로 (점 하나까지 같음).
    끝 두 점은 받은 값 그대로 (끝점 맞추기 유지). → (시각, 관절)."""
    gap_max = PATH_GAP_MAX_S if gap_max is None else gap_max
    step = PATH_STEP_S if step is None else step
    t, q = np.asarray(t, float), np.asarray(q, float)
    if len(t) < 2 or float(np.max(np.diff(t))) <= gap_max:
        return t, q
    T = float(t[-1] - t[0])
    xs = np.linspace(0.0, T, int(math.ceil(T / step)) + 1)
    Q = _path_spline(t, q)(xs)
    Q[0], Q[-1] = q[0], q[-1]
    return t[0] + xs, Q


def scale_path(t, q, s, vmax, amax):
    """속도 배율 s (머리말) 를 계획 경로에: speed_stretch 배수로 고르게 늘림 → 점 사이가 넓으면 다시 찍기 (densify_path) → 다시 찍었으면
    그 경로를 한 번 더 재서 한계 × s 를 넘으면 그만큼 더 늘림 (새 점의 스플라인은 원래 스플라인과 아주 조금 다를 수 있음 — 보통 × 1).
    → (시각, 관절, 전체 배수, 배율 1 속도 사용률, 가속 사용률)."""
    t, q = np.asarray(t, float), np.asarray(q, float)
    k, vu, au = speed_stretch(t, q, s, vmax, amax)
    t = t[0] + (t - t[0]) * k
    n0 = len(t)
    t, q = densify_path(t, q)
    if len(t) != n0:
        k2 = speed_stretch(t, q, s, vmax, amax)[0]
        t = t[0] + (t - t[0]) * k2
        k *= k2
    return t, q, k, vu, au


# [2026-10-27 R-1 ⑬] 회전 바꾸기 = scipy Rotation (전엔 손으로 쓴 식). cuRobo 쿼터니언은 wxyz, scipy 기본은 xyzw → scalar_first.
#  전 R_quat 은 x·y·z 부호를 비대각 차이로 정해 정확히 180° (그 차이 = 0) 에서는 부동소수 잡음의 부호에 맡겼음 — 비대각이 정확히 대칭인
#  180° (예: 축 (1, −1, 0)) 는 축 (1, 1, 0) 으로 바뀌어 90° 틀림. cm.rpy_R 로 만든 공구 아래 목표 72 개는 잡음 부호가 맞아 다 맞았음
#  (tests/test_srv_rotations.py) — 우연에 기대지 않게. 전 rotvec 도 180° 가까이에서 0/0. scipy 는 어디서나 맞음.
def R_quat(R):
    """회전 3x3 → 쿼터니언 wxyz (cuRobo Pose), w ≥ 0."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(np.asarray(R, float)[:3, :3]).as_quat(canonical=True, scalar_first=True)


def quat_R(q):
    """쿼터니언 wxyz → 회전 3x3."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_quat(np.asarray(q, float), scalar_first=True).as_matrix()


def cart_step(T, axis, d, frame):
    """공구 끝 4x4 를 축 axis (0~5 = X Y Z Rx Ry Rz) 로 d (m 또는 rad) 만큼. frame 'base' = 받침 축 (돌기는 공구 끝 점 둘레),
    'tool' = 공구 자신의 축."""
    T = np.array(T, float)
    if axis < 3:
        e = np.eye(3)[axis]
        T[:3, 3] += d * (e if frame == 'base' else T[:3, :3] @ e)
    else:
        Rd = cm.rot_axis(np.eye(3)[axis - 3], d)[:3, :3]
        T[:3, :3] = Rd @ T[:3, :3] if frame == 'base' else T[:3, :3] @ Rd
    return T


def rotvec(R):
    """회전 3x3 → 회전 벡터 (축 × 각 rad — 한 칸 조그의 남은 방향, 조그로 돈 각). [R-1 ⑬] scipy: 180° 가까이도 맞음."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(np.asarray(R, float)[:3, :3]).as_rotvec()


def rot_angle(Ra, Rb):
    """두 회전 3x3 사이 각 (rad) — [R-1 ④] IK 해 확인."""
    return float(np.linalg.norm(rotvec(np.asarray(Ra, float).T @ np.asarray(Rb, float))))


def clip_norm(v, m):
    n = float(np.linalg.norm(v))
    return v * (m / n) if n > m else v


def ramp_to(v, v0, dv):
    """v0 에서 v 쪽으로 크기 dv 까지만."""
    return v0 + clip_norm(v - v0, dv)


def start_servo(out, log):
    """[2026-10-27] MoveIt Servo (조그) 켜기 — R2 를 켠 뒤 (servo/cr3a_servo.launch.py, 같은 ROS 환경). 반환 Popen (끌 때 stop_servo)."""
    path = Path(out) / 'servo_launch.log' if out else SERVO_LOG_TMP
    with open(path, 'w') as f:                  # (자식은 받은 파일 설명자를 따로 가짐 — 여기서 닫아도 launch 출력은 계속 써짐)
        p = subprocess.Popen(['ros2', 'launch', str(SERVO_LAUNCH)], stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    log(f'MoveIt Servo 켜기 (조그 — 입력성형 없음, 기록 {path})')
    return p


def stop_servo(p, log):
    """Servo 끄기 (R2 보다 먼저): 프로세스 묶음에 SIGINT → SERVO_STOP_WAIT_S 안에 안 끝나면 SIGKILL."""
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(SERVO_STOP_WAIT_S[0])
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.wait(SERVO_STOP_WAIT_S[1])
    except ProcessLookupError:
        return
    log('MoveIt Servo 끔')


class DepthFeed:
    """[2026-10-27 R-1 S2] (실제 카메라) 서버 지도용 깊이 영상 받기 — 깊이 토픽 하나만. GUI 의 camera_io.CameraFeed 는 컬러·보정 정보도
    받아 풀어서 (640x480 컬러 15 fps — rclpy 스레드가 GIL 을 잡고 바꿈) 서버 제어 주기를 늦출 수 있어 따로 둠. 받은 시각 = time.monotonic
    (끊김 감시 — [R-1 ⑤]), 새 영상 알림 = Condition (wait_depth — 지도 점검·새로 보기가 묻지 않고 기다림, [R-1 ⑨]). 토픽 이름·영상 바꾸기는
    camera_io 그대로 (PyQt 없음). 전엔 camera_panel (PyQt 를 가져옴) 의 CameraFeed.
    [2026-10-28 D-4 이미지 기록 (snap_log)·깊이 거르기 (depth_filters)] ring = True 면 최근 깊이 RING_N 장 (원본 — 거르기·팔 지우기 전) 을 둠. 컬러는 여기서 안 받음:
    서버 안에서 컬러 (15 fps · 900 KB) 를 날 것으로만 받아도 늦은 메시지가 ~2 배 (Thor 시험 597 vs 304) → 이미지 기록의 따로 프로세스가 받음."""
    RING_N = 40                 # 최근 영상 수 (15 fps 의 ~2.7 s — 이미지 기록이 트리거 앞뒤 0.5 s 를 꺼낼 때까지 남게)
    color_src = 'ros'           # snap_log: 컬러는 따로 프로세스가 ROS 토픽에서 받음

    def __init__(self, ring=False):
        import camera_io
        self._cio = camera_io
        self._cv = threading.Condition()
        self.depth, self.stamp_depth, self.n_depth = None, 0.0, 0      # 마지막 깊이 (uint16 mm)·찍힌 시각 (헤더 = 이 PC 벽시계)·받은 수
        self.t_depth_mono = -math.inf                                   # 마지막으로 받은 시각 (monotonic)
        self._fps_t = collections.deque(maxlen=30)
        self.keep_ring = bool(ring)
        self.ring = collections.deque(maxlen=self.RING_N)              # (번호, 찍힌 시각, 받은 시각 (벽시계), 깊이) — ring 일 때만
        self.ok, self.err = False, ''
        self._ctx, self._closed = None, threading.Event()               # rclpy Context (받기 스레드가 만듦)·close 불림
        self._th = threading.Thread(target=self._spin, daemon=True, name='depth_feed')
        self._th.start()
        atexit.register(self.close)

    def close(self, timeout=2.0):
        """[2026-10-28 R-1] 프로세스가 끝나기 전 (atexit — 어느 길로 끝나든) rclpy 를 끄고 받기 스레드가 끝나기를 기다림. 전엔 데몬 스레드가
        rclpy (C++) 안에서 기다리는 채로 인터프리터가 끝나 실제 카메라 실행만 끝에 SIGSEGV (139)·abort (134, 'terminate called without an
        active exception') 로 끝났음 (Thor WMX 시뮬레이터 10-27 17:38·10-28 01:50 base, 01:52 new — 가상 카메라는 0)."""
        self._closed.set()
        ctx = self._ctx
        if ctx is not None:
            ctx.try_shutdown()
        self._th.join(timeout)

    def _spin(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image
        except ImportError as e:
            self.err = f'ROS 없음 ({e})'
            return
        ctx = Context()
        rclpy.init(context=ctx)
        self._ctx = ctx
        node = Node('cr3a_server_depth', context=ctx)
        node.create_subscription(Image, f'{self._cio.CAM_TOPIC}/depth/image_rect_raw', self._on_depth, qos_profile_sensor_data)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        self.ok = True
        if self._closed.is_set():                   # (만드는 동안 close 가 불림 — close 는 아직 ctx 를 못 봤을 수 있음)
            ctx.try_shutdown()
        try:
            ex.spin()                               # close 의 try_shutdown 이 깨워 돌아옴
        except Exception:                           # (rclpy ExternalShutdownException 등 — 끄는 중)
            pass
        finally:
            try:
                ex.shutdown(timeout_sec=1.0)
                node.destroy_node()
            except Exception:
                pass

    def _on_depth(self, m):
        d = self._cio.ros_image_to_np(m)
        now = time.monotonic()
        with self._cv:
            self.depth, self.n_depth, self.t_depth_mono = d, self.n_depth + 1, now
            self.stamp_depth = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self._fps_t.append(now)
            if self.keep_ring:
                self.ring.append((self.n_depth, self.stamp_depth, time.time(), d))
            self._cv.notify_all()

    def frame(self, num):
        """[D-4] 번호 num 의 깊이 원본 (기록에 있으면, 없으면 None) — 깊이 거르기 시간 확인이 서버가 건너뛴 영상을 채움."""
        with self._cv:
            for f in reversed(self.ring):
                if f[0] == num:
                    return f[3]
        return None

    def depth_between(self, t0, t1):
        """[D-4] 찍힌 시각이 t0~t1 (벽시계) 인 깊이 [(번호, 찍힌 시각, 받은 시각, 깊이 uint16 mm)]."""
        with self._cv:
            return [f for f in self.ring if t0 <= f[1] <= t1]

    def take_depth(self, after):
        """after 번째 다음 깊이 영상: (번호, 깊이 uint16 mm, 찍힌 시각) — 새 것이 없으면 None."""
        with self._cv:
            return None if self.depth is None or self.n_depth <= after else (self.n_depth, self.depth, self.stamp_depth)

    def wait_depth(self, after, timeout):
        """after 번째 다음 깊이 영상이 올 때까지 (최대 timeout s). 반환: 왔나."""
        with self._cv:
            return self._cv.wait_for(lambda: self.n_depth > after, timeout)

    def fps(self):
        with self._cv:
            t = list(self._fps_t)
        return 0.0 if len(t) < 2 or time.monotonic() - t[-1] > 1.0 else (len(t) - 1) / (t[-1] - t[0])


def drop_specks(depth_mm):
    """(실제 카메라) 깊이 uint16 mm 에서 외따로 뜬 작은 조각을 0 으로 (SPECK_PX · SPECK_DIFF_MM). 반환: (깊이, 지운 픽셀 수)."""
    import cv2
    d = np.minimum(depth_mm, 32767).astype(np.int16)
    cv2.filterSpeckles(d, 0, SPECK_PX, SPECK_DIFF_MM)
    gone = (d == 0) & (depth_mm > 0)
    return np.where(gone, 0, depth_mm).astype(depth_mm.dtype), int(gone.sum())


def run_r2(cmd, out, log):
    """wmx_r2.sh up|down <out> (컨테이너 안 admin, sudo -n) — 출력 줄마다 로그로. 반환: 종료 코드.
    [R-1] 자식은 따로 세션 — 터미널 Ctrl+C (프로세스 묶음 전체에 SIGINT) 가 up·down 을 한가운데서 끊지 않게 (신호는 서버가 받아 up·down 이
    끝난 뒤 처리 — main 의 r2_busy)."""
    p = subprocess.Popen(['bash', str(R2_SH), cmd, str(out)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                         start_new_session=True)
    for line in p.stdout:
        line = re.sub(r'^\[\d\d:\d\d:\d\d\] ', '', line.rstrip())
        if line.strip():
            log('  R2 ' + line.strip())
    return p.wait()


def wmx_comm_deadline(t0, log):
    """[2026-10-27 ⑤] 이번에 켠 WMX 엔진의 통신이 끊기는 시각. 엔진 로그 (WMX_LOG_DIR/<엔진 시작 시각>.txt, 켤 때마다 새 파일) 중 t0 (R2 켜기
    직전) 뒤에 시작한 것 → 시작 시각 + N 시간. N = 로그의 "stop communicating after N hours" (이 PC 의 엔진 로그는 모두 6). 엔진이 로그를
    끌 때 한꺼번에 써서 켤 때는 내용이 비어 있음 (10-27 시험) → 그때는 6 시간으로 봄. 정식 라이선스 (무료 줄 없이 내용이 있음) 면 (None, None)."""
    start, path = None, None
    for p in WMX_LOG_DIR.glob('20*.txt'):
        try:
            s = time.mktime(time.strptime(p.name[:19], '%Y-%m-%d_%H.%M.%S'))
        except ValueError:
            continue
        if s >= t0 - 5.0 and (start is None or s > start):
            start, path = s, p
    if start is None:
        log('이번 WMX 엔진 로그를 못 찾음 → R2 켜기 시각부터 무료 라이선스 6 시간으로 봄')
        return t0 + 6 * 3600.0, 6.0
    try:
        head = path.read_text(errors='ignore')[:20000]
    except OSError:
        head = ''
    m = re.search(r'stop communicating after (\d+(?:\.\d+)?) hours', head)
    if m:
        return start + float(m.group(1)) * 3600.0, float(m.group(1))
    if 'LicenseVerify' in head and 'Free license' not in head:
        return None, None
    return start + 6 * 3600.0, 6.0


def watch_r2(ctl, link, path):
    """R2 로그 (r2_launch.log) 의 오류·경고 줄을 GUI 로그로, 오류면 비상정지 (ctl.r2_fault). 끄는 중 (quit) 의 오류는 무시
    (끌 때 노드가 차례로 꺼지며 오류 줄을 남김 — 10-26 시험)."""
    err = re.compile(r'\[(ERROR|FATAL)\]')
    warn = re.compile(r'\[WARN\].*(Cyclic|MaxAcc|stall|fault|not ready|servo)', re.I)
    ansi = re.compile(r'\x1b\[[0-9;]*m')
    try:
        pos = path.stat().st_size                   # 켤 때까지의 줄은 wmx_r2.sh up 이 이미 봄
    except OSError:
        pos = 0
    last = {}
    while not ctl.quit:
        time.sleep(R2_WATCH_S)
        try:
            with open(path, 'rb') as f:
                f.seek(pos)
                data = f.read()
        except OSError:
            continue
        cut = data.rfind(b'\n') + 1                 # 다 쓴 줄만
        if cut == 0:
            continue
        pos += cut
        for line in data[:cut].decode('utf-8', 'replace').splitlines():
            line = ansi.sub('', line)
            is_err = bool(err.search(line))
            if not (is_err or warn.search(line)) or ctl.quit:
                continue
            text = line.split(']: ', 1)[-1].strip()[:200]
            key = re.sub(r'\d+', '#', text)[:100]
            now = time.monotonic()
            if key not in last or now - last[key] > R2_SAME_S:
                last[key] = now
                link.log(f'R2 {"오류" if is_err else "경고"}: {text}')
            if is_err and ctl.r2_fault is None:
                ctl.r2_fault = text


def urdf_velocity(urdf_path, joints):
    """[R-1 ⑦] URDF 관절 속도 한계 (rad/s, joints 차례) — cuRobo 가 읽는 로봇 속도 한계 (velocity_scale 곱하기 전)."""
    v = {j.get('name'): float(j.find('limit').get('velocity'))
         for j in ET.parse(urdf_path).getroot().iter('joint') if j.find('limit') is not None}
    return np.array([v[n] for n in joints], float)


def limit_frac_errors(checks):
    """[R-1 ⑦] checks = {이름: (쓰는 한계, 로봇 한계)} → 비율이 LIMIT_FRAC 에서 LIMIT_FRAC_TOL 넘게 벗어난 것 {이름: 관절별 비율}."""
    bad = {}
    for k, (got, ref) in checks.items():
        r = np.asarray(got, float) / np.asarray(ref, float)
        if not np.all(np.abs(r - LIMIT_FRAC) <= LIMIT_FRAC_TOL):
            bad[k] = np.round(r, 4).tolist()
    return bad


LOGGER = logging.getLogger('cr3a_rt_server')      # [R-1 ⑩] 서버 출력 (setup_logging). traceback 은 이것으로 바로 = 출력 (파일) 에만


def setup_logging(link):
    """[2026-10-27 R-1 ⑩] 서버 출력 = logging. 부르는 스레드는 줄을 큐에만 넣고 (QueueHandler) 화면·파일 쓰기는 QueueListener 스레드가
    함 — stdout (GUI 가 /tmp 파일로 돌림) 이 느리거나 막혀도 제어 주기가 안 기다림. 큐 크기 제한 없음 (줄을 버리지 않음), 줄 모양은 전과
    같음 '[%H:%M:%S] …' (시각 = 부른 때). GUI 로는 짧은 줄만 바로 (link.log — 다른 GUI 메시지와 차례 그대로; traceback 은 LOGGER 로
    따로 = 파일에만). 반환 (log, listener) — 끝낼 때 listener.stop() (남은 줄을 다 씀)."""
    q = queue.Queue()
    out = logging.StreamHandler(sys.stdout)
    out.setFormatter(logging.Formatter(LOG_FMT, LOG_DATEFMT))
    listener = logging.handlers.QueueListener(q, out)
    LOGGER.handlers[:] = [logging.handlers.QueueHandler(q)]
    LOGGER.setLevel(logging.INFO)
    LOGGER.propagate = False
    listener.start()

    def log(*x):
        line = ' '.join(str(v) for v in x)
        LOGGER.info(line)
        link.log(line)
    return log, listener


def make_controller(link, log, args):
    """무거운 준비 (cuRobo·지도·계획기·MPC: 1~2 분) 와 제어기 만들기 — rt_run.main 과 같은 순서. [2026-10-27 R-1 ⑮] 클래스는 gui_controller.py
    (여기는 만드는 차례만). 가져오는 차례: gui_controller 먼저 — 그 맨 위가 rt_run (cuRobo 실행 설정을 cuRobo 보다 먼저) — 그다음 엔진 나머지."""
    from gui_controller import GuiClock, GuiController, GuiRosRobot, GuiWorld
    from rt_config import FLOOR_REACT_TOP_MAX_M, Config, TaskCfg
    from rt_robot import IdealRobot, WallClock
    from rt_world import JOINTS, robot_cfg
    import shaper_def

    # ---- rt_run.main 과 같은 순서 ----
    c = Config(task=TaskCfg(), style='carrot', model='tool')
    table_z = 0.0
    reg = None
    if args.camera == 'real':
        # [2026-10-27 D-2] 테이블 윗면 높이 (받침 원점 기준) = 카메라 보정이 잰 값 (camera_pose.json table_z_m, 없으면 0 = 예전과 같음).
        #  [D-2 검토] 파일은 여기서 한 번만 읽어 아래 카메라 위치에도 씀 (사이 수십 초에 [등록] 이 끼어도 테이블 높이·카메라 위치가 같은 보정)
        import camera_io as cio                         # [R-1 S2] PyQt 없는 카메라 모듈 (전엔 camera_panel — PyQt 를 가져옴)
        reg = cio.load_pose()
        tz = (reg or {}).get('table_z_m')
        up = FLOOR_REACT_TOP_MAX_M - c.dist.act_floor
        if tz is None:
            log('테이블 높이: 카메라 보정이 안 잼 (예전 보정이거나 테이블 점이 적음) → 0 (예전과 같음)')
        elif -TABLE_Z_DOWN_MAX_M <= float(tz) <= up:
            table_z = float(tz)
        else:
            log(f'⚠️ 카메라 보정의 테이블 높이 {float(tz) * 1000:+.1f} mm 가 쓸 수 있는 범위 ({-TABLE_Z_DOWN_MAX_M * 1000:+.0f} ~ {up * 1000:+.0f} mm) '
                f'밖 — 안 씀 (0). 카메라 보정·받침 모델을 확인')
        # [2026-10-26 사용자 "받침 둘레 케이블 자리 (책상 높이) 를 지도에서 빼고"] 받침 옆 책상 위 케이블이 경계 채우기에 늘 걸려
        #  받침 자리가 '닿음' 이 됐음 (작업기록 5-29) → 받침 축에서 반지름 · 높이 아래를 지도에 안 넣음 (rt_config.MapCfg.base_ring).
        #  [D-2 검토] 높이는 받침 기준 그대로 — 케이블은 받침 뒤 커넥터로 올라가 높이가 받침에 묶임 (테이블을 따라 내리면 깜빡임 여유만 줄어듦)
        c = dataclasses.replace(c, map=dataclasses.replace(c.map, base_ring=CABLE_RING_M, table_z=table_z))
        log(f'실제 카메라: 받침 둘레 케이블 자리 빼기 — 반지름 {CABLE_RING_M[0] * 100:.0f} cm · 받침 원점 위 {CABLE_RING_M[1] * 100:.0f} cm 아래, '
            f'외따로 뜬 깊이 조각 지우기 — {SPECK_PX} 픽셀 이하 (이웃 {SPECK_DIFF_MM / 10:.0f} cm 안으로 이어진 덩어리)')
        log(f'테이블 윗면 = 받침 원점 {table_z * 1000:+.1f} mm (카메라 보정 {(reg or {}).get("time", "?")}) → 지도 바닥 띠 = 테이블 위 '
            f'{c.map.floor_band_m * 100:.1f} cm ({(table_z + c.map.floor_band_m) * 1000:+.1f} mm), 충돌 바닥 (계획기·MPC·안전 검사) = {max(table_z, 0.0) * 1000:+.1f} mm'
            + (' (테이블이 낮아 예전처럼 0 — 안전 쪽)' if table_z < 0 else ''))
    method, rows = args.shaper
    K = shaper_def.kernel(method, rows)
    log(f'입력성형 {shaper_def.label(method, rows)}: 임펄스 {len(K.A)} 개 · 지연 시간 {shaper_def.ms(K.duration_s)} ms')
    robot_dict = robot_cfg(c.model)
    cs = (robot_dict['robot_cfg']['kinematics'] if 'robot_cfg' in robot_dict else robot_dict['kinematics'])['cspace']
    nj = len(cs['max_jerk'])
    cs['max_jerk'] = [x * JERK_MULT for x in cs['max_jerk']]        # 배율 1 기준 한계 (LIMIT_FRAC 설명) — 계획기·MPC·추종기 모두
    cs['velocity_scale'] = [LIMIT_FRAC ** 0.5] * nj                 # cuRoboV2 가 두 번 곱함 → LIMIT_FRAC
    cs['acceleration_scale'] = [LIMIT_FRAC] * nj
    cs['jerk_scale'] = [LIMIT_FRAC] * nj
    # 공구 끝 IK 성공 기준을 조임 (IK_TOL — 계획기의 IK 는 공구 끝 명령만 씀, 계획은 관절 목표 plan_cspace). [2026-10-28 R-1] 계획기를
    #  만들기 전에 넣음 (전엔 warmup 뒤 W.planner.ik_solver.config 를 바꿈 — 그래서 _ik 가 고른 해를 정기구학으로 다시 봄, 그대로 둠)
    W = GuiWorld(robot_dict, c, log=log, ik_tol=(IK_TOL_M, IK_TOL_RAD))     # [D-7] 엔진 World + 카메라 지도 끄기 (gui_controller)
    grid = W.cm.esdf_mpc()
    from rt_mpc import Mpc
    mpc = Mpc(robot_dict, c.mpc, c.dist.act_new, grid, [W.floor])
    jl = W.planner.kinematics.get_joint_limits()
    log(f'MPC {c.mpc.rate_hz:.0f} Hz 준비. 배율 1 한계 (로봇 한계의 {LIMIT_FRAC * 100:.0f} %, 저크 × {JERK_MULT}): 계획기 속도 '
        f'{np.round(jl.velocity[1].cpu().numpy(), 2).tolist()} · 가속 {np.round(jl.acceleration[1].cpu().numpy(), 2).tolist()} · 저크 '
        f'{np.round(jl.jerk[1].cpu().numpy(), 1).tolist()}, MPC·추종기 속도 {np.round(mpc.vmax, 2).tolist()} · 가속 {np.round(mpc.amax, 2).tolist()}')
    # [2026-10-27 R-1 ⑦] 위 velocity_scale = √LIMIT_FRAC 은 cuRoboV2 (0.8.0) 가 속도 배율을 두 번 곱하는 것에 기댄 우회 — 고쳐진 판이 오면
    #  속도 한계가 로봇의 97.5 % 가 되어도 모름. 켤 때 계획기·MPC 한계 ÷ 로봇 한계 (URDF 속도 · yml 가속 · yml 저크 × JERK_MULT — velocity_
    #  scale 등을 곱하기 전) 가 LIMIT_FRAC 인지 보고, 아니면 켜지 않음 (main 이 R2 끄기까지 정리)
    kin = robot_dict['robot_cfg']['kinematics'] if 'robot_cfg' in robot_dict else robot_dict['kinematics']
    ck = [list(cs['joint_names']).index(n) for n in JOINTS]
    pk = [list(jl.joint_names).index(n) for n in JOINTS]
    mk = [list(mpc.names).index(n) for n in JOINTS]
    v_robot = urdf_velocity(kin['urdf_path'], JOINTS)
    a_robot, j_robot = (np.asarray(cs[k], float)[ck] for k in ('max_acceleration', 'max_jerk'))
    bad = limit_frac_errors({'계획기 속도': (jl.velocity[1].cpu().numpy()[pk], v_robot), 'MPC 속도': (np.asarray(mpc.vmax)[mk], v_robot),
                             '계획기 가속': (jl.acceleration[1].cpu().numpy()[pk], a_robot),
                             'MPC 가속': (np.asarray(mpc.amax)[mk], a_robot), '계획기 저크': (jl.jerk[1].cpu().numpy()[pk], j_robot)})
    if bad:
        raise RuntimeError(f'cuRobo 한계 ÷ 로봇 한계가 LIMIT_FRAC {LIMIT_FRAC} 이 아님 {bad} — velocity_scale = √LIMIT_FRAC 우회 (cuRoboV2 가 두 번 '
                           f'곱함) 가 이 cuRobo 판과 안 맞음. rt_server.make_controller 의 cspace 배율을 고친 뒤 켜기')
    log(f'한계 확인: 계획기·MPC 속도·가속·저크 = 로봇 한계 (URDF 속도 {np.round(v_robot, 2).tolist()} · yml 가속·저크) × {LIMIT_FRAC} (±{LIMIT_FRAC_TOL})')
    qa, qb = (np.array(g, float) for g in c.task.goals(c.model))
    if args.robot in ('wmx', 'real'):
        robot, clock = GuiRosRobot(), WallClock()
        q0 = robot.wait_js(JS_WAIT_S)
        log(f'R2 /joint_states 받음{" (실제 로봇 엔코더)" if args.robot == "real" else ""}: {np.round(np.degrees(q0), 2).tolist()}°')
    else:
        robot, clock = IdealRobot({'A': qa, 'B': qb, 'zero': np.zeros(6)}[args.q0]), GuiClock()
    real_cam = mask_q = None
    # [2026-10-28 D-4] 이미지 기록 (snap_log): 실제 카메라 + 기록 폴더 (--out) 일 때. 시험: CR3A_SNAP=0 → 끔
    import snap_log
    snap_on = args.camera == 'real' and bool(args.out) and snap_log.enabled_by_env()
    if args.camera == 'real' and not snap_on:
        log('이미지 기록 끔 (' + ('기록 폴더 없음' if not args.out else 'CR3A_SNAP=0') + ')')
    if args.camera == 'real':
        # [2026-10-26] 실제 고정 카메라: 등록한 카메라 위치 (GUI 카메라 보정 — camera_pose.json) 와 깊이 내부 값으로 지도를 쌓음 (rt_world 렌더 대신)
        import depth_sim as ds
        pose = reg                                      # [D-2 검토] 위에서 테이블 높이와 함께 읽은 것
        if pose is None:
            raise RuntimeError('실제 카메라: 등록된 카메라 위치가 없음 (camera_pose.json) — GUI [카메라 위치 맞추기…]')
        if list(pose['depth_wh']) != [W.cm.s.image_width, W.cm.s.image_height]:
            raise RuntimeError(f'실제 카메라 해상도 {pose["depth_wh"]} ≠ 지도 설정 {[W.cm.s.image_width, W.cm.s.image_height]}')
        T_cam = np.array(pose['T_base_depth'], float)
        W.cm.set_camera(np.array(pose['depth_K'], float).reshape(3, 3).tolist(), ds.pose7(T_cam))
        W.cam.set_cameras([T_cam])                      # (가상 렌더는 안 씀 — 기록·보고서의 카메라 자리)
        # [R-1 S2] 깊이만 받는 서버 쪽 받기 (monotonic 시각·새 영상 알림). [D-4] 최근 깊이 원본도 둠 (이미지 기록·깊이 거르기 시간 확인)
        real_cam = DepthFeed(ring=True)
        if args.mask_q_file:
            mask_q = np.radians(np.asarray(json.loads(Path(args.mask_q_file).read_text())['q_deg'], float))
            log(f'실제 카메라 + 시뮬레이션 로봇: 실제 팔을 고정 자세로 지움 {np.round(np.degrees(mask_q), 2).tolist()}°')
        elif args.robot != 'real':
            raise RuntimeError('실제 카메라 + 시뮬레이션 로봇: 실제 팔 자세 (--mask-q-file, 엔코더 읽기) 가 필요')
        log(f'실제 카메라 지도: 등록 위치 {pose["time"]} (camera_link {pose["camera_link_xyzrpy"]}), 노드 '
            f'{"켜짐" if cio.CameraProcess.pid() else "꺼짐 — GUI [▶ 카메라 켜기]"}')
    # [2026-10-26 사용자 "꺼진 채로 시작"] 실물은 실시간 회피 끔으로 시작 (--realtime auto). on·off 는 시험용
    rt0 = (args.robot != 'real') if args.realtime == 'auto' else args.realtime == 'on'
    ctl = GuiController(c, W, mpc, robot, clock, args.scenario, K, shaping_label=shaper_def.label(method, rows), log=log, link=link,
                        shaper_cfg=(method, rows), out=args.out, record_sent=False, real_cam=real_cam, mask_q=mask_q,
                        kind=args.robot, speed=args.speed, realtime=rt0)
    if args.test_rec_max_s is not None:
        ctl.rec_max_s = args.test_rec_max_s
        log(f'(시험) 기록을 다 남기는 시간 {ctl.rec_max_s:g} s (그 뒤 GUI 용으로 줄임)')
    ctl.rec['settings']['camera'] = args.camera
    ctl.rec['settings']['speed0'] = ctl.speed
    if args.camera == 'real':
        ctl.setup_depth_filters()                   # [2026-10-28 D-4] 가림 띠 거르기 (환경 변수 CR3A_DEPTH_* — depth_filters)
    if snap_on:
        ctl.setup_snap(Path(args.out) / 'snap', reg)
    log(f'속도 배율 {ctl.speed:.2f} (로봇 명령 속도·가속 한계 × 배율 — 목표 이동은 배율 1 경로를 그 한계 안으로 고르게 늘림, 입력성형은 그 뒤), 실시간 회피 {"켬" if rt0 else "끔 (명령할 때만 움직임)"}')
    # 공구 끝 IK 를 미리 한 번 (cuRobo 첫 풀이는 CUDA 그래프를 잡느라 느림 — 첫 공구 끝 조그가 늦지 않게). 성공 기준 IK_TOL 은 World 를
    #  만들 때 넣었음 (위)
    t_ik, q_now = time.monotonic(), np.asarray(robot.measured(), float)
    T_now = ctl._tip(q_now)
    q_back, why = ctl._ik(T_now, q_now)
    t_ik1 = time.monotonic()
    ctl._ik(T_now, q_now, n=1)
    log(f'공구 끝 IK (cuRobo) 준비 {t_ik1 - t_ik:.1f}초, 한 번 풀기 {(time.monotonic() - t_ik1) * 1000:.0f} ms — 지금 공구 끝 '
        f'{np.round(T_now[:3, 3] * 1000, 1).tolist()} mm' + (f', 되돌린 관절 차 {np.degrees(np.max(np.abs(q_back - q_now))):.3f}° ({why})'
                                                            if q_back is not None else f' (IK 실패: {why})'))
    link.hello.update({'robot': args.robot, 'cam_T': np.asarray(W.cam.cam_T[0], float).tolist(), 'shaper': [method, rows], 'table_z': table_z,
                       'realtime': rt0, 'speed': ctl.speed, 'ready': False, 'scenario': args.scenario, 'out': args.out, 'camera': args.camera,
                       'qA': qa.tolist(), 'qB': qb.tolist(), 'qlim': [ctl.mpc.qlim[0].tolist(), ctl.mpc.qlim[1].tolist()]})
    link.send({'type': 'hello', **link.hello})
    return ctl


def main():
    ap = argparse.ArgumentParser(description=__doc__.split(chr(10))[0])
    ap.add_argument('--robot', choices=['ideal', 'wmx', 'real'], default='ideal')
    ap.add_argument('--speed', type=float, default=1.0, help='속도 배율 처음 값 (0.01~1, GUI 슬라이더 — 처음 0.2)')
    ap.add_argument('--realtime', choices=['auto', 'on', 'off'], default='auto',
                    help='실시간 회피 처음 상태: auto = 실물 (real) 은 끔, 나머지는 켬 (on·off 는 시험용)')
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--shaper', default='["ETM4", [[true, 8.0], [false, 15.0], [false, 23.0]]]',
                    help='["방식", [[켬, Hz] × 3]] (GUI 의 적용 중 값)')
    ap.add_argument('--q0', choices=['A', 'B', 'zero'], default='A', help='이상적 로봇의 처음 자세')
    ap.add_argument('--scenario', default='moving3t', help='시험 물체 (rt_scenarios)')
    ap.add_argument('--out', default=None, help='기록 폴더 (wmx 기본 = ~/workspaces/wmx_r2_logs/gui_wmx_<시각>, ideal 기본 = 기록 안 함)')
    ap.add_argument('--camera', choices=['virtual', 'real'], default='virtual',
                    help='지도 카메라: virtual = 가상 (rt_world 렌더), real = 실제 고정 카메라 (camera_pose.json, GUI 가 켠 노드)')
    ap.add_argument('--mask-q-file', default=None,
                    help='실제 카메라 + 시뮬레이션 로봇: 실제 팔을 지울 고정 자세 (JSON {"q_deg": [6]}, GUI 의 마지막 엔코더 읽기)')
    # [2026-10-27 R-1 ⑧③] 시험용 (전엔 환경 변수 CR3A_COMM_LEFT_S)
    ap.add_argument('--test-comm-left-s', type=float, default=None,
                    help='(시험) WMX 통신 남은 시간을 이 초로 — 경고·정지·거절 확인 (wmx·real)')
    ap.add_argument('--test-rec-max-s', type=float, default=None,
                    help=f'(시험) 기록 (--out) 을 다 남기는 시간 (기본 {REC_MAX_S:g} s) — 그 뒤 GUI 용으로 줄이기 확인')
    args = ap.parse_args()
    m, r = json.loads(args.shaper)
    args.shaper = (m, [(bool(u), float(f)) for u, f in r])
    if args.robot in ('wmx', 'real') and args.out is None:
        args.out = str(LOG_ROOT / time.strftime(f'gui_{args.robot}_%Y%m%d_%H%M%S'))
    link = Link(args.port)
    log, listener = setup_logging(link)
    ctl = None
    # [R-1 ⑥] 켜는 중 받은 신호. 처리기가 던진 SystemExit 은 파이썬이 weakref 콜백·__del__ 을 도는 중에 받으면 'Exception ignored' 로
    #  삼켜짐 — [2026-10-27 Thor 시험] 켜는 중 SIGTERM 이 그렇게 사라져 서버가 끝까지 켜져 명령을 기다렸음 → 켜는 단계마다 다시 봄 (startup_check)
    startup_sig = []
    # [R-1] R2 를 켜거나 (wmx_r2.sh up) 끄는 (finally — 마지막 감속 기다림·Servo·down) 중: 신호를 적어만 두고 그 일을 끝까지 함. 전엔 up 도중
    #  신호로 끝나면 r2_up 이 아직 거짓이라 down 을 안 해 R2 (서보 ON) 가 서버 없이 남았고, finally 의 down 도 두 번째 신호에 끊길 수 있었음
    r2_busy = [False]

    def on_signal(signum, _frame):
        """[2026-10-27 R-1 ⑥] 전엔 SIGTERM = 어디서든 SystemExit (주기 한가운데서 끊겨 serve 의 성형 거친 정지를 건너뜀). 이제 명령을 받는
        중 (serve) 첫 신호 = 이름만 넣음 (잠금·로그 없이 — 어디서든 끼어들 수 있음) → 다음 주기 _poll 이 [■ 시스템 종료] 처럼 끄기.
        켜는 중 (serve 전) 이나 두 번째 신호 = 전처럼 바로 끝 (finally 에서 저장·R2 끄기). R2 를 켜거나 끄는 중 (r2_busy) 은 적어만 둠
        (켜는 중이면 up 이 끝난 바로 뒤 startup_check 가 끝냄)."""
        if r2_busy[0]:
            startup_sig.append(signum)
            return
        if ctl is not None and ctl.serving and ctl.sig_quit is None:
            ctl.sig_quit = signal.Signals(signum).name
            return
        startup_sig.append(signum)
        if signum == signal.SIGINT:
            raise KeyboardInterrupt
        raise SystemExit(143)

    def startup_check():
        """켜는 중 받은 신호가 삼켜져 아직 안 끝났으면 지금 끝 (켜는 단계 사이에서)."""
        if startup_sig:
            log(f'켜는 중 {signal.Signals(startup_sig[0]).name} 받음 → 끝 (finally 에서 정리)')
            if startup_sig[0] == signal.SIGINT:
                raise KeyboardInterrupt
            raise SystemExit(143)
    signal.signal(signal.SIGTERM, on_signal)
    # [2026-10-27] 셸에서 '&' 로 띄우면 SIGINT '무시' 가 물려지고 (exec 를 지나도 남음) 이 서버가 띄우는 R2 launch 까지 SIGINT 로 안 꺼져
    #  wmx_r2.sh 가 30 s 뒤에야 노드에 직접 SIGINT 를 보냈음 (시험 1·2). 처리기는 exec 때 기본으로 돌아가므로 여기서 처리기를 둠 (Ctrl+C = 끄기)
    signal.signal(signal.SIGINT, on_signal)

    try:
        mode = {'ideal': '이상적 모드: 로봇·WMX 없음', 'wmx': 'WMX 시뮬레이터: R2 → WMX, 실제 로봇 안 움직임',
                'real': '★ 실제 로봇 ★ R2 → WMX (EtherCAT) → CR3A, 서보 ON'}[args.robot]
        log(f'서버 시작 ({mode}) — 지도·계획기·MPC 준비 (1~2 분), 연결 127.0.0.1:{args.port}' + (f', 기록 {args.out}' if args.out else ''))
        link.hello.update({'robot': args.robot, 'ready': False, 'out': args.out})
        if args.robot == 'real' and args.camera != 'real':
            # [2026-10-27 ④] 실물은 실제 카메라 지도만 — 가상 카메라면 진짜 책상·벽·물건을 못 보고 가상 기둥만 피해 계획함 (GUI 도 막음)
            log('✗ 실제 로봇은 실제 카메라 지도로만 켬 (--camera real) — 서버 끝')
            time.sleep(EXIT_FLUSH_S)
            sys.exit(2)
        r2_maybe, comm, servo = False, (None, None), None
        try:
            if args.robot in ('wmx', 'real'):
                log('R2 켜기 (안전장치 → R2 → 시뮬레이터 확인 → WMX 축 로그)' if args.robot == 'wmx' else
                    'R2 켜기 (Module.ini EtherCAT → R2 = 통신·서보 ON → 엔진 플랫폼 기록 → WMX 축 로그)')
                t_up = time.time()
                # [R-1] up 을 부르기 전부터 'R2 가 켜졌을 수 있음' — 어느 길로 끝나든 finally 가 down (down 은 우리 R2 (PID 파일의 세션) 만 끄고
                #  없으면 아무것도 안 함: 반쯤 된·실패한 up 뒤에도 됨). up 이 도는 동안 신호는 적어만 둠 — up 이 반쯤 된 채 down 이 끼어들면 (경쟁)
                #  down 이 아직 없는 R2 를 못 보고 넘어간 뒤 up 이 켜서 남을 수 있음. 적어 둔 신호는 up 이 끝난 바로 뒤 startup_check 가 처리
                r2_maybe = True
                r2_busy[0] = True
                try:
                    rc = run_r2('up' if args.robot == 'wmx' else 'up-real', args.out, log)
                finally:
                    r2_busy[0] = False
                if rc != 0:
                    log(f'R2 를 못 켬 (코드 {rc}) — 서버 끝')
                    time.sleep(EXIT_FLUSH_S)
                    sys.exit(1)
                startup_check()
                comm = wmx_comm_deadline(t_up, log)
                if args.test_comm_left_s is not None:           # (시험용) 통신 남은 시간을 짧게 — 경고·정지·거절 확인
                    comm = (time.time() + args.test_comm_left_s, comm[1] or 6.0)
                    log(f'(시험) WMX 통신 남은 시간을 {args.test_comm_left_s:.0f} s 로 둠')
                if comm[0] is not None:
                    log(f'WMX 무료 라이선스: 엔진이 {comm[1]:g} 시간 뒤 ({time.strftime("%H:%M", time.localtime(comm[0]))} 쯤) 통신을 끊음 '
                        f'— 남은 시간은 GUI 위쪽, {COMM_STOP_S / 60:.0f} 분 전부터 새 이동 안 받음')
                    # [R-1 ⑤] 끊기는 시각 (엔진 로그 이름 = 벽시계) 을 monotonic 기준으로 한 번 바꿔 둠 — 벽시계가 바뀌어도 남은 시간이 맞게
                    comm = (time.monotonic() + (comm[0] - time.time()), comm[1])
                servo = start_servo(args.out, log)              # 지도·계획기 준비 (1~2 분) 동안 같이 켜짐
                startup_check()
            ctl = make_controller(link, log, args)
            startup_check()
            ctl.comm_deadline, ctl.comm_hours = comm
            if args.robot in ('wmx', 'real'):
                threading.Thread(target=watch_r2, args=(ctl, link, Path(args.out) / 'r2_launch.log'), daemon=True, name='watch_r2').start()
                ctl.servo_ok = servo.poll() is None and ctl.robot.servo_ready(SERVO_READY_S)
                log('MoveIt Servo 준비 — 조그는 Servo 로 (입력성형 없음), 목표 이동만 입력성형' if ctl.servo_ok else
                    '✗ MoveIt Servo 가 안 뜸 — 조그 안 됨 (servo_launch.log 확인), 목표 이동은 됨')
                startup_check()
            ctl.serve()
        except SystemExit:
            raise
        except Exception as e:
            log(f'서버 오류로 끝남: {e!r}')
            LOGGER.error('  (서버 오류 — traceback)', exc_info=True)
            if startup_sig:     # [R-1 ⑥] 켜는 중 신호가 모듈 가져오기 한가운데를 끊어 난 오류 (Thor 시험: sympy 반쯤 가져옴) → 신호로 끝낸 것과 같게
                raise SystemExit(143) if startup_sig[0] != signal.SIGINT else KeyboardInterrupt from e
            raise
        finally:
            try:
                if ctl is not None:
                    ctl.quit = True
                    if ctl.snap is not None:            # [D-4] 남은 이미지 기록을 넘기고 다 쓸 때까지 (저장 전 — rec['snaps'])
                        try:
                            ctl.snap.close()
                            ctl.rec['snap_stats'] = ctl.snap.stats()
                        except Exception as e:
                            log(f'이미지 기록 끝내기 오류: {e!r}')
                    if args.out:
                        try:
                            ctl.save_all(args.out)
                        except Exception as e:
                            log(f'기록 저장 오류: {e!r}')
                            LOGGER.error('  (기록 저장 오류 — traceback)', exc_info=True)
            finally:
                # [R-1] 여기부터 신호는 적어만 둠 (r2_busy) — 마지막 감속 기다림·Servo 끄기·R2 down 을 끝까지 함 (R2 를 서버 없이 남기지 않게,
                #  SIGKILL 만 못 막음). 저장 도중 두 번째 신호로 끊겨도 이 안쪽 finally 는 돎
                r2_busy[0] = True
                if ctl is not None and not ctl.ideal:
                    # [2026-10-27 ②] 오류로 끝나도 마지막 메시지의 멈춤 꼬리 (JTC 가 받은 대로 끝까지 실행 — 메시지마다 속도 0 으로 끝남) 가 끝난
                    #  뒤 R2 를 끔. 바로 끄면 R2 가 버퍼를 즉시 끊어 (사실상 감속 없이) 급정지. 정상 끄기는 serve 가 이미 기다렸음 (0 s)
                    try:
                        wait = min(max(ctl.stream.tail_done, ctl._estop_until) + TAIL_PAD_S - ctl.clock.now(), TAIL_WAIT_MAX_S)
                        if wait > 0:
                            log(f'마지막 감속이 끝날 때까지 {wait:.1f} s 기다린 뒤 R2 끄기')
                            time.sleep(wait)
                    except Exception:
                        pass
                    try:
                        ctl.robot.close()
                    except Exception:
                        pass
                if servo is not None:
                    if ctl is not None and ctl.sj is not None:
                        time.sleep(SERVO_COAST_STOP_S)      # (오류·신호로 조그 중에 끝남) 명령이 끊긴 Servo 가 가속 한계로 설 때까지
                    stop_servo(servo, log)
                if r2_maybe:
                    log('R2 끄기 (서보 OFF → 축 로그 끝 → R2 → Module.ini)')
                    run_r2('down', args.out, log)
                log('서버 프로세스 끝')
                time.sleep(EXIT_FLUSH_S)
        if ctl.sig_quit:                            # 신호로 끈 것은 전처럼 128 + 신호 번호로 끝 (SIGTERM 143)
            sys.exit(128 + signal.Signals[ctl.sig_quit].value)
    finally:
        listener.stop()                             # 남은 출력 줄을 다 씀


if __name__ == '__main__':
    sys.modules.setdefault('rt_server', sys.modules[__name__])     # [R-1 ⑮] gui_controller 의 'from rt_server import …' 가 이 모듈을 씀
    main()
