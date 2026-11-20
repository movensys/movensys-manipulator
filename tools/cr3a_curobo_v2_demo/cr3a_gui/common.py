#!/usr/bin/env python3
"""CR3A GUI·서버가 같이 쓰는 값과 작은 함수 (화면 없음 — rt_server 도 가져다 쓸 수 있게 PyQt 를 맨 위에서 가져오지 않음).

[2026-10-27 R-1 리팩터링] 여러 파일에 똑같이 있던 것을 한곳에: 연결 포트·인증 (cr3a_gui·rt_server), 경로 (/tmp·로그 폴더 — 환경 변수로
바꿀 수 있게), camera_link → 광학 좌표 (cr3a_gui·camera_fit·camera_panel), BGR 영상 → QImage (cr3a_gui·camera_panel·시안).
환경 변수 (없으면 예전과 같은 값)
  CR3A_GUI_PORT   GUI ↔ 서버 포트 (기본 47861). 시험은 다른 포트로 — 실물 서버에 붙지 않게
  CR3A_TMP_DIR    서버 로그·GUI 로그·카메라 노드 PID·설정 파일 폴더 (기본 /tmp). 시험은 다른 폴더로 — 실물 카메라 노드·서버 로그를 안 건드리게
  CR3A_LOG_ROOT   기록 폴더 (기본 /home/mic-735d/workspaces/wmx_r2_logs — 호스트·컨테이너 같은 경로)
"""
import math
import os
from pathlib import Path

import numpy as np

import cr3a_model as cm

PORT = int(os.environ.get('CR3A_GUI_PORT', 47861))                   # GUI ↔ rt_server (127.0.0.1, multiprocessing.connection)
AUTH = b'cr3a-gui'
TMP_DIR = Path(os.environ.get('CR3A_TMP_DIR', '/tmp'))
LOG_ROOT = Path(os.environ.get('CR3A_LOG_ROOT', '/home/mic-735d/workspaces/wmx_r2_logs'))
SERVER_LOG = TMP_DIR / 'cr3a_rt_server.log'                          # [▶ 시스템 시작] 이 띄운 서버의 출력
GUI_LOG = TMP_DIR / 'cr3a_gui_events.log'                            # GUI 로그 창 + 예외 (run_gui.sh 의 /tmp/cr3a_gui.log = 표준 출력과 다른 파일)
# camera_link (x 앞, y 왼쪽, z 위) → 깊이 광학 좌표 (z 앞, x 오른쪽, y 아래). 받침 ← 광학 = 받침 ← camera_link @ 이것
LINK_TO_OPTICAL = cm.rpy_R(-math.pi / 2, 0.0, -math.pi / 2)


def link_from_optical(T):
    """받침 ← 깊이 광학 좌표 4x4 → 받침 ← camera_link 4x4."""
    R = np.eye(4)
    R[:3, :3] = LINK_TO_OPTICAL.T
    return np.asarray(T, float) @ R


def to_qimage(bgr):
    """BGR uint8 영상 (h, w, 3) → QImage (복사본 — numpy 배열이 사라져도 됨). PyQt 는 여기서만 가져옴 (서버는 안 부름)."""
    from PyQt5 import QtGui
    a = np.ascontiguousarray(bgr)
    h, w = a.shape[:2]
    return QtGui.QImage(a.data, w, h, a.strides[0], QtGui.QImage.Format_BGR888).copy()
