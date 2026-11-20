#!/bin/bash
# [2026-10-27 R-1 리팩터링] GUI 안전 시험 (a)~(d) + 시안 캡처 — wmx_r2 컨테이너 안, 화면 없이 (QT_QPA_PLATFORM=offscreen).
# 실물과 안 섞이게: 시험 포트 (CR3A_GUI_PORT, 기본 47873 — 47861 은 거부), 임시 폴더 (CR3A_TMP_DIR — 카메라 PID·서버 로그·GUI 로그),
# Qt 설정 (XDG_CONFIG_HOME), ROS 도메인 (ROS_DOMAIN_ID 93 — 실물 카메라 토픽을 안 받음), .pyc 안 씀 (PYTHONDONTWRITEBYTECODE).
# R2·WMX·실제 카메라 노드는 건드리지 않음 (서버는 이상적 로봇 · 가상 카메라).
#   bash tests/run_r1_tests.sh <결과 폴더> [시험 …]      시험 = cam close refuse live mock (기본 모두)
source /opt/wmx_r2_env.sh >/dev/null 2>&1      # (set -u 보다 먼저 — ROS 설정 스크립트가 없는 변수를 읽음)
set -u
GUI=$(cd "$(dirname "$0")/.." && pwd)
OUT=${1:?결과 폴더}
shift || true
WHICH=${*:-cam close refuse live mock}
export CR3A_GUI_PORT=${CR3A_GUI_PORT:-47873}
if [ "$CR3A_GUI_PORT" = 47861 ]; then echo "47861 은 실물 서버 포트 — 시험 안 함"; exit 2; fi
export CR3A_TMP_DIR=$OUT/tmp XDG_CONFIG_HOME=$OUT/xdg ROS_DOMAIN_ID=93 QT_QPA_PLATFORM=offscreen PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=/home/admin/workspaces/movensys_ws/curobo_v2_shaping:${PYTHONPATH:-}
mkdir -p "$CR3A_TMP_DIR" "$XDG_CONFIG_HOME"
cd "$GUI" || exit 1
rc=0
for t in $WHICH; do
  case $t in
    cam)   echo "### (b) 카메라 끄기"; python3 tests/test_cam_stop.py ;;
    close) echo "### (c) 보정 창 열린 채 닫기"; python3 tests/test_close_calib.py ;;
    refuse) echo "### 서버가 연결을 거절 (다른 GUI) — 알림·다시 안 붙음·[↻ 다시 연결]·닫기"; python3 tests/test_close_calib.py --refuse ;;
    live)  echo "### (d)+(a) 이상적 서버 · 누르는 동안 조그 멈춤"; python3 tests/test_safety_live.py ;;
    mock)  echo "### 시안 캡처"; rm -rf "$OUT/mock"; python3 cr3a_gui.py --mock --shot "$OUT/mock" && ls "$OUT/mock" | wc -l ;;
    *)     echo "모르는 시험 $t"; false ;;
  esac
  r=$?
  echo "### $t: 끝 코드 $r"
  [ $r -ne 0 ] && rc=1
done
exit $rc
