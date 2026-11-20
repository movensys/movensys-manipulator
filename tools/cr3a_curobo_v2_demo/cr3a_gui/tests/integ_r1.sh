#!/bin/bash
# [2026-10-28 R-1 통합 시험] HEAD 전체 (GUI + 서버 + 엔진 + GPU 3D) 를 WMX 시뮬레이터로 — wmx_r2 컨테이너 안 (admin).
#   호스트에서는 tests/integ_r1_host.sh 가 부름 (코어 파일 확인 포함). 직접: bash tests/integ_r1.sh <결과 폴더> [A] [B]
# 부분마다 켜기 전: Module.ini (EtherCAT disable = 1, 켜진 플랫폼 = 시뮬레이터 하나) · 남은 프로세스·포트 없음 — 아니면 그 자리에서 끝.
# 실물과 안 섞이게: 시험 포트 (CR3A_GUI_PORT 47873 — 47861 거부), 임시 폴더 (부분마다 CR3A_TMP_DIR — 서버 로그·GUI 로그·카메라 PID),
#   Qt 설정 (XDG_CONFIG_HOME), 기록 (CR3A_LOG_ROOT = <결과>/rec), WMX_AUTO_MODE=0 (wmx_r2.sh 가 Module.ini 를 스스로 바꾸지 않음),
#   .pyc 안 씀 (movensys_ws 에 쓰지 않게). 3D = QT_QPA_PLATFORM=offscreen + DISPLAY=:1 (창 없이 GPU).
# 끝난 뒤: 부분마다 tools.wmx_check (성형 확인·R2 오류·실시간), A 는 cr3a_rt_shaping tools.rt_report (닿음·늦은 메시지) → 요약.
source /opt/wmx_r2_env.sh >/dev/null 2>&1      # (set -u 보다 먼저 — ROS 설정 스크립트가 없는 변수를 읽음)
set -u
GUI=$(cd "$(dirname "$0")/.." && pwd)
RT=$(cd "$GUI/../cr3a_rt_shaping" && pwd)
OUT=${1:?결과 폴더}
shift || true
PARTS=${*:-A B}
export CR3A_GUI_PORT=${CR3A_GUI_PORT:-47873}
if [ "$CR3A_GUI_PORT" = 47861 ]; then echo "47861 은 실물 서버 포트 — 시험 안 함"; exit 2; fi
export XDG_CONFIG_HOME=$OUT/xdg CR3A_LOG_ROOT=$OUT/rec WMX_AUTO_MODE=0 PYTHONDONTWRITEBYTECODE=1 QT_QPA_PLATFORM=offscreen
export DISPLAY=${DISPLAY:-:1}
export PYTHONPATH=/home/admin/workspaces/movensys_ws/curobo_v2_shaping:${PYTHONPATH:-}
mkdir -p "$OUT/xdg" "$OUT/rec"
cd "$GUI" || exit 1
echo "### 통합 시험 $(date '+%F %T') — GUI $GUI · 엔진 $RT · 결과 $OUT · 포트 $CR3A_GUI_PORT · ROS 도메인 ${ROS_DOMAIN_ID:-?}"
rc=0
for P in $PARTS; do
  echo "### 켜기 전 확인 (부분 $P)"
  python3 tests/integ_check.py ini || { echo "✗ Module.ini — 부분 $P 안 함"; rc=1; break; }
  python3 tests/integ_check.py procs || { echo "✗ 남은 프로세스·포트 — 부분 $P 안 함"; rc=1; break; }
  export CR3A_TMP_DIR=$OUT/tmp$P
  mkdir -p "$CR3A_TMP_DIR"
  echo "### 부분 $P (GUI)"
  python3 tests/integ_r1.py --part "$P" --out "$OUT"
  r=$?
  echo "### 부분 $P 끝 코드 $r"
  [ $r -ne 0 ] && rc=1
  REC=$(cat "$OUT/rec_$P.txt" 2>/dev/null)
  if [ -n "$REC" ] && [ -f "$REC/wmx_axis.csv" ]; then
    echo "### wmx_check (부분 $P) → $REC/wmx_check.json"
    PYTHONPATH=$RT:$PYTHONPATH python3 -m tools.wmx_check "$REC" > "$OUT/wmx_check_$P.txt" 2>&1 || { echo "✗ wmx_check 실패"; tail -5 "$OUT/wmx_check_$P.txt"; rc=1; }
    if [ "$P" = A ]; then
      echo "### rt_report (부분 A) → $REC/summary.json · report.html"
      (cd "$RT" && python3 -m tools.rt_report "$REC") > "$OUT/rt_report_A.txt" 2>&1 || { echo "✗ rt_report 실패"; tail -5 "$OUT/rt_report_A.txt"; rc=1; }
    fi
  else
    echo "✗ 부분 $P 기록 폴더 · WMX 축 로그 없음 ($REC)"; rc=1
  fi
done
echo "### 끝난 뒤 확인 (엔진·R2·Servo·카메라 노드·서버·GUI 가 없어야, Module.ini 그대로)"
python3 tests/integ_check.py procs || rc=1
python3 tests/integ_check.py ini || rc=1
python3 tests/integ_check.py summary "$OUT" || rc=1
echo "### 통합 시험 끝 코드 $rc ($(date '+%T'))"
exit $rc
