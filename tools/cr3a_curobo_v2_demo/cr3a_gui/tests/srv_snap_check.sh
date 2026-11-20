#!/bin/bash
# [2026-10-28 D-4] 이미지 기록 통합 시험 — WMX 시뮬레이터만 (컨테이너 wmx_r2 안 admin), 실제 고정 카메라 토픽 (GUI 가 켠 노드 — 안 건드림).
# 실물 포트 47861 안 씀. 기록 끔 (CR3A_SNAP=0) · 켬 (near 기준을 CR3A_SNAP_NEAR_MM 로 크게 = 억지로 자주 기록) 을 같은 순서로 돌려
# 주기 시간·늦음을 비교 (srv_snap_check.py).   srv_snap_check.sh <결과 폴더> [포트 47891] [RUN_S 60] [켬 near mm 2000]
# [D-4] 다섯째 인자 dfilt → 비교 대상이 깊이 거르기: off = CR3A_DEPTH_TEMPORAL=0 CR3A_DEPTH_BAND=0, on = 기본 설정 (이미지 기록은 둘 다 켬·기본 기준)
ROOT=${1:?결과 폴더}
P=${2:-47891}
RUN_S=${3:-60}
NEAR_MM=${4:-2000}
WHAT=${5:-snap}
[ "$P" = 47861 ] && { echo '실물 포트'; exit 2; }
GUI=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$ROOT"
source /opt/wmx_r2_env.sh >/dev/null 2>&1
set -u
export PYTHONPATH=/home/admin/workspaces/movensys_ws/curobo_v2_shaping:${PYTHONPATH:-} PYTHONDONTWRITEBYTECODE=1
platforms() { awk -F= '/^\[/{s=$0} $1~/DllName/{d=$2} $1~/disable/{gsub(/[ \t\r]/,"",$2); if (s~/Platform/ && $2=="0") print d}' /opt/wmx3/Module.ini | tr -d ' \r' | sort -u | tr '\n' ' '; }
ec_disabled() { awk -F= '/^\[/{s=$0; d=""} $1~/DllName/{d=$2} $1~/disable/{gsub(/[ \t\r]/,"",$2); gsub(/[ \t\r]/,"",d); if (d=="ec_platform.so") print $2}' /opt/wmx3/Module.ini; }
ours() { pgrep -f '[l]ib/wmx_r2_package/|[r]os2_control_node'; }
guard() {
  [ "$(ec_disabled)" = 1 ] || { echo "Module.ini EtherCAT (ec_platform.so) disable ≠ 1 ($(ec_disabled)) — 중단"; exit 1; }
  [ "$(platforms)" = "simu_platform.so " ] || { echo "Module.ini 켜진 플랫폼이 시뮬레이터 하나가 아님: $(platforms) — 중단"; exit 1; }
  if pgrep -af '[r]t_server.py' >/dev/null; then echo 'rt_server 가 돌고 있음 (사용자 서버?) — 중단'; exit 1; fi
  if pgrep -x wmx3_engine >/dev/null; then echo 'WMX 엔진이 떠 있음 — 중단'; exit 1; fi
  if ours >/dev/null; then echo 'R2 노드가 돌고 있음 — 중단'; exit 1; fi
  timeout 10 ros2 topic list 2>/dev/null | grep -q '^/cam_fixed/camera/depth/image_rect_raw$' || { echo '카메라 깊이 토픽 없음 — 중단'; exit 1; }
  timeout 10 ros2 topic list 2>/dev/null | grep -q '^/cam_fixed/camera/color/image_raw$' || { echo '카메라 컬러 토픽 없음 — 중단'; exit 1; }
  echo "안전 확인: EtherCAT disable=1, 켜진 플랫폼 $(platforms), rt_server·엔진·R2 없음, 카메라 토픽 있음"
}
RES=0
for MODE in off on; do
  guard
  OUT=$ROOT/snap_$MODE
  rm -rf "$OUT"
  echo "### 기록 $MODE  $(date +%H:%M:%S)"
  if [ "$WHAT" = dfilt ]; then
    if [ $MODE = off ]; then E="CR3A_DEPTH_TEMPORAL=0 CR3A_DEPTH_BAND=0"; else E="CR3A_DFILT_DEFAULT=1"; fi
    ( cd "$GUI" && env $E python3 -W ignore tests/srv_snap_check.py "$OUT" "$P" "$ROOT/server_$MODE.log" "$RUN_S" ) > "$ROOT/result_$MODE.json" 2> "$ROOT/client_$MODE.log"
  elif [ $MODE = off ]; then
    ( cd "$GUI" && CR3A_SNAP=0 python3 -W ignore tests/srv_snap_check.py "$OUT" "$P" "$ROOT/server_$MODE.log" "$RUN_S" ) > "$ROOT/result_$MODE.json" 2> "$ROOT/client_$MODE.log"
  else
    ( cd "$GUI" && CR3A_SNAP_NEAR_MM=$NEAR_MM python3 -W ignore tests/srv_snap_check.py "$OUT" "$P" "$ROOT/server_$MODE.log" "$RUN_S" ) > "$ROOT/result_$MODE.json" 2> "$ROOT/client_$MODE.log"
  fi
  r=$?
  echo "### $MODE 끝 코드 $r: $(tail -1 "$ROOT/result_$MODE.json")"
  [ $r -ne 0 ] && RES=1
  sleep 3
  echo "끝 상태: 엔진 $(pgrep -x wmx3_engine >/dev/null && echo 켜짐 || echo 꺼짐) · R2 노드 $(ours | wc -l) · 켜진 플랫폼 $(platforms)"
done
exit $RES
