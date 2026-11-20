#!/bin/bash
# CR3A GUI 를 Thor 화면에 띄움 — ★ Thor 호스트 터미널에서 ★ 실행: ~/workspaces/movensys_ws/cr3a_gui/run_gui.sh
# X 화면 접근 허용 → wmx_r2 컨테이너 안에서 cr3a_gui.py (서버·R2·카메라도 컨테이너 안에서 GUI 가 켬). GUI 출력은 컨테이너 /tmp/cr3a_gui.log
set -u
DISP=${DISPLAY:-:1}
xhost +local:docker >/dev/null 2>&1 || true
if docker exec wmx_r2 pgrep -f "[c]r3a_gui.py" >/dev/null; then
  echo "GUI 가 이미 떠 있음 (컨테이너 wmx_r2) — 그 창을 쓰세요"
  exit 0
fi
# [2026-10-28 D-4] 호스트 셸에 준 시험·되돌리기 스위치를 컨테이너 GUI (→ 서버) 로 넘김 (docker exec 는 환경을 안 물려줌)
#  예: CR3A_DEPTH_BAND=0 ./run_gui.sh  (가림 띠 끔) · CR3A_DEPTH_TEMPORAL=0 · CR3A_SNAP=0 · R2_TRACE=1
ENVS=()
for v in CR3A_DEPTH_BAND CR3A_DEPTH_BAND_ROWS CR3A_DEPTH_TEMPORAL CR3A_SNAP CR3A_SNAP_NEAR_MM R2_TRACE; do
  [ -n "${!v:-}" ] && ENVS+=(-e "$v=${!v}") && echo "넘김: $v=${!v}"
done
docker exec -d -u admin -w /home/admin/workspaces/movensys_ws/cr3a_gui -e DISPLAY="$DISP" "${ENVS[@]}" wmx_r2 bash -c \
  "source /opt/wmx_r2_env.sh && export PYTHONPATH=/home/admin/workspaces/movensys_ws/curobo_v2_shaping:\$PYTHONPATH && python3 cr3a_gui.py > /tmp/cr3a_gui.log 2>&1"
echo "CR3A GUI 띄움 (화면 $DISP). 안 뜨면: docker exec wmx_r2 tail -30 /tmp/cr3a_gui.log"
