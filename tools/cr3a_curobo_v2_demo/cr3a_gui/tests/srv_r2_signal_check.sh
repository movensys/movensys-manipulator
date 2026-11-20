#!/bin/bash
# [2026-10-27 R-1] R2 를 켜는 중 (wmx_r2.sh up) 신호 시험 — WMX 시뮬레이터만 (컨테이너 wmx_r2 안 admin). 실물 포트 47861 안 씀.
#   srv_r2_signal_check.sh <결과 폴더> [포트 47874]
# 경우마다 서버 (--robot wmx, 가상 카메라) 를 띄우고, 서버 출력에 정한 줄이 나오면 신호를 보낸 뒤 끝나기를 기다려 본다:
#   서버 끝 코드, '켜는 중 … 받음' (up 이 끝난 바로 뒤 처리), 'R2 꺼짐', 그 뒤 우리 R2 노드·WMX 엔진이 없음, Module.ini = simu 하나
#   A  SIGTERM  — R2 launch 를 막 띄운 때 ('1) R2 켜기')
#   B  SIGINT 을 프로세스 묶음 전체에 (터미널 Ctrl+C 와 같음) — 같은 때. wmx_r2.sh 는 따로 세션이라 안 끊겨야 함
#   C  SIGTERM 을 up 처음 ('WMX 시뮬레이터 켜기') 에 + 끄는 중 ('R2 끄기 (서보 OFF') 에 한 번 더 — 끄기 (down) 가 안 끊겨야 함
ROOT=${1:?결과 폴더}
P=${2:-47874}
[ "$P" = 47861 ] && { echo '실물 포트'; exit 2; }
GUI=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$ROOT"
source /opt/wmx_r2_env.sh >/dev/null 2>&1
export PYTHONPATH=/home/admin/workspaces/movensys_ws/curobo_v2_shaping:$PYTHONPATH
platforms() { awk -F= '/^\[/{s=$0} $1~/DllName/{d=$2} $1~/disable/{gsub(/[ \t\r]/,"",$2); if (s~/Platform/ && $2=="0") print d}' /opt/wmx3/Module.ini | tr -d ' \r' | sort -u | tr '\n' ' '; }
ours() { pgrep -f '[l]ib/wmx_r2_package/|[r]os2_control_node'; }
state() { echo "R2 노드: $(ours | wc -l) · 엔진: $(pgrep -x wmx3_engine >/dev/null && echo 켜짐 || echo 꺼짐) · Module.ini 켜진 플랫폼: $(platforms)"; }
guard() {
  if pgrep -af '[r]t_server.py.*--robot (wmx|real)' >/dev/null; then echo 'WMX 를 쓰는 rt_server 가 돌고 있음 — 중단'; exit 1; fi
  if pgrep -x wmx3_engine >/dev/null; then echo 'WMX 엔진이 떠 있음 — 중단'; exit 1; fi
  if ours >/dev/null; then echo 'R2 노드가 돌고 있음 — 중단'; exit 1; fi
  [ "$(platforms)" = "simu_platform.so " ] || { echo "Module.ini 가 시뮬레이터 하나만이 아님: $(platforms) — 중단"; exit 1; }
}
RES=0
run_case() {     # $1 이름 $2 신호 $3 기다릴 줄 $4 묶음 (1 = 프로세스 묶음 전체) $5 (있으면) 두 번째 신호를 보낼 줄
  guard
  local NAME=$1 SIG=$2 PAT=$3 GRP=$4 PAT2=$5 OUT=$ROOT/r2sig_$1 SLOG=$ROOT/r2sig_$1_server.log
  rm -rf "$OUT"
  ( cd "$GUI" && exec setsid python3 -W ignore rt_server.py --robot wmx --realtime off --speed 0.2 --port "$P" --out "$OUT" \
      --camera virtual --scenario moving3t > "$SLOG" 2>&1 ) &
  local SP=$!
  for i in $(seq 1 600); do grep -q "$PAT" "$SLOG" 2>/dev/null && break; kill -0 $SP 2>/dev/null || break; sleep 0.1; done
  local PG; PG=$(ps -o pgid= -p $SP | tr -d ' ')
  if [ "$GRP" = 1 ]; then kill -"$SIG" -- -"$PG"; else kill -"$SIG" $SP; fi
  echo "[$NAME] $(date +%H:%M:%S) '$PAT' 뒤 $SIG $([ "$GRP" = 1 ] && echo '(프로세스 묶음 전체)')"
  if [ -n "$PAT2" ]; then
    for i in $(seq 1 1800); do grep -q "$PAT2" "$SLOG" 2>/dev/null && break; kill -0 $SP 2>/dev/null || break; sleep 0.1; done
    kill -TERM $SP 2>/dev/null && echo "[$NAME] $(date +%H:%M:%S) '$PAT2' 뒤 SIGTERM 한 번 더"
  fi
  for i in $(seq 1 300); do kill -0 $SP 2>/dev/null || break; sleep 1; done
  if kill -0 $SP 2>/dev/null; then echo "[$NAME] ✗ 300 s 안에 안 끝남"; RES=1; return; fi
  wait $SP; local RC=$?
  sleep 2
  local S; S=$(state)
  local OK=1
  grep -q "켜는 중 SIG.* 받음" "$SLOG" || OK=0
  grep -q "R2 꺼짐" "$SLOG" || OK=0
  [ "$(ours | wc -l)" = 0 ] || OK=0
  pgrep -x wmx3_engine >/dev/null && OK=0
  [ "$(platforms)" = "simu_platform.so " ] || OK=0
  echo "[$NAME] $([ $OK = 1 ] && echo 통과 || echo 실패) 끝 코드 $RC · $S"
  grep -E "켜는 중 SIG|R2 끄기|R2 꺼짐|엔진|Module.ini 되돌|서버 프로세스 끝|✗" "$SLOG" | grep -v '^\s*$' | tail -8 | sed "s/^/    /" | cut -c1-200
  [ $OK = 1 ] || RES=1
}
run_case A TERM '1) R2 켜기' 0
run_case B INT '1) R2 켜기' 1
run_case C TERM 'WMX 시뮬레이터 켜기' 0 'R2 끄기 (서보 OFF'
echo "끝: $(state)"
exit $RES
