#!/bin/bash
# CR3A 실시간 장애물 회피 + 입력성형 — WMX 시뮬레이터에서 끝까지 + 보고서.  ★ Thor 호스트에서 실행 ★
#   ~/workspaces/movensys_ws/cr3a_rt_shaping/tools/rt_demo.sh   ([2026-10-14] 입력성형 작업 사본, [2026-10-27 R-1] tools/ 로 옮김)
#   tools/rt_demo.sh                                            성형 설정 = shaping.yaml, 명령 방식 = rt_config (Config.style, 기본 당근 MPC)
#   SHAPER=ZV SHAPER_HZ="8 15" SHAPER_ZETA=0.02 tools/rt_demo.sh   성형기 바꾸기 (shaping.yaml 보다 우선, SHAPER=none = 성형 없음)
#   STYLE=cmpc tools/rt_demo.sh                                 성형 MPC (C 방식, rt_cmpc — 다음 단계)
#   SCENARIO=goal_wait RT_ARGS="--legs 4" tools/rt_demo.sh      시나리오 (기본: 공구 모델 moving3t, MODEL=bare 면 moving5) · rt_run.py 인자 더하기
#   MODEL=bare tools/rt_demo.sh                                 로봇 형상 (rt_model): 기본 tool = 끝단 브래킷·D455·그리퍼 붙음 (2026-10-16), bare = 그 전
#   R2_TRACE=1 tools/rt_demo.sh                                 R2 디버그 기록 (r2_trace.txt) 켬 — 기본 끔 (R-1 안전 ⑭)
#   R2_WS=…/pvat_r2_ws R2_SAMPLES=1 R2_TIMING=1 RT_ARGS="--pvat-record" tools/rt_demo.sh
#                                                               [2026-10-30 PVAT 단계 1] 계측 R2 (버퍼에 넣은 1 ms 점·주기별 시간) + 보낸 메시지 전체
#   cuRobo 는 사본 ~/workspaces/movensys_ws/curobo_v2_shaping (가지 shaping: 성형 훅·기준 궤적 비용) 을 PYTHONPATH 로 씀
#   [2026-10-27 R-1] 시뮬레이터 안전장치·R2 켜기/끄기·WMX 축 로그 = cr3a_gui/wmx_r2.sh up / down (GUI 와 같은 한 벌 — 예전엔 이 파일에 사본,
#     사용자 2026-10-16 "절대로 이더켓으로 실행하면 안된다. 시뮬레이션으로 실행해야해."). R2 는 devel_cyclic_ws 하나 (10-15 까지의 R2=cyclic 은 뺌).
#     기본 이름·시나리오 고침: 방식 cmpc → carrot (rt_config 기본), 시나리오 moving5 → moving3t (moving5 는 공구 모델의 A·B 자리를 막음).
#   순서
#     0) R2 노드가 하나라도 돌고 있으면 (GUI 가 켠 R2 포함) 시작 안 함 — wmx_r2.sh up 은 남은 R2 를 끄고 다시 켜므로 여기서 먼저 막음
#     1) wmx_r2.sh up: Module.ini 의 켜진 플랫폼이 simu 하나뿐인지 (EtherCAT 이면 엔진이 꺼져 있을 때만 simu 로 바꾸고 down 에서 되돌림,
#        WMX_AUTO_MODE=0 이면 바꾸지 않고 멈춤) · 떠 있는 엔진이 시뮬레이터인지 · R2 켜기 직전에 한 번 더 · R2 (CyclicBuffer 곡선 모드
#        cyclic_subsample:=hermite + 속도 적분 CR3A_R2_VELINT=1, cyclic_max_acc:=15 — 사용자 결정 2026-10-13) · 서보 켠 엔진이 시뮬레이터인지 ·
#        WMX 엔진 축 로그 (1 ms, 명령·실제 위치 — 보고서의 조인트 값)
#     2) rt_run.py --robot ros: 합성 깊이 (카메라 1 대, 15 fps) → Mapper → MPC 20 Hz → 명령 궤적 → JointTrajectory → JTC → WMX
#     3) wmx_r2.sh down: 서보 OFF → 축 로그 끝 → R2 끄기 → Module.ini 되돌리기 (켤 때 바꿨으면)
#     4) python3 -m tools.rt_report (보고서 HTML + summary.json)
#   결과: ~/workspaces/wmx_r2_logs/is_<방식>_<성형>_<날짜_시각>/  (rt_run.json, u_exec.npz, wmx_axis.csv, r2_launch.log, run.log, report.html,
#         summary.json, R2_TRACE=1 이면 r2_trace.txt)
set -u
DOCK=$HOME/workspaces/wmx_r2_docker
NAME=wmx_r2
CWS=/home/admin/workspaces/movensys_ws
CRT=${CRT:-$CWS/cr3a_rt_shaping}            # 컨테이너 안 경로. CRT·R2SH 는 시험 사본으로 바꿀 수 있음 (예 ~/workspaces/wmx_r2_logs/refactor/…
R2SH=${R2SH:-$CWS/cr3a_gui/wmx_r2.sh}       #  — 그 폴더는 컨테이너에서도 같은 경로). wmx_r2.sh 는 admin 으로 (sudo -n 됨)
SH_ARGS=""
[ -n "${SHAPER:-}" ] && SH_ARGS="$SH_ARGS --shaper $SHAPER"
[ -n "${SHAPER_HZ:-}" ] && SH_ARGS="$SH_ARGS --shaper-hz $SHAPER_HZ"
[ -n "${SHAPER_ZETA:-}" ] && SH_ARGS="$SH_ARGS --shaper-zeta $SHAPER_ZETA"
[ -n "${SHAPING_FILE:-}" ] && SH_ARGS="$SH_ARGS --shaping-file $SHAPING_FILE"
[ -n "${STYLE:-}" ] && SH_ARGS="$SH_ARGS --style $STYLE"
[ -n "${MODEL:-}" ] && SH_ARGS="$SH_ARGS --model $MODEL"
case ${MODEL:-tool} in bare) SCEN=${SCENARIO:-moving5} ;; *) SCEN=${SCENARIO:-moving3t} ;; esac
TAG=$(echo "${STYLE:-carrot}_${SHAPER:-cfg}${SHAPER_HZ:+_$SHAPER_HZ}" | tr ' ' '+')
OUT=${OUT:-$HOME/workspaces/wmx_r2_logs/is_${TAG}_$(date +%Y%m%d_%H%M%S)}
RT_ARGS=${RT_ARGS:-""}             # rt_run.py 인자 (설정값은 rt_config.py). --scenario 를 주면 SCENARIO 보다 우선 (뒤에 붙음)
ENVSRC="source /opt/wmx_r2_env.sh && source $CWS/devel_cyclic_ws/install/setup.bash && export PYTHONPATH=$CWS/curobo_v2_shaping:\$PYTHONPATH"
UP=0

say() { echo "[$(date +%H:%M:%S)] $*"; }
r2sh() {     # cr3a_gui/wmx_r2.sh <명령> … 를 컨테이너 안 admin 으로. 설정 환경 변수는 있으면 그대로 넘김
  local e=() v
  for v in R2_ARGS R2_VELINT R2_TRACE LOG_MAX_S WMX_AUTO_MODE R2_WS R2_SAMPLES R2_TIMING; do [ -n "${!v:-}" ] && e+=(-e "$v=${!v}"); done
  docker exec -u admin ${e[@]+"${e[@]}"} $NAME bash "$R2SH" "$@" 2>&1 | sed -u 's/^/    /'
  return "${PIPESTATUS[0]}"
}
cleanup() {
  trap - EXIT INT TERM HUP
  if [ $UP = 1 ]; then
    say "R2 끄기 (wmx_r2.sh down: 서보 OFF → 축 로그 끝 → R2 → 엔진 → Module.ini 되돌리기)"
    r2sh down "$OUT"
    UP=0
  fi
}
trap cleanup EXIT
trap 'echo; say "중단 요청 받음"; exit 130' INT TERM HUP

mkdir -p "$OUT"
exec > >(tee -a "$OUT/run.log") 2>&1
say "결과 폴더: $OUT  (시나리오 $SCEN, 모델 ${MODEL:-tool}, R2 = devel_cyclic_ws, 안전장치·R2 = $R2SH)"
"$DOCK/run.sh" create || exit 1
if pgrep -f "[l]ib/wmx_r2_package/|[r]os2_control_node" >/dev/null; then
  say "0) ✗ R2 노드가 돌고 있음 (GUI 가 켠 R2 포함) → 시작 안 함"; pgrep -af "[l]ib/wmx_r2_package/|[r]os2_control_node" | head -3 | sed 's/^/    /'
  exit 1
fi
say "1) WMX 시뮬레이터 안전장치 → R2 → 축 로그 (wmx_r2.sh up)"
UP=1                               # (실패·중단이어도 down 으로 정리 — down 은 남은 것이 없으면 아무것도 안 함)
r2sh up "$OUT" || { say "   ✗ wmx_r2.sh up 실패 → 중단"; exit 1; }

say "2) 실시간 회피 + 입력성형 (rt_run.py --robot ros --scenario $SCEN $SH_ARGS $RT_ARGS)"
docker exec -u admin -w $CRT $NAME bash -c "$ENVSRC && python3 -W ignore rt_run.py --robot ros --out '$OUT' --scenario $SCEN $SH_ARGS $RT_ARGS 2>&1" \
  | grep --line-buffered -v -E "^Warp |^   |Module .* load|it/s|max_autotune" | sed -u 's/^/    /'
RC=${PIPESTATUS[0]}
sleep 1
say "3) R2 끄기"
cleanup
grep -E "\[(WARN|ERROR)\].*(Cyclic|MaxAcc|stalled|\bfault)" "$OUT/r2_launch.log" 2>/dev/null | cut -c1-220 | sed 's/^/    /' | head
say "4) 보고서 (python3 -m tools.rt_report)"
docker exec -u admin -w $CRT $NAME bash -c "$ENVSRC && python3 -W ignore -m tools.rt_report '$OUT' 2>&1" | grep -v -E "^Warp |^   " | sed 's/^/    /'
[ $RC = 0 ] || { say "✗ 실시간 회피 단계 실패 (rc=$RC)"; exit 1; }
say "끝: $OUT"
