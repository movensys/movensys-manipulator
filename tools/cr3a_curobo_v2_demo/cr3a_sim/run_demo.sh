#!/bin/bash
# [만듦 2026-10-13] CR3A 1회성 장애물 회피 시뮬레이션 — cuRoboV2 로 계획 → R2 로 실행 → WMX 로그로 확인.  ★ Thor 에서 실행 ★
#   ./run_demo.sh                    (~/workspaces/movensys_ws/cr3a_sim/run_demo.sh) 한 번: ① 시작→A, ② A→B
#   ./run_demo.sh random [N] [시드] [mixed|pillar]  [추가 2026-10-13] A↔B 왕복 N번 (기본 20), 동작마다 상자를 랜덤 위치로 → 매번 1회성 계획
#                                    (plan_random.py). 결과 폴더 이름은 cr3a_rand_<날짜_시각>
#   결과 폴더: ~/workspaces/wmx_r2_logs/cr3a_<날짜_시각>/
#     plan.json        cuRoboV2 계획 (장면, A/B, 직선 보간 검사, 궤적, 상자 거리)       ← plan_avoid.py
#     exec.json        R2 액션 결과와 /joint_states 기록                                  ← plan_avoid.py --execute
#     wmx_axis.csv     WMX 엔진 축 로그 (축 0..5, 1 ms: 명령/실제 위치·속도, inPos, opState) ← wmx_axis_log
#     wmx_axis_api.dat/.txt  WMX API 로그 (R2 가 부른 StartCSplinePos 등)                    ← wmx_axis_log
#     r2_launch.log    R2 노드 로그
#     viewer.html      WMX 로그 값으로 CR3A 를 움직이는 뷰어                              ← make_report.py
#
# 순서
#   0) 안전장치 ①: /opt/wmx3/Module.ini 에서 켜진 플랫폼이 simu 하나뿐인가 (아니면 중단 = 실제 로봇 보호)
#      엔진이 이미 떠 있으면 안전장치 ②: 그 엔진이 실제로 불러온 플랫폼이 simu 뿐인가 (wmx_status_check --require-simu)
#   1) R2 CR3A 실행 (wmx_r2_cr3a_manipulator.launch.py, root): 엔진 붙기/켜기 → 통신 시작 → 6축 알람 해제·서보 ON →
#      CR3A WMX 파라미터 쓰기 (cr3a_wmx_parameters.xml: 단위 rad) → 궤적 액션 서버
#   2) 안전장치 ② 를 한 번 더 (서보가 켜진 엔진이 시뮬레이터인지 기록)
#   3) WMX 축 로그 + API 로그 시작
#   4) 계획 + R2 실행 (plan_avoid.py --execute, admin): ① 현재 자세 → A, ② A → B (상자 회피)
#   5) 로그 끝 → R2 끄기 (서보 OFF, 통신 정지, 엔진 정지) → API 로그 글자로 → 뷰어 만들기
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
DOCK=$HOME/workspaces/wmx_r2_docker
NAME=wmx_r2
CWS=/home/admin/workspaces/movensys_ws
CSIM=$CWS/cr3a_sim
ENVSRC="source /opt/wmx_r2_env.sh"
MODE=${1:-single}
if [ "$MODE" = random ]; then
  NREP=${2:-20}; SEED=${3:-20261013}
  KIND=${4:-mixed}                # mixed = 기둥·벽·판·떠 있는 상자 1~3개, pillar = 첫 판 (바닥 기둥 하나)
  PLAN_CMD="plan_random.py --n $NREP --seed $SEED --kind $KIND"
  LOG_MAX=900                     # WMX 축 로그 최대 초 (멈춤 파일이 생기면 그 전에 멈춤)
  OUT=${OUT:-$HOME/workspaces/wmx_r2_logs/cr3a_rand_$(date +%Y%m%d_%H%M%S)}
else
  PLAN_CMD="plan_avoid.py"
  LOG_MAX=120
  OUT=${OUT:-$HOME/workspaces/wmx_r2_logs/cr3a_$(date +%Y%m%d_%H%M%S)}
fi
PIDF=/tmp/cr3a_demo_launch.pid
CLOG=/tmp/cr3a_demo_launch.log
LAUNCHED=0; LOGGING=0

say() { echo "[$(date +%H:%M:%S)] $*"; }
engine_pid() { ps -C wmx3_engine -o stat=,pid= 2>/dev/null | awk '$1 !~ /^Z/ {print $2; exit}'; }
ini_platforms() {
  awk -F= '
    function flush() { if (sec ~ /^\[Platform/ && dis == 0 && dll != "") print dll }
    /^[ \t]*\[/ { flush(); sec=$0; gsub(/[ \t\r]/, "", sec); dll=""; dis=0; next }
    { k=$1; gsub(/[ \t\r]/, "", k); v=$2; gsub(/[ \t\r]/, "", v)
      if (k == "DllName") dll=v; if (k == "disable") dis=v+0 }
    END { flush() }' /opt/wmx3/Module.ini | sort -u | tr '\n' ' '
}
check_simu_engine() {
  local ep=$1
  docker exec -u root $NAME bash -c "s=\$(ps -o stat= -p $ep 2>/dev/null); \
    if [ -n \"\$s\" ] && [ \"\${s:0:1}\" != Z ]; then /tmp/wmx_status_check --require-simu; else echo '엔진이 꺼짐'; exit 4; fi" \
    | sed 's/^/    /'
  return ${PIPESTATUS[0]}
}
launch_running() { docker exec -u root $NAME bash -c "P=\$(cat $PIDF 2>/dev/null); [ -n \"\$P\" ] && kill -0 \$P 2>/dev/null"; }
stop_launch() {
  docker exec -u root $NAME bash -c "P=\$(cat $PIDF 2>/dev/null); [ -n \"\$P\" ] || exit 0; kill -INT \$P; \
    for i in \$(seq 1 60); do kill -0 \$P 2>/dev/null || break; sleep 0.5; done; \
    kill -0 \$P 2>/dev/null && echo '    ✗ R2 launch 가 30초 안에 안 꺼짐 — 직접 확인 필요' || echo '    R2 꺼짐'; rm -f $PIDF"
}

cleanup() {
  trap - EXIT INT TERM HUP
  if [ $LOGGING = 1 ]; then touch "$OUT/stop"; sleep 1; fi
  if [ $LAUNCHED = 1 ]; then
    say "5) R2 끄기 (서보 OFF → 통신 정지 → 엔진 정지)"
    stop_launch
    docker exec -u root $NAME cat $CLOG > "$OUT/r2_launch.log" 2>/dev/null
    grep -E 'Servo [0-9] off|Communication stopped|Engine stopped' "$OUT/r2_launch.log" | sed 's/^/    /' | tail -9
    EP=$(engine_pid); if [ -n "$EP" ]; then say "   호스트 엔진: 켜져 있음 (PID $EP)"; else say "   호스트 엔진: 꺼짐"; fi
  fi
}
trap cleanup EXIT
trap 'echo; say "중단 요청 받음"; exit 130' INT TERM HUP

mkdir -p "$OUT"
# [추가 2026-10-13] 실행 화면 출력도 결과 폴더에 그대로 남긴다 (안전 확인·동작별 결과의 근거. 검증 때 필요했음)
exec > >(tee -a "$OUT/run_demo.log") 2>&1
say "결과 폴더: $OUT"

# ---- 0) 안전장치 ① ----
P=$(ini_platforms)
say "0) Module.ini 의 켜진 플랫폼: ${P:-없음}"
[ "$P" = "simu_platform.so " ] || { say "   ✗ 시뮬레이터 하나만 켜져 있지 않음 → 중단 (실제 로봇 보호)"; exit 2; }
say "   ✓ 시뮬레이터만 켜져 있음"

"$DOCK/run.sh" create || exit 1
docker cp "$DOCK/tools/wmx_status_check.cpp" $NAME:/tmp/wmx_status_check.cpp
docker exec -u root $NAME g++ -std=c++17 -O1 -I/opt/wmx3/include /tmp/wmx_status_check.cpp \
  -L/opt/wmx3/lib -lwmx3api -limdll -lpthread -lrt -ldl -o /tmp/wmx_status_check || exit 1
docker exec -u admin -w $CSIM $NAME bash -c "[ wmx_axis_log -nt wmx_axis_log.cpp ] || g++ -std=c++17 -O1 -I/opt/wmx3/include \
  wmx_axis_log.cpp -L/opt/wmx3/lib -llogapi -lcoremotionapi -lwmx3api -limdll -lpthread -lrt -ldl -o wmx_axis_log" || exit 1
[ -f "$HERE/cr3a_curobo.yml" ] || { say "   cr3a_curobo.yml 이 없음 → build_model.py 먼저"; exit 1; }

if pgrep -f "[l]ib/wmx_r2_package/" >/dev/null; then
  say "   ✗ 다른 R2 노드가 이미 돌고 있음 (axis_demo KEEP=1 등) → 먼저 끄고 다시"; exit 1
fi
EP=$(engine_pid)
if [ -n "$EP" ]; then
  say "   엔진이 이미 떠 있음 (PID $EP, 부모 $(ps -o comm= -p $(ps -o ppid= -p $EP))) → 붙기 전에 플랫폼 확인"
  check_simu_engine $EP || { say "   ✗ 떠 있는 엔진이 시뮬레이터가 아님 → 중단"; exit 3; }
fi

# ---- 1) R2 CR3A ----
say "1) R2 CR3A 실행 (wmx_r2_cr3a_manipulator.launch.py, root)"
docker exec -u root $NAME rm -f $CLOG $PIDF
docker exec -d -u root -w $CWS $NAME bash -c "$ENVSRC; echo \$\$ > $PIDF; \
  exec ros2 launch wmx_r2_package wmx_r2_cr3a_manipulator.launch.py use_sim_time:=false > $CLOG 2>&1"
LAUNCHED=1
READY=0
for i in $(seq 1 120); do
  docker exec -u root $NAME bash -c "grep -q 'joint_state_broadcaster is ready' $CLOG 2>/dev/null && \
    grep -q 'joint_trajectory_controller is ready' $CLOG && grep -q 'Success to set WMX params' $CLOG" && { READY=1; break; }
  docker exec -u root $NAME bash -c "grep -q -E 'Init failed|Failed to set WMX params|Failed to (create device|start communication)' $CLOG 2>/dev/null" && break
  sleep 0.5
done
if [ $READY != 1 ]; then
  say "   ✗ R2 준비 안 됨. 로그:"; docker exec -u root $NAME tail -20 $CLOG | sed 's/^/    /'; exit 1
fi
say "   ✓ R2 준비됨 (6축 서보 ON, CR3A 파라미터 적용, 궤적 액션 서버)"
docker exec -u root $NAME grep -E "Set axis|Success to set WMX|joint_trajectory_action" $CLOG | sed 's/^/    /' | tail -4

# ---- 2) 안전장치 ② (서보 켜진 엔진) ----
EP=$(engine_pid)
say "2) 서보가 켜진 엔진(PID ${EP:-없음})의 플랫폼 확인"
[ -n "$EP" ] && check_simu_engine $EP || { say "   ✗ 시뮬레이터 확인 실패 → 중단"; exit 3; }

# ---- 3) WMX 로그 ----
say "3) WMX 축 로그 + API 로그 시작"
rm -f "$OUT/stop"
# API 로그는 WMX 기본 위치에 쓴다: 글자로 바꾸는 WMX 도구가 /opt/wmx3/log/apilog/ 안의 파일만 읽음. 끝나면 결과 폴더로 옮김
APIDIR=/opt/wmx3/log/apilog
APINAME=$(basename "$OUT")_api
docker exec -u root $NAME mkdir -p $APIDIR
docker exec -d -u root -w $CSIM $NAME bash -c "./wmx_axis_log log '$OUT' wmx_axis $LOG_MAX '$OUT/stop' '$APIDIR/$APINAME.dat' \
  > '$OUT/wmx_axis_log.out' 2>&1"
LOGGING=1
for i in $(seq 1 20); do grep -q "기록 시작" "$OUT/wmx_axis_log.out" 2>/dev/null && break; sleep 0.25; done
sed 's/^/    /' "$OUT/wmx_axis_log.out"
grep -q "기록 시작" "$OUT/wmx_axis_log.out" || { say "   ✗ WMX 로그를 못 켬"; exit 1; }
sleep 0.5

# ---- 4) 계획 + 실행 ----
say "4) cuRoboV2 계획 + R2 실행 ($PLAN_CMD --execute)"
docker exec -u admin -w $CSIM $NAME bash -c "$ENVSRC; python3 -W ignore $PLAN_CMD --out '$OUT' --execute 2>&1" \
  | grep -v -E "^Warp |^   CUDA|^   \"cuda|it/s|self.setter" | sed 's/^/    /'
RC=${PIPESTATUS[0]}
sleep 1

# ---- 5) 로그 끝 ----
touch "$OUT/stop"
for i in $(seq 1 40); do grep -q "기록 끝" "$OUT/wmx_axis_log.out" 2>/dev/null && break; sleep 0.25; done
LOGGING=0
tail -1 "$OUT/wmx_axis_log.out" | sed 's/^/    /'
# API 로그를 글자로: 엔진이 켜져 있어야 풀린다 ("System is not running") → R2 를 끄기 전에.
#  파일 이름은 WMX 가 뒤에 시각을 붙인다 (wmx_axis_api_<날짜>.dat)
API_DAT=$(ls -t $APIDIR/${APINAME}*.dat 2>/dev/null | head -1)
if [ -n "$API_DAT" ]; then
  docker exec -u root $NAME bash -c "cd /opt/wmx3 && timeout 180 bin/wmx3-apilog-parse -n '$(basename "$API_DAT")' \
    > '$OUT/wmx_apilog_parse.out' 2>&1"
  sed 's/^/    /' "$OUT/wmx_apilog_parse.out" | tail -3
  docker exec -u root $NAME bash -c "mv $APIDIR/${APINAME}* '$OUT'/ 2>/dev/null"
else
  say "   API 로그 파일이 없음"
fi
cleanup; LAUNCHED=0
docker exec -u root $NAME chown -R 1000:1000 "$OUT"
for f in "$OUT"/${APINAME}*.txt; do
  [ -f "$f" ] && say "   API 로그 $(basename "$f"): $(grep -c . "$f") 줄, StartCSplinePos $(grep -c 'StartCSplinePos' "$f") 건"
done
ls -la "$OUT" | sed 's/^/    /'
[ $RC = 0 ] || { say "계획/실행 단계 실패 (rc=$RC)"; exit $RC; }

# ---- 6) 뷰어 ----
if [ -f "$HERE/make_report.py" ]; then
  say "6) 뷰어 만들기"
  docker exec -u admin -w $CSIM $NAME bash -c "$ENVSRC; python3 -W ignore make_report.py '$OUT' 2>&1" \
    | grep -v -E "^Warp |^   CUDA|^   \"cuda" | sed 's/^/    /'
  # [추가 2026-10-13] 보고서가 안 만들어졌으면 분명히 실패로 (랜덤 시험 첫 판에서 오류가 출력에 묻혔음)
  [ -f "$OUT/viewer.html" ] || { say "   ✗ 보고서를 못 만듦 (위 make_report 출력 확인)"; exit 5; }
fi
say "끝: $OUT"
