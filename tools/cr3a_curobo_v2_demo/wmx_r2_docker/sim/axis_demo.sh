#!/bin/bash
# [만듦 2026-10-13] WMX R2 로 하는 가장 간단한 시뮬레이션: 축 하나를 WMX 시뮬레이터에서 움직여 본다.  ★ Thor 에서 실행 ★
#   MoveIt·로봇 모델·궤적 없음. R2 의 general nodes(엔진·코어모션 노드)와 그 서비스/토픽만 쓴다.
#     ./sim/axis_demo.sh                      축 0 을 +10000 만큼 (상대 이동, 속도 5000/s, 가감속 20000/s²)
#     AXIS=1 DIST=-5000 ./sim/axis_demo.sh    다른 축 / 다른 거리
#     KEEP=1 ./sim/axis_demo.sh               끝나도 R2 노드를 켜 둠 (엔진 유지 → WMX Home 에서도 보임). 다시 실행하면 켜진 노드를 그대로 씀
#     ./sim/axis_demo.sh stop                 켜 둔 노드 끄기 (서보 OFF → 통신 정지 → 엔진 정지)
#   단위: WMX 사용자 단위 (시뮬레이터 기본 설정이면 펄스). 시뮬레이터 축은 simu_network.def 의 Master 0 = 축 0,1,2
#
# 순서 (공식 doc/reference_wmx_r2_general_nodes.md 의 Typical Startup Sequence 에서 파라미터 읽기·호밍만 뺀 것)
#   0) 안전장치 ①: /opt/wmx3/Module.ini 에서 켜진 플랫폼이 simu 하나뿐인가. 아니면 중단 (실제 로봇 보호)
#      + 엔진이 이미 떠 있으면 안전장치 ②(아래)를 먼저 한 번 (Module.ini 를 나중에 바꿨어도 떠 있는 엔진은 켤 때 설정을 씀)
#   1) R2 general nodes 켜기 (wmx_r2_general_nodes.launch.py, root) → wmx_engine_node 가 엔진에 붙고(없으면 켜고) 통신 시작
#   2) /wmx/engine/ready 가 true 가 될 때까지 기다림
#   3) 안전장치 ②: 떠 있는 엔진이 실제로 불러온 플랫폼이 simu 뿐인가 (tools/wmx_status_check --require-simu, 읽기만)
#   4) /wmx/axis/clear_alarm → /wmx/axis/set_on (서보 ON)
#   5) /wmx/axis/position/relative 로 이동 명령 + /wmx/axis/state 로 위치 변화 보기 (sim/watch_axis.py)
#   6) /wmx/axis/set_on 0 (서보 OFF) → general nodes 끄기
#      ⚠️ 끌 때 wmx_engine_node 가 통신 정지 + 엔진 정지(StopEngine)를 한다. WMX Home 이 켠 엔진이어도 꺼진다.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
DOCK="$(dirname "$HERE")"
NAME=wmx_r2
WS=/home/admin/workspaces/movensys_ws
ENVSRC="source /opt/wmx_r2_env.sh"
AXIS=${AXIS:-0}; DIST=${DIST:-10000}; VEL=${VEL:-5000}; ACC=${ACC:-20000}; KEEP=${KEEP:-0}
PIDF=/tmp/axis_demo_general_nodes.pid
LOG=/tmp/axis_demo_general_nodes.log
LAUNCHED=0; SERVO=0; DONE=0

say() { echo "[$(date +%H:%M:%S)] $*"; }
cx() { docker exec -u root -w $WS $NAME bash -c "$ENVSRC; $*"; }   # 컨테이너 안에서 root 로
svc() {  # R2 서비스 부르기: svc <이름> <타입> <요청>
  cx "timeout 30 ros2 service call $1 $2 \"$3\"" 2>&1 | grep -E "response|success|message|Error|error" | tail -2 | sed 's/^/    /'
}
engine_pid() { ps -C wmx3_engine -o stat=,pid= 2>/dev/null | awk '$1 !~ /^Z/ {print $2; exit}'; }
ini_platforms() {  # Module.ini 에서 켜진(disable=0 또는 없음) [Platform N] 의 DllName
  awk -F= '
    function flush() { if (sec ~ /^\[Platform/ && dis == 0 && dll != "") print dll }
    /^[ \t]*\[/ { flush(); sec=$0; gsub(/[ \t\r]/, "", sec); dll=""; dis=0; next }
    { k=$1; gsub(/[ \t\r]/, "", k); v=$2; gsub(/[ \t\r]/, "", v)
      if (k == "DllName") dll=v; if (k == "disable") dis=v+0 }
    END { flush() }' /opt/wmx3/Module.ini | sort -u | tr '\n' ' '
}
check_simu_engine() {  # 안전장치 ②. 떠 있는 엔진에 읽기만 하고 붙어서 플랫폼 확인
  local ep=$1
  docker exec -u root $NAME bash -c "s=\$(ps -o stat= -p $ep 2>/dev/null); \
    if [ -n \"\$s\" ] && [ \"\${s:0:1}\" != Z ]; then /tmp/wmx_status_check --require-simu; else echo '엔진이 꺼짐'; exit 4; fi" \
    | sed 's/^/    /'
  return ${PIPESTATUS[0]}
}

nodes_running() { docker exec -u root $NAME bash -c "P=\$(cat $PIDF 2>/dev/null); [ -n \"\$P\" ] && kill -0 \$P 2>/dev/null"; }

cleanup() {
  trap - EXIT INT TERM HUP
  if [ $KEEP = 1 ] && [ $DONE = 1 ]; then
    say "KEEP=1: R2 노드와 엔진을 켜 둡니다 (축 $AXIS 서보 ON). 끄기: ./sim/axis_demo.sh stop"
    return
  fi
  if [ $SERVO = 1 ]; then
    say "6) 서보 OFF (축 $AXIS)"; svc /wmx/axis/set_on wmx_r2_message/srv/SetAxis "{index: [$AXIS], data: [0]}"
  fi
  if [ $LAUNCHED = 1 ]; then
    # launch 는 root 프로세스라 끄는 것도 root 로 (처음 판은 admin 으로 보내서 거부됨 → 노드가 남았음)
    say "6) general nodes 끄기 (통신 정지 + 엔진 정지)"
    docker exec -u root $NAME bash -c "P=\$(cat $PIDF 2>/dev/null); [ -n \"\$P\" ] || exit 0; kill -INT \$P; \
      for i in \$(seq 1 40); do kill -0 \$P 2>/dev/null || break; sleep 0.5; done; \
      kill -0 \$P 2>/dev/null && echo '    ✗ launch 가 20초 안에 안 꺼짐 — 직접 확인 필요' || echo '    launch 꺼짐'; rm -f $PIDF"
    docker exec -u root $NAME bash -c "grep -E 'Servo|Communication stopped|Engine stopped|ERROR' $LOG | sed 's/^/    /' | tail -8"
    EP=$(engine_pid)
    if [ -n "$EP" ]; then say "   호스트 엔진: 켜져 있음 (PID $EP)"; else say "   호스트 엔진: 꺼짐"; fi
  fi
}
trap cleanup EXIT
trap 'echo; say "중단 요청 받음"; DONE=0; exit 130' INT TERM HUP

if [ "${1:-}" = stop ]; then   # 켜 둔 노드 끄기
  if nodes_running; then
    LAUNCHED=1; SERVO=1; AXIS=${AXIS:-0}; KEEP=0
    say "켜 둔 R2 노드를 끕니다"; exit 0     # → cleanup 이 서보 OFF + 노드 끄기
  fi
  say "켜 둔 R2 노드 없음"; LAUNCHED=0; exit 0
fi

# ---- 0) 안전장치 ① ----
P=$(ini_platforms)
say "0) Module.ini 의 켜진 플랫폼: ${P:-없음}"
if [ "$P" != "simu_platform.so " ]; then
  say "   ✗ 시뮬레이터 하나만 켜져 있지 않음 → 중단 (실제 로봇에 명령이 갈 수 있음). WMX Home 에서 Simulation 으로 바꾼 뒤 다시."
  exit 2
fi
say "   ✓ 시뮬레이터만 켜져 있음"

"$DOCK/run.sh" create || exit 1
docker cp "$HERE/watch_axis.py" $NAME:/tmp/watch_axis.py
docker cp "$DOCK/tools/wmx_status_check.cpp" $NAME:/tmp/wmx_status_check.cpp
docker exec -u root $NAME g++ -std=c++17 -O1 -I/opt/wmx3/include /tmp/wmx_status_check.cpp \
  -L/opt/wmx3/lib -lwmx3api -limdll -lpthread -lrt -ldl -o /tmp/wmx_status_check || exit 1

EP=$(engine_pid)
if [ -n "$EP" ]; then
  say "   엔진이 이미 떠 있음 (PID $EP, 부모 $(ps -o comm= -p $(ps -o ppid= -p $EP))) → 붙기 전에 플랫폼 확인"
  check_simu_engine $EP || { say "   ✗ 떠 있는 엔진이 시뮬레이터가 아님 → 중단"; exit 3; }
fi

# ---- 1) R2 general nodes ----
if nodes_running; then
  say "1) R2 general nodes 가 이미 켜져 있음 (KEEP=1 로 켜 둔 것) → 그대로 씀"
else
  say "1) R2 general nodes 켜기 (wmx_r2_general_nodes.launch.py, root, 로그 컨테이너 $LOG)"
  docker exec -u root $NAME rm -f $LOG $PIDF   # 지난 실행 로그가 남아 있으면 2) 가 준비됐다고 잘못 볼 수 있음
  docker exec -d -u root -w $WS $NAME bash -c "$ENVSRC; echo \$\$ > $PIDF; \
    exec ros2 launch wmx_r2_package wmx_r2_general_nodes.launch.py > $LOG 2>&1"
fi
LAUNCHED=1

# ---- 2) 준비 기다리기 ----
# 노드 로그로 본다: 엔진 노드 "Communication started" + 코어모션 노드 "is ready" (토픽 echo 로 보던 첫 판은 감지 실패)
say "2) 엔진 통신 시작 + 코어모션 노드 준비 기다리는 중 (최대 60초)"
READY=0
for i in $(seq 1 120); do
  docker exec -u root $NAME bash -c "grep -q 'Communication started' $LOG 2>/dev/null && grep -q 'wmx_core_motion_node is ready' $LOG" \
    && { READY=1; break; }
  docker exec -u root $NAME bash -c "grep -q -E 'Failed to (create device|start communication)' $LOG 2>/dev/null" && break
  sleep 0.5
done
if [ $READY != 1 ]; then
  say "   ✗ 준비 안 됨. 노드 로그:"; docker exec -u root $NAME tail -15 $LOG | sed 's/^/    /'; exit 1
fi
say "   ✓ 준비됨. R2 서비스로 본 엔진 상태: $(cx "timeout 20 ros2 service call /wmx/engine/get_status std_srvs/srv/Trigger" | grep -o "message='[^']*'")"

# ---- 3) 안전장치 ② ----
EP=$(engine_pid)
say "3) 떠 있는 엔진(PID ${EP:-없음})이 불러온 플랫폼 확인"
[ -n "$EP" ] && check_simu_engine $EP || { say "   ✗ 시뮬레이터 확인 실패 → 서보를 켜지 않고 중단"; exit 3; }

# ---- 4) 알람 해제 → 서보 ON ----
say "4) 축 $AXIS 알람 해제 → 서보 ON"
svc /wmx/axis/clear_alarm wmx_r2_message/srv/SetAxis "{index: [$AXIS], data: [0]}"
SERVO=1
svc /wmx/axis/set_on wmx_r2_message/srv/SetAxis "{index: [$AXIS], data: [1]}"

# ---- 5) 이동 + 관찰 ----
T=$(awk -v d=$DIST -v v=$VEL -v a=$ACC 'BEGIN { d = d < 0 ? -d : d; print int(d / v + v / a + 3) }')
say "5) 축 $AXIS 를 $DIST 만큼 이동 (속도 $VEL, 가감속 $ACC) — /wmx/axis/state 를 ${T}초 관찰"
docker exec -u root $NAME bash -c "$ENVSRC; python3 -u /tmp/watch_axis.py $AXIS $T" &
WPID=$!
sleep 1.5
cx "timeout 20 ros2 topic pub --once -w 1 /wmx/axis/position/relative wmx_r2_message/msg/AxisPose \
  \"{index: [$AXIS], target: [$DIST], velocity: [$VEL], acc: [$ACC], dec: [$ACC]}\" > /dev/null" \
  && say "   이동 명령 보냄 (/wmx/axis/position/relative)" || say "   ✗ 이동 명령 못 보냄"
wait $WPID
DONE=1
say "끝"
