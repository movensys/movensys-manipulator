#!/bin/bash
# CR3A GUI — WMX R2 켜기·끄기·엔코더 읽기.  ★ wmx_r2 컨테이너 안 (admin, sudo -n 됨) ★  rt_server.py --robot wmx 가 부름 (손으로도 됨)
#   wmx_r2.sh up <결과 폴더>        [WMX 시뮬레이터] 안전장치 → R2 (devel_cyclic_ws, CyclicBuffer 스트리밍) → 시뮬레이터 확인 → WMX 축 로그
#   wmx_r2.sh up-real <결과 폴더>   [EtherCAT, ★ 실제 CR3A 가 움직임 ★] Module.ini 를 EtherCAT 하나로 (시뮬레이터였으면 바꾸고 down 에서 되돌림)
#                                  → R2 (up 과 같음, 켜지면 서보 ON) → 엔진 플랫폼 기록 → WMX 축 로그. rt_server.py --robot real 만 부름
#                                  (GUI 대상 '실제 로봇' + 확인 창). [2026-10-26 사용자 "자 그럼 실물 로봇으로 움직여보자", "내가 Thor 화면에서 직접"]
#   wmx_r2.sh down [결과 폴더]      축 로그 끝 → R2 끄기 (버퍼 중단 → 감속 정지 → 서보 OFF → 엔진 정지) → Module.ini 되돌리기 (켤 때 바꿨으면)
#   wmx_r2.sh encoders <결과 폴더>  [EtherCAT, 실제 로봇] R2 일반 노드로 통신만 켜서 6 축 엔코더 위치를 읽고 끔. 서보는 안 켬 (안 움직임).
#                                  시뮬레이터 설정이었으면 끝나고 다시 시뮬레이터로 (사용자 2026-10-26 "끝났으면 WMX를 다시 시뮬레이션으로")
#   wmx_r2.sh status
# up 의 절차·안전장치는 cr3a_rt_shaping/rt_demo.sh 와 같음 (사용자 2026-10-16 "절대로 이더켓으로 실행하면 안된다. 시뮬레이션으로 실행해야해."):
#   0) Module.ini 의 켜진 플랫폼이 simu_platform.so 하나뿐 (EtherCAT 이면, 엔진이 꺼져 있을 때만 wmx/wmx_mode.sh simu 로 바꿈)
#   1) 떠 있는 엔진이 있으면 시뮬레이터인지 (wmx_status_check --require-simu)   2) R2 켜기 직전에 0)·1) 다시
#   3) R2 가 서보를 켠 뒤 엔진에 다시 물음 — 시뮬레이터가 아니면 바로 R2 끔
# encoders 는 [2026-10-26 사용자 "WMX R2로 커뮤니케이션 해도 서보를 키지 않으면 움직이지 않으니까 통신까진 해서 엔코더를 읽어와봐"]:
#   wmx/wmx_read_encoders.launch.py = 엔진 + 축 상태 노드만 (서보를 켜는 joint_state_broadcaster·ros2_control·그리퍼·IO 없음),
#   wmx_encoders.py 가 서보가 켜진 축을 보면 바로 멈춤.
# wmx/wmx_mode.sh·wmx/wmx_status_check.cpp = ~/workspaces/wmx_r2_docker 의 것과 같은 사본 (그 폴더는 컨테이너에 안 붙어 있음)
# wmx/wmx_axis_log.cpp = WMX 축 로그 (1 ms) — [2026-10-27 R-1] cr3a_sim/wmx_axis_log.cpp 에서 그대로 옮김 (cr3a_sim 은 보관). 처음 쓸 때
#   (또는 .cpp 가 더 새것이면) wmx/wmx_axis_log 로 빌드 (빌드 결과는 git 에 안 넣음)
# 환경 변수: R2_ARGS (R2 launch 인자, 기본 아래), R2_VELINT (기본 1), LOG_MAX_S (축 로그 최대 초),
#   R2_TRACE=1 (R2 가 명령을 받을 때마다 한 줄 → <폴더>/r2_trace.txt, 디버그용. [2026-10-27 R-1 안전 ⑭] 기본 끔 — 실물에서도 늘 쓰던 것),
#   WMX_AUTO_MODE=0 (up: Module.ini 가 EtherCAT 이면 simu 로 바꾸지 않고 멈춤 — tools/rt_demo.sh 가 넘김)
#   [2026-10-30 PVAT 단계 1] R2_WS=<작업 공간> (기본 devel_cyclic_ws — 계측 판은 pvat_r2_ws 로 따로 빌드해서 실물용은 그대로),
#   R2_SAMPLES=1 (버퍼에 넣은 1 ms 점 → <폴더>/r2_samples.txt), R2_TIMING=1 (주기마다 R2 걸린 시간 → <폴더>/r2_timing.txt) — 둘 다
#   pvat_r2_ws 의 R2 에만 있는 기록 (기본 끔)
set -u
CMD=${1:-status}
OUT=${2:-}
HERE=$(cd "$(dirname "$0")" && pwd)
CWS=/home/admin/workspaces/movensys_ws
R2_WS=${R2_WS:-$CWS/devel_cyclic_ws}
AXLOG=$HERE/wmx                          # wmx_axis_log.cpp → wmx_axis_log (빌드)
ARM=movensys_manipulator_arm_controller
PIDF=/tmp/cr3a_gui_r2.pid               # 우리가 띄운 R2 launch (root)
SWITCHED=/tmp/cr3a_gui_wmx_switched     # 켤 때 Module.ini 를 simu 로 바꿨음 → 끌 때 EtherCAT 으로
SWITCHED_EC=/tmp/cr3a_gui_wmx_to_ec     # (up-real) 켤 때 Module.ini 를 EtherCAT 으로 바꿨음 → 끌 때 다시 simu 로 (평소 = 시뮬레이터)
CHK=/tmp/cr3a_wmx_status_check
R2_ARGS=${R2_ARGS:-"cyclic_subsample:=hermite cyclic_max_acc:=15"}    # 15 = 사용자 결정 2026-10-13 (rt_demo.sh 와 같음)
LOG_MAX_S=${LOG_MAX_S:-2400}            # WMX 축 로그 최대 길이 (1 ms 줄, ~0.4 MB/s → 40 분 ~0.9 GB). [2026-10-27] 20 → 40 분 (rt_server REC_MAX_S 와 같게)
ENVSRC="source /opt/wmx_r2_env.sh && source $R2_WS/install/setup.bash"
# sudo 는 환경을 지우므로 ROS 설정을 직접 넘김 (컨테이너 기본: 도메인 70, 이 PC 안만, CycloneDDS)
ROSENV="ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-70} ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp} HOME=/root"

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
build_check() {
  [ -x $CHK ] && [ $CHK -nt "$HERE/wmx/wmx_status_check.cpp" ] && return 0
  g++ -std=c++17 -O1 -I/opt/wmx3/include "$HERE/wmx/wmx_status_check.cpp" -L/opt/wmx3/lib -lwmx3api -limdll -lpthread -lrt -ldl -o $CHK
}
check_simu_engine() {      # $1 = 엔진 PID. 0 = 시뮬레이터만 불러옴
  local s; s=$(ps -o stat= -p "$1" 2>/dev/null)
  if [ -z "$s" ] || [ "${s:0:1}" = Z ]; then echo "    엔진이 꺼짐"; return 4; fi
  sudo -n $CHK --require-simu 2>&1 | sed 's/^/    /'
  return ${PIPESTATUS[0]}
}
NODE_PAT='wmx_r2_package/|ros2_control_node|ros2 launch'     # 우리 R2 세션 안의 launch·노드 (엔진 wmx3_engine 은 엔진 노드가 끔)
r2_sid() { cat $PIDF 2>/dev/null; }    # launch PID = 세션 번호 (setsid -f)
r2_pid() { local p; p=$(r2_sid); [ -n "$p" ] && sudo -n pgrep -s "$p" -f "$NODE_PAT" >/dev/null && echo "$p"; }
other_r2() { pgrep -f "[l]ib/wmx_r2_package/|[r]os2_control_node"; }
stop_r2() {
  local p; p=$(r2_pid)
  if [ -z "$p" ]; then sudo -n rm -f $PIDF; return 0; fi
  say "R2 끄기 (세션 $p: 제어기·버퍼 닫기 → 서보 OFF → 통신·엔진 정지)"
  sudo -n kill -INT "$p" 2>/dev/null
  for i in $(seq 1 60); do sudo -n pgrep -s "$p" -f "$NODE_PAT" >/dev/null || break; sleep 0.5; done
  # [2026-10-26 엔코더 읽기] launch 가 SIGINT 를 무시해 안 꺼진 적 있음 → 같은 세션의 노드에 직접 SIGINT (노드는 rclcpp 가 받아서 정리)
  if sudo -n pgrep -s "$p" -f "$NODE_PAT" >/dev/null; then
    say "   launch 가 30 초 안에 안 꺼짐 → 노드에 직접 SIGINT"
    sudo -n pkill -INT -s "$p" -f 'wmx_r2_package/|ros2_control_node'
    for i in $(seq 1 60); do sudo -n pgrep -s "$p" -f 'wmx_r2_package/|ros2_control_node' >/dev/null || break; sleep 0.5; done
    sudo -n pkill -TERM -s "$p" -f 'ros2 launch'
    sleep 1
  fi
  if sudo -n pgrep -s "$p" -f "$NODE_PAT" >/dev/null; then say "   ✗ R2 노드가 안 꺼짐 — 직접 확인 필요"; return 1; fi
  sudo -n rm -f $PIDF
  for i in $(seq 1 20); do [ -z "$(engine_pid)" ] && break; sleep 0.5; done
  local e; e=$(engine_pid)
  say "   R2 꺼짐 · 엔진 ${e:+아직 켜짐 (PID $e)}${e:-꺼짐}"
}
restore() {                # 켤 때 simu 로 바꿨으면 EtherCAT 으로 (엔진이 꺼진 뒤에만)
  [ -f $SWITCHED ] || return 0
  local e; e=$(engine_pid)
  if [ -n "$e" ]; then say "엔진이 아직 떠 있음 (PID $e) → Module.ini 는 그대로 (엔진이 꺼진 뒤 down 을 다시)"; return 1; fi
  say "Module.ini 되돌리기 (EtherCAT)"
  "$HERE/wmx/wmx_mode.sh" ethercat | sed 's/^/    /'
  [ "${PIPESTATUS[0]}" = 0 ] && rm -f $SWITCHED
}
launch_root() {            # $1 = 로그 파일, 나머지 = ENVSRC 뒤에 이어 실행할 명령 (root, 새 세션 = launch PID, PID 를 PIDF 에)
  local log=$1; shift
  sudo -n rm -f $PIDF
  # [2026-10-26] '... &' 로 띄우면 비대화 셸이 그 프로세스에 SIGINT 무시를 물려줘서 ros2 launch 가 SIGINT 로 안 꺼졌음 (엔코더 읽기 때
  #  EtherCAT 엔진이 남음) → 앞에서 setsid -f (스스로 갈라지고 바로 돌아옴) 로 띄움
  sudo -n env $ROSENV setsid -f bash -c "$ENVSRC; echo \$\$ > $PIDF; $*" > "$log" 2>&1 < /dev/null
  for i in $(seq 1 20); do [ -s $PIDF ] && break; sleep 0.25; done
}

up() {
  [ -n "$OUT" ] || { echo "결과 폴더가 없음"; exit 2; }
  mkdir -p "$OUT"
  say "WMX 시뮬레이터 켜기 → $OUT"
  if [ -n "$(r2_pid)" ]; then say "지난번 R2 가 남아 있음 → 끄고 다시 켬"; stop_r2 || exit 1; fi
  if other_r2 >/dev/null; then say "✗ 다른 R2 노드가 돌고 있음 → 안 켬"; other_r2 | head -3 | sed 's/^/    /'; exit 1; fi
  local P EP
  P=$(ini_platforms)
  if [ "$P" = "ec_platform.so " ] && [ "${WMX_AUTO_MODE:-1}" = 1 ]; then
    "$HERE/wmx/wmx_mode.sh" simu | sed 's/^/    /'
    case ${PIPESTATUS[0]} in 0) touch $SWITCHED ;; 1) ;; *) say "✗ simu 로 못 바꿈 → 중단"; exit 2 ;; esac
    P=$(ini_platforms)
  fi
  say "0) Module.ini 의 켜진 플랫폼: ${P:-없음}"
  [ "$P" = "simu_platform.so " ] || { say "✗ 시뮬레이터 하나만 켜져 있지 않음 → 중단 (실제 로봇 보호)"; restore; exit 2; }
  build_check || { restore; exit 1; }
  build_axis_log || { restore; exit 1; }
  EP=$(engine_pid)
  if [ -n "$EP" ]; then check_simu_engine $EP || { say "✗ 떠 있는 엔진이 시뮬레이터가 아님 → 중단"; exit 3; }; fi
  # R2 (엔진·서보 켬) 직전에 한 번 더 (rt_demo.sh 10-16 과 같음)
  P=$(ini_platforms)
  [ "$P" = "simu_platform.so " ] || { say "✗ (R2 켜기 직전) Module.ini 가 시뮬레이터 하나만이 아님: ${P:-없음} → 중단"; restore; exit 2; }
  EP=$(engine_pid)
  if [ -n "$EP" ]; then check_simu_engine $EP || { say "✗ (R2 켜기 직전) 떠 있는 엔진이 시뮬레이터가 아님 → 중단"; exit 3; }; fi
  start_r2 "$P" || { restore; exit 1; }
  EP=$(engine_pid)
  say "   서보 켠 엔진 (PID ${EP:-없음}) 플랫폼 확인"
  if [ -z "$EP" ] || ! check_simu_engine $EP; then say "✗ 시뮬레이터 확인 실패 → R2 끔"; stop_r2; restore; exit 3; fi
  start_axis_log || { restore; exit 1; }
  say "준비 끝: R2 PID $(r2_pid) · 엔진 PID $EP = 시뮬레이터 · 서보 켜짐"
}

build_axis_log() {
  ( cd "$AXLOG" && { [ wmx_axis_log -nt wmx_axis_log.cpp ] || g++ -std=c++17 -O1 -I/opt/wmx3/include wmx_axis_log.cpp -L/opt/wmx3/lib \
      -llogapi -lcoremotionapi -lwmx3api -limdll -lpthread -lrt -ldl -o wmx_axis_log; } )
}

start_r2() {     # R2 켜기 → 팔 제어기까지 켜질 때까지 기다림 (켜지면 서보 ON). 실패면 R2 끄고 1. $1 = 켜기 직전 Module.ini 플랫폼 (기록용)
  say "1) R2 켜기 (CyclicBuffer 스트리밍: $R2_ARGS) — 켜기 직전 Module.ini: $1"
  local CLOG=$OUT/r2_launch.log CY TRACE="unset CR3A_R2_TRACE"
  # 제어기 설정: 곡선 모드 (cyclic_subsample:=hermite, 기본) 면 hermite, 아니면 stream (tools/rt_demo.sh 의 R2_ARGS 를 그대로 받으려고)
  case "$R2_ARGS" in *cyclic_subsample:=hermite*) CY=cr3a_controllers_hermite.yaml ;; *) CY=cr3a_controllers_stream.yaml ;; esac
  # [2026-10-27 R-1 안전 ⑭] R2 디버그 기록 (명령을 받을 때마다 한 줄, 읽는 코드 없음) 은 R2_TRACE=1 일 때만 — 예전엔 늘 켬 (실물에서도)
  [ "${R2_TRACE:-0}" = 1 ] && TRACE="export CR3A_R2_TRACE=$OUT/r2_trace.txt"
  say "   R2 기록 (CR3A_R2_TRACE): $([ "${R2_TRACE:-0}" = 1 ] && echo "켬 → $OUT/r2_trace.txt" || echo "끔 (켜려면 R2_TRACE=1)")"
  local SAMP="unset CR3A_R2_SAMPLES" TIMG="unset CR3A_R2_TIMING"       # [2026-10-30 PVAT 단계 1]
  [ "${R2_SAMPLES:-0}" = 1 ] && SAMP="export CR3A_R2_SAMPLES=$OUT/r2_samples.txt"
  [ "${R2_TIMING:-0}" = 1 ] && TIMG="export CR3A_R2_TIMING=$OUT/r2_timing.txt"
  [ "$R2_WS" != "$CWS/devel_cyclic_ws" ] && say "   R2 작업 공간: $R2_WS (1 ms 점 기록 ${R2_SAMPLES:-0} · 시간 기록 ${R2_TIMING:-0})"
  launch_root "$CLOG" "P=\$(ros2 pkg prefix --share wmx_r2_package); C=\$(ros2 pkg prefix --share wmx_r2_control); \
$TRACE; $SAMP; $TIMG; export CR3A_R2_VELINT=${R2_VELINT:-1}; \
exec ros2 launch wmx_r2_control wmx_r2_control_manipulator.launch.py stream_position:=true \
config_file:=\$P/example/cr3a_manipulator_config.yaml wmx_param_file:=\$P/example/cr3a_wmx_parameters.xml \
urdf_file:=\$C/urdf/cr3a.wmx.urdf.xacro controllers_file:=\$C/config/$CY use_sim_time:=false $R2_ARGS"
  local READY=0 L
  for i in $(seq 1 120); do
    if grep -q "WmxSystemHardware activated" "$CLOG" 2>/dev/null; then
      L=$(bash -c "$ENVSRC && timeout 10 ros2 control list_controllers 2>/dev/null")
      if echo "$L" | grep -q "joint_state_broadcaster.*active" && echo "$L" | grep -q "$ARM.*active"; then READY=1; break; fi
    fi
    grep -q -E "Cyclic buffer (open|start) failed|Failed to (servo-on|import)|not Communicating after|FATAL" "$CLOG" 2>/dev/null && break
    [ -n "$(r2_pid)" ] || break
    sleep 0.5
  done
  if [ $READY != 1 ]; then say "✗ R2 준비 안 됨"; tail -25 "$CLOG" | cut -c1-220 | sed 's/^/    /'; stop_r2; return 1; fi
  grep -E "Servos enabled|Cyclic buffer streaming|Cyclic subsample" "$CLOG" | sed -e 's/^.*\]: //' | cut -c1-200 | sed 's/^/    /'
}

start_axis_log() {     # WMX 엔진 축 로그 (1 ms). 실패면 R2 끄고 1
  say "2) WMX 엔진 축 로그 시작 (1 ms, 최대 ${LOG_MAX_S} s)"
  rm -f "$OUT/stop"
  ( cd "$AXLOG" && sudo -n setsid -f ./wmx_axis_log log "$OUT" wmx_axis $LOG_MAX_S "$OUT/stop" "$OUT/wmx_api.dat" \
      > "$OUT/wmx_axis_log.out" 2>&1 < /dev/null )
  for i in $(seq 1 20); do grep -q "기록 시작" "$OUT/wmx_axis_log.out" 2>/dev/null && break; sleep 0.25; done
  if ! grep -q "기록 시작" "$OUT/wmx_axis_log.out" 2>/dev/null; then say "✗ WMX 로그를 못 켬"; cat "$OUT/wmx_axis_log.out" 2>/dev/null; stop_r2; return 1; fi
}

back_to_simu() {     # (up-real) 켤 때 simu → EtherCAT 으로 바꿨으면 다시 simu (엔진이 꺼진 뒤에만)
  [ -f $SWITCHED_EC ] || return 0
  local e; e=$(engine_pid)
  if [ -n "$e" ]; then say "엔진이 아직 떠 있음 (PID $e) → Module.ini 는 EtherCAT 그대로 (엔진이 꺼진 뒤 down 을 다시)"; return 1; fi
  say "Module.ini 되돌리기 (시뮬레이터)"
  "$HERE/wmx/wmx_mode.sh" simu | sed 's/^/    /'
  [ "${PIPESTATUS[0]}" = 0 ] && rm -f $SWITCHED_EC
}

up_real() {
  [ -n "$OUT" ] || { echo "결과 폴더가 없음"; exit 2; }
  mkdir -p "$OUT"
  say "실제 로봇 (EtherCAT) 켜기 → $OUT  ★ 실제 CR3A 에 통신·서보 ON ★"
  if [ -n "$(r2_pid)" ]; then say "지난번 R2 가 남아 있음 → 끄고 다시 켬"; stop_r2 || exit 1; fi
  if other_r2 >/dev/null; then say "✗ 다른 R2 노드가 돌고 있음 → 안 켬"; other_r2 | head -3 | sed 's/^/    /'; exit 1; fi
  local P EP
  EP=$(engine_pid)
  [ -z "$EP" ] || { say "✗ WMX 엔진이 이미 떠 있음 (PID $EP, 다른 사람이 쓰는지 확인) → 안 켬"; exit 3; }
  P=$(ini_platforms)
  if [ "$P" = "simu_platform.so " ]; then
    say "Module.ini: 시뮬레이터 → EtherCAT (끌 때 다시 시뮬레이터로)"
    "$HERE/wmx/wmx_mode.sh" ethercat | sed 's/^/    /'
    [ "${PIPESTATUS[0]}" = 0 ] || { say "✗ EtherCAT 으로 못 바꿈 → 중단"; exit 2; }
    touch $SWITCHED_EC
    P=$(ini_platforms)
  fi
  say "0) Module.ini 의 켜진 플랫폼: ${P:-없음}"
  [ "$P" = "ec_platform.so " ] || { say "✗ EtherCAT 하나만 켜져 있지 않음 → 중단"; back_to_simu; exit 2; }
  build_check || { back_to_simu; exit 1; }
  build_axis_log || { back_to_simu; exit 1; }
  start_r2 "$P" || { back_to_simu; exit 1; }
  EP=$(engine_pid)
  say "   서보 켠 엔진 (PID ${EP:-없음}) 플랫폼"
  sudo -n $CHK 2>&1 | grep -E "엔진 상태|플랫폼 모듈" | sed 's/^/    /'
  start_axis_log || { back_to_simu; exit 1; }
  say "준비 끝: R2 PID $(r2_pid) · 엔진 PID $EP = EtherCAT · 서보 켜짐 (실제 로봇)"
}

servo_off() {     # R2 를 끄기 전에, 통신이 살아 있을 때 서보 OFF
  # [2026-10-26 시험] launch 를 그냥 끄면 엔진 노드가 통신을 먼저 끊어 하드웨어의 서보 OFF 가 "Communication is not started" 로 실패
  #  (실물이면 드라이브가 통신 끊김으로 스스로 멈추는 셈) → 팔 제어기 끄기 → 하드웨어 비활성 (버퍼 중단 → 정지 → 서보 OFF) 을 먼저
  [ -n "$(r2_pid)" ] || return 0
  local CLOG=${OUT:+$OUT/r2_launch.log}
  bash -c "$ENVSRC && timeout 15 ros2 control switch_controllers --deactivate $ARM >/dev/null 2>&1; \
    timeout 20 ros2 control set_hardware_component_state WmxManipulatorArmSystem inactive" >/dev/null 2>&1
  if [ -n "$CLOG" ] && grep -q "WmxSystemHardware deactivated" "$CLOG" 2>/dev/null; then say "서보 OFF (팔 제어기 끔 → 하드웨어 비활성)"
  else say "⚠ 하드웨어 비활성 확인 못 함 — R2 끄기로 진행"; fi
}

down() {
  servo_off                # 축 로그를 켠 채로 (서보 OFF 까지 기록)
  if [ -n "$OUT" ] && [ -f "$OUT/wmx_axis_log.out" ] && ! grep -q "기록 끝" "$OUT/wmx_axis_log.out"; then
    touch "$OUT/stop"
    for i in $(seq 1 40); do grep -q "기록 끝" "$OUT/wmx_axis_log.out" && break; sleep 0.25; done
    tail -1 "$OUT/wmx_axis_log.out" | sed 's/^/    /'
  fi
  stop_r2 || exit 1
  restore
  back_to_simu
  [ -n "$OUT" ] && [ -d "$OUT" ] && sudo -n chown -R "$(id -u):$(id -g)" "$OUT" 2>/dev/null
  return 0
}

encoders() {
  [ -n "$OUT" ] || { echo "결과 폴더가 없음"; exit 2; }
  mkdir -p "$OUT"
  [ -z "$(r2_pid)" ] || { say "✗ GUI 의 R2 가 켜져 있음 → 시스템을 먼저 끄기"; exit 1; }
  if other_r2 >/dev/null; then say "✗ 다른 R2 노드가 돌고 있음 → 안 함"; other_r2 | head -3 | sed 's/^/    /'; exit 1; fi
  local EP P WAS_SIMU=0 RC
  EP=$(engine_pid)
  [ -z "$EP" ] || { say "✗ WMX 엔진이 떠 있음 (PID $EP) → 안 함"; exit 3; }
  P=$(ini_platforms)
  if [ "$P" = "simu_platform.so " ]; then
    WAS_SIMU=1
    say "Module.ini: 시뮬레이터 → EtherCAT (실제 로봇 엔코더를 읽으려고. 끝나면 다시 시뮬레이터)"
    "$HERE/wmx/wmx_mode.sh" ethercat | sed 's/^/    /'
    [ "${PIPESTATUS[0]}" = 0 ] || exit 2
    rm -f $SWITCHED
    P=$(ini_platforms)
  fi
  [ "$P" = "ec_platform.so " ] || { say "✗ Module.ini 가 EtherCAT 하나만이 아님: ${P:-없음} → 안 함"; exit 2; }
  say "1) R2 일반 노드만 켬 (엔진·통신 + 축 상태). 서보를 켜는 노드 (joint_state_broadcaster·ros2_control·그리퍼·IO) 는 안 띄움"
  launch_root "$OUT/r2_encoders.log" "P=\$(ros2 pkg prefix --share wmx_r2_package); \
exec ros2 launch $HERE/wmx/wmx_read_encoders.launch.py wmx_param_file:=\$P/example/cr3a_wmx_parameters.xml"
  say "2) 축 상태 1 s 읽기 (서보가 켜진 축이 보이면 바로 멈춤)"
  bash -c "$ENVSRC && timeout 90 python3 -u $HERE/wmx_encoders.py '$OUT/encoders.json'" 2>&1 | grep -v -E "^$" | sed 's/^/    /'
  RC=${PIPESTATUS[0]}
  stop_r2
  grep -E "Device created|Communicat|imported|ImportAndSetAll|ERROR|FATAL" "$OUT/r2_encoders.log" | sed -e 's/^.*\]: //' | cut -c1-160 | sort -u | head -8 | sed 's/^/    /'
  if [ $WAS_SIMU = 1 ]; then
    EP=$(engine_pid)
    if [ -n "$EP" ]; then say "✗ 엔진이 아직 떠 있어 시뮬레이터로 못 되돌림 (PID $EP)"; exit 1; fi
    say "Module.ini: 다시 시뮬레이터로"
    "$HERE/wmx/wmx_mode.sh" simu | sed 's/^/    /'
  fi
  say "Module.ini 켜진 플랫폼: $(ini_platforms)"
  sudo -n chown -R "$(id -u):$(id -g)" "$OUT" 2>/dev/null
  [ "$RC" = 0 ] || { say "✗ 엔코더 읽기 실패 (코드 $RC)"; exit 1; }
}

status() {
  echo "Module.ini 켜진 플랫폼: $(ini_platforms)"
  local e p; e=$(engine_pid); p=$(r2_pid)
  echo "엔진: ${e:-꺼짐} · GUI R2: ${p:-꺼짐} · simu 로 바꿔 둠: $([ -f $SWITCHED ] && echo 예 || echo 아니오)" \
       "· EtherCAT 으로 바꿔 둠 (실물): $([ -f $SWITCHED_EC ] && echo 예 || echo 아니오)"
  other_r2 | head -3
}

case "$CMD" in
  up) up ;;
  up-real) up_real ;;
  down) down ;;
  encoders) encoders ;;
  status) status ;;
  *) echo "사용법: $0 up <폴더> | up-real <폴더> | down [폴더] | encoders <폴더> | status"; exit 2 ;;
esac
