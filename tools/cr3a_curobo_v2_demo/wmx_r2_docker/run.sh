#!/bin/bash
# [만듦 2026-10-13] WMX R2 작업용 컨테이너 (wmx_r2) 만들기 / 들어가기 / 빌드 / 확인.  ★ Thor 에서 실행 ★  (wmx_r2_작업기록.md)
#   ./run.sh build    이미지 빌드 → wmx_r2:thor   (Thor 는 인터넷이 없어서 PC 의 프록시 통로를 켜고:
#                     http_proxy=http://127.0.0.1:3128 https_proxy=http://127.0.0.1:3128 ./run.sh build)
#   ./run.sh          컨테이너가 없으면 만들고, 켜고, admin 으로 들어감 (빌드·편집용)
#   ./run.sh create   만들고 켜기만 함
#   ./run.sh colcon   ws 빌드 (admin 으로. 결과 build/ install/ log/ 는 호스트 ws 폴더에 생김)
#   ./run.sh check    읽기만 하는 점검 (라이브러리 연결, 패키지, URDF, 호스트 WMX 엔진 연결, GPU·cuRoboV2). 로봇은 안 움직임
#   ./run.sh root [명령]  root 로 들어가기 / 명령 실행 (노드 실행은 root 가 필요. 공식 wros 와 같은 역할)
#   ./run.sh stop     컨테이너 끄기          ./run.sh rm   컨테이너 지우기 (ws 파일은 호스트에 남음)
#
# 이 컨테이너 하나에서 WMX R2(로봇 구동) + cuRoboV2(경로 계획·장애물 회피) + 카메라를 같이 쓴다 (Mirobot 컨테이너와는 따로).
#
# 폴더 (호스트 → 컨테이너, 같은 폴더를 두 곳에서 봄)
#   ~/workspaces/movensys_ws      → /home/admin/workspaces/movensys_ws   (src/wmx-r2, src/movensys-manipulator)
#   /opt/wmx3 (WMX3 SDK 3.7.0)    → /opt/wmx3            WMX API 가 엔진을 찾는 경로. 노드 안에 "/opt/wmx3/" 로 컴파일됨
#   /usr/local/share/.shm         → 같은 경로            WMX API ↔ 엔진 공유메모리 (공식 도커 설정과 같음)
#   /etc/eni                      → 같은 경로 (읽기만)   엔진을 컨테이너 안에서 켜게 될 때 슬레이브 설정을 찾도록
#   /dev                          → /dev                 EtherCAT NIC(uio), 라이선스(/dev/aks) 등. 다시 꽂아도 보이게
#   ~/workspaces/wmx_r2_logs      → 같은 경로 (/home/mic-735d/...)  [추가 2026-10-13] WMX 로그 파일. 로그는 엔진이 쓰는데 엔진은
#                                   호스트(WMX Home)에도 컨테이너(R2)에도 뜰 수 있어서, 양쪽에서 같은 경로가 되게 함
# 권한: --privileged + 호스트 네트워크/IPC/PID 공유 (공식 docker/general.yaml 과 같음. 실시간 스레드, 엔진 프로세스 확인용)
# GPU: --runtime nvidia (Thor 는 --gpus all 이 안 됨, Mirobot run.sh 와 같음) + render 그룹 번호 (Thor 993)
set -e

IMAGE=wmx_r2:thor
NAME=wmx_r2
CUSER=admin
HERE="$(cd "$(dirname "$0")" && pwd)"
WS_HOST=${WS_HOST:-$HOME/workspaces/movensys_ws}
WS=/home/$CUSER/workspaces/movensys_ws
LOGS=${LOGS:-$HOME/workspaces/wmx_r2_logs}
# ROS 도메인: 공식 문서 값 70. Mirobot 컨테이너(0)와 일부러 나눈다.
#   이유: 둘 다 관절 이름 joint1..joint6 + /joint_states 를 써서, 같은 도메인에서 Mirobot 브릿지와 WMX 를 같이 켜면 관절 값이 섞인다.
#   나머지는 Mirobot 과 같게: CycloneDDS, 이 PC 안에서만 찾기(LOCALHOST), 호스트 네트워크.
DOMAIN=${ROS_DOMAIN_ID:-70}
RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
# 무시할 패키지: Isaac ROS(cuMotion v1)가 필요한 설정. 계획은 cuRoboV2 로 하므로 빌드하지 않는다.
SKIP_PKGS=(movensys_manipulator_isaac_ros_config)
# GPU (Thor): nvidia 런타임 + /dev/dri 의 render 그룹 번호
GPU_ARGS=(--runtime nvidia --env NVIDIA_VISIBLE_DEVICES=all --env NVIDIA_DRIVER_CAPABILITIES=all)
R=$(getent group render | cut -d: -f3); [ -n "$R" ] && GPU_ARGS+=(--group-add "$R")

ENVSRC="source /opt/wmx_r2_env.sh"

engine_pid() {  # 호스트에서 살아 있는 wmx3_engine (좀비 제외)
  ps -C wmx3_engine -o stat=,pid= 2>/dev/null | awk '$1 !~ /^Z/ {print $2; exit}'
}
build_status_tool() {  # tools/wmx_status_check.cpp → 컨테이너 /tmp/wmx_status_check (읽기 전용 엔진 확인 프로그램)
  docker cp "$HERE/tools/wmx_status_check.cpp" $NAME:/tmp/wmx_status_check.cpp
  docker exec -u root $NAME g++ -std=c++17 -O1 -I/opt/wmx3/include /tmp/wmx_status_check.cpp \
    -L/opt/wmx3/lib -lwmx3api -limdll -lpthread -lrt -ldl -o /tmp/wmx_status_check
}

case "$1" in
  build)
    # 프록시를 쓰는 중이면 빌드에도 넘긴다. 빌드 안에서 127.0.0.1 이 Thor 자신(= 통로 입구)을 가리키도록 --network host.
    PROXY_ARGS=()
    for v in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY; do
      if [ -n "${!v}" ]; then PROXY_ARGS+=(--build-arg "$v=${!v}"); fi
    done
    if [ ${#PROXY_ARGS[@]} -gt 0 ]; then PROXY_ARGS+=(--network host); fi
    docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) --build-arg USERNAME=$CUSER \
      "${PROXY_ARGS[@]}" -f "$HERE/Dockerfile" -t $IMAGE "$HERE"
    exit ;;
  rm)
    docker rm -f $NAME
    exit ;;
  stop)
    docker stop $NAME
    exit ;;
esac

# 화면 번호: ssh 로 들어와 DISPLAY 가 없으면 켜져 있는 화면을 찾는다 (Thor 는 :1). rviz2 를 띄울 때만 필요.
DISP=${DISPLAY:-:$(ls /tmp/.X11-unix 2>/dev/null | sed -n 's/^X//p' | head -1)}
[ "$DISP" = ":" ] && DISP=:0
XA=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}
DISPLAY=$DISP XAUTHORITY=$XA xhost +local:docker >/dev/null 2>&1 || true

if ! docker container inspect $NAME >/dev/null 2>&1; then
  [ -d "$WS_HOST/src" ] || { echo "ws 가 없음: $WS_HOST/src"; exit 1; }
  [ -d /opt/wmx3/lib ] || { echo "WMX3 SDK 가 없음: /opt/wmx3"; exit 1; }
  mkdir -p "$LOGS"
  echo "컨테이너 $NAME 를 $IMAGE 로 만듭니다. (ROS_DOMAIN_ID=$DOMAIN)"
  docker create -it --name $NAME \
    "${GPU_ARGS[@]}" \
    --privileged \
    --network host --ipc host --pid host \
    --ulimit rtprio=99 --ulimit memlock=-1 \
    --env DISPLAY=$DISP \
    --env QT_X11_NO_MITSHM=1 \
    --env ROS_DOMAIN_ID=$DOMAIN \
    --env ROS_AUTOMATIC_DISCOVERY_RANGE=$RANGE \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume /etc/localtime:/etc/localtime:ro \
    --volume "$WS_HOST":$WS \
    --volume /opt/wmx3:/opt/wmx3 \
    --volume /usr/local/share/.shm:/usr/local/share/.shm \
    --volume /etc/eni:/etc/eni:ro \
    --volume /dev:/dev \
    --volume "$LOGS":"$LOGS" \
    "$IMAGE" >/dev/null
fi

docker start $NAME >/dev/null
# /opt/wmx3/lib 는 연결된 뒤에야 보이므로 켤 때마다 라이브러리 목록을 다시 만든다 (몇 ms)
docker exec -u root $NAME ldconfig

T=(-i); [ -t 0 ] && T=(-it)   # 터미널이 아닐 때(스크립트, ssh 명령)도 되게
case "$1" in
  create)
    exit 0 ;;
  colcon)
    shift
    exec docker exec "${T[@]}" -u $CUSER -w $WS $NAME bash -c "$ENVSRC && \
      colcon build --symlink-install --packages-ignore ${SKIP_PKGS[*]} $*" ;;
  root)
    shift
    if [ $# -eq 0 ]; then exec docker exec "${T[@]}" -u root -w $WS $NAME bash; fi
    if [ -t 0 ]; then
      exec docker exec -it -u root -w $WS --env ROS_DOMAIN_ID=$DOMAIN $NAME bash -c "$ENVSRC && $*"
    fi
    # [고침 2026-10-13, 리뷰 지적] 터미널이 없으면(스크립트, ssh 명령) docker exec 는 Ctrl+C·연결 끊김을 컨테이너 안으로
    #  넘기지 않는다 → 노드가 root 로 계속 돈다. 그래서 안쪽 프로세스 번호를 적어 두고, 신호를 받으면 그 프로세스에 SIGINT.
    PIDF=/tmp/run_root.$$.pid
    trap 'docker exec -u root $NAME bash -c "[ -f $PIDF ] && kill -INT \$(cat $PIDF)" 2>/dev/null' INT TERM HUP
    docker exec -i -u root -w $WS --env ROS_DOMAIN_ID=$DOMAIN $NAME bash -c "$ENVSRC && echo \$\$ > $PIDF && exec $*" &
    CPID=$!; RC=0
    while kill -0 $CPID 2>/dev/null; do wait $CPID || RC=$?; done   # 신호를 받으면 wait 가 먼저 끝나므로 다시 기다림
    docker exec -u root $NAME rm -f $PIDF
    exit $RC ;;
  check)
    echo "== 1. 빌드된 노드가 WMX 라이브러리를 찾는가 (ldd: not found 가 없어야 함)"
    docker exec -u root $NAME bash -c "$ENVSRC; \
      d=$WS/install/wmx_r2_package/lib/wmx_r2_package; [ -d \$d ] || { echo '  아직 빌드 안 됨 (./run.sh colcon)'; exit 0; }; \
      for f in \$d/* $WS/install/wmx_r2_control/lib/libwmx_system_hardware.so; do \
        nf=\$(ldd \$f | grep -c 'not found' || true); im=\$(ldd \$f | awk '/libimdll/{print \$3}'); \
        printf '  %-32s not-found=%s  libimdll=%s\n' \$(basename \$f) \$nf \"\${im:--}\"; done"
    echo "== 2. ROS 패키지 / 메시지"
    docker exec -u root $NAME bash -c "$ENVSRC; \
      for p in wmx_r2_message wmx_r2_package wmx_r2_control movensys_manipulator_description movensys_manipulator_moveit_config \
               movensys_manipulator_perception realsense2_camera; do \
        printf '  %-36s %s\n' \$p \"\$(ros2 pkg prefix \$p 2>/dev/null || echo 없음)\"; done; \
      echo \"  wmx_r2_message 인터페이스 \$(ros2 interface package wmx_r2_message 2>/dev/null | wc -l)개\"; \
      echo \"  wmx_r2_package 실행 파일: \$(ros2 pkg executables wmx_r2_package 2>/dev/null | awk '{print \$2}' | tr '\n' ' ')\""
    echo "== 3. CR3A URDF (ros2_control 판) 만들어지는가"
    docker exec -u root $NAME bash -c "$ENVSRC; \
      x=\$(ros2 pkg prefix wmx_r2_control 2>/dev/null)/share/wmx_r2_control/urdf/cr3a.wmx.urdf.xacro; \
      xacro \$x > /tmp/cr3a.urdf 2>/tmp/cr3a.err && \
        echo \"  OK: 링크 \$(grep -c '<link ' /tmp/cr3a.urdf)개, 관절 \$(grep -c '<joint ' /tmp/cr3a.urdf)개, plugin \$(grep -o '<plugin>[^<]*' /tmp/cr3a.urdf | head -1 | cut -c9-)\" || \
        { echo '  실패:'; tail -3 /tmp/cr3a.err; }"
    echo "== 4. 호스트 WMX 엔진에 컨테이너에서 붙을 수 있는가 (읽기만: 상태·장치 목록. 통신/서보/파라미터 안 건드림)"
    EP=$(engine_pid)
    if [ -z "$EP" ]; then
      echo "  엔진이 꺼져 있어 건너뜀 (붙는 순간 엔진이 켜지므로). WMX Home 이나 launch 로 엔진을 켠 뒤 다시 check."
    else
      echo "  호스트 엔진 PID $EP ($(ps -o user= -p $EP), 부모: $(ps -o comm= -p $(ps -o ppid= -p $EP)))"
      # [고침 2026-10-13, 리뷰 지적] 컴파일(몇 초)을 먼저 하고, 붙기 바로 앞에서 엔진이 아직 살아 있는지 다시 본다.
      #  그 틈에 엔진이 꺼지면 CreateDevice 가 컨테이너 안에서 엔진을 새로 켜기 때문. (--pid host 라 호스트 PID 가 보임)
      build_status_tool
      docker exec -u root $NAME bash -c "s=\$(ps -o stat= -p $EP 2>/dev/null); \
          if [ -n \"\$s\" ] && [ \"\${s:0:1}\" != Z ]; then /tmp/wmx_status_check; else echo '엔진이 방금 꺼져서 건너뜀'; fi" \
        | sed 's/^/  /'
    fi
    echo "== 5. 같은 ROS 도메인($DOMAIN)에 지금 보이는 것 (/joint_states 를 이미 내는 노드가 있으면 WMX 와 섞임)"
    docker exec -u root $NAME bash -c "$ENVSRC; \
      n=\$(timeout 15 ros2 node list --no-daemon --spin-time 3 2>/dev/null | grep -v '^/_' ); \
      echo \"  노드 \$(echo -n \"\$n\" | grep -c .)개: \$(echo \$n | cut -c1-300)\"; \
      p=\$(timeout 15 ros2 topic info /joint_states --no-daemon --spin-time 3 2>/dev/null | awk '/Publisher count/{print \$3}'); \
      echo \"  /joint_states 발행자: \${p:-0}  (WMX R2 를 켜기 전에 0 이어야 함)\""
    echo "== 6. 호스트 실시간 설정 (참고)"
    echo "  커널 $(uname -r), isolcpus=$(grep -o 'isolcpus=[^ ]*' /proc/cmdline || echo 없음)"
    echo "== 7. GPU · cuRoboV2 (admin 으로. 로봇과 무관한 계산만: V2 기본 예제 = Franka 모델 IK)"
    # [추가 2026-10-13, 리뷰 지적] WMX 노드가 돌고 있으면 무거운 GPU/CPU 계산이 제어 주기를 흔들 수 있어 건너뛴다
    if pgrep -f "lib/wmx_r2_package/|share/wmx_r2_control/" >/dev/null; then
      echo "  WMX R2 노드가 돌고 있어 건너뜀"; exit 0
    fi
    docker exec -u $CUSER $NAME bash -c "$ENVSRC; python3 -c \"
import torch, warp, curobo
print('  torch', torch.__version__, '| CUDA', torch.cuda.is_available(), '|', torch.cuda.get_device_name(0) if torch.cuda.is_available() else '-',
      '| sm', ''.join(map(str, torch.cuda.get_device_capability(0))) if torch.cuda.is_available() else '-')
print('  warp', warp.__version__, '| curobo', curobo.__file__)
\" 2>&1 | grep -v -E '^Warp|^   ' ; \
      cd /tmp && timeout 600 python3 -m curobo.examples.getting_started.inverse_kinematics 2>&1 \
        | grep -E 'IK|solved|Solved|error|Error|Traceback' | grep -v -i 'warning' | head -12 | sed 's/^/  /'"
    exit 0 ;;
esac

exec docker exec -it -u $CUSER -w $WS $NAME bash
