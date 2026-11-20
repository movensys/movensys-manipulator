# [만듦 2026-10-13] WMX R2 + 경로 계획·장애물 회피 작업용 이미지 — Thor (ARM64, Ubuntu 24.04, JetPack 7, ROS 2 Jazzy)
#   빌드: ./run.sh build   (Thor 는 인터넷이 없어서 PC 의 프록시 통로를 켠 상태로. wmx_r2_작업기록.md 2장)
#   들어 있는 것
#     - WMX R2 (github.com/movensys/wmx-r2) 빌드·실행에 필요한 ROS 패키지: feature/docker-setup 브랜치의 general.Dockerfile
#       + main 의 doc/first_setup.md 목록 + package.xml 의 depend/test_depend
#     - MoveIt (first_setup.md 목록 + movensys-manipulator 의 moveit_config)
#     - GPU 경로 계획·장애물 회피 라이브러리: PyTorch 2.9.0+cu130, warp 1.17.0+cu13, cuRoboV2 (NVlabs/curobo 78fd485)
#       = Mirobot 컨테이너에서 쓰고 있는 것과 같은 판 (curobo_v2_작업기록.md 2장). Isaac ROS(cuMotion v1)·nvblox 는 넣지 않음
#     - RealSense 카메라 드라이버 (장애물 지도용)
#   이미지에 넣지 않는 것 (컨테이너를 만들 때 호스트 것을 연결한다. run.sh 참고)
#     - WMX3 SDK /opt/wmx3 : 엔진·라이선스·NIC 드라이버가 호스트 것이라 헤더/라이브러리도 같은 것을 써야 버전이 맞는다
#     - 소스 ~/workspaces/movensys_ws : 고치면 바로 반영되게 (참고_도커이미지와_ws.md 5장 기준과 같음)
#
# 바탕: nvidia/cuda:13.0.2-devel-ubuntu24.04 (Mirobot 이미지와 같은 바탕. 드라이버 580 = CUDA 13.0 과 맞음. Thor 에 이미 받아 둠)
FROM nvidia/cuda:13.0.2-devel-ubuntu24.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    NVIDIA_DRIVER_CAPABILITIES=all

# --- 우분투 ARM 저장소를 국내 미러로 (ports.ubuntu.com 0.7MB/s → KAIST 9MB/s. Thor_작업기록.md 2장) ---
ARG UBUNTU_MIRROR=http://ftp.kaist.ac.kr/ubuntu-ports
RUN sed -i "s#http://ports.ubuntu.com/ubuntu-ports#${UBUNTU_MIRROR}#g" /etc/apt/sources.list.d/ubuntu.sources \
    && grep -h '^URIs' /etc/apt/sources.list.d/ubuntu.sources

# --- ROS 2 Jazzy 저장소 (Mirobot Dockerfile.thor 와 같은 방법) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release software-properties-common ca-certificates locales \
    && add-apt-repository -y universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list \
    && rm -rf /var/lib/apt/lists/*

# --- 1) ROS 기본 + wmx-r2 빌드·실행·시험에 필요한 것 ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-controller-manager \
        ros-jazzy-joint-trajectory-controller \
        ros-jazzy-joint-state-broadcaster \
        ros-jazzy-diff-drive-controller \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-xacro \
        ros-jazzy-topic-tools \
        ros-jazzy-graph-msgs \
        ros-jazzy-tf-transformations \
        ros-jazzy-rviz2 \
        ros-jazzy-ros-testing \
        ros-jazzy-launch-testing-ament-cmake \
        ros-jazzy-ament-lint-auto \
        ros-jazzy-ament-lint-common \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-ament-cmake-gtest \
        python3-colcon-common-extensions \
        python3-rosdep python3-pip python3-venv python3-pytest \
        build-essential cmake git nano less sudo udev iproute2 iputils-ping procps \
        mesa-utils libgl1 libglx-mesa0 \
        fonts-nanum \
    && rm -rf /var/lib/apt/lists/*

# --- 2) MoveIt + 카메라 (RealSense: Mirobot 과 같은 ROS 패키지. cv_bridge·opencv: movensys_manipulator_perception 이 씀) ---
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-moveit-ros \
        ros-jazzy-moveit-servo \
        ros-jazzy-moveit-planners \
        ros-jazzy-moveit-plugins \
        ros-jazzy-moveit-setup-assistant \
        ros-jazzy-moveit-configs-utils \
        ros-jazzy-moveit-task-constructor-core \
        ros-jazzy-moveit-resources-panda-description \
        ros-jazzy-realsense2-camera \
        ros-jazzy-cv-bridge \
        python3-opencv \
        python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# --- 3) GPU 계획 라이브러리: PyTorch + warp + cuRoboV2 ---
#  어디에: 시스템 파이썬(/usr/local/lib/python3.12/dist-packages)에 바로 넣는다 → ros2 run / launch 로 띄우는 파이썬 노드도
#   그대로 import torch, curobo 가 된다. (Mirobot 은 cuMotion v1 과 이름이 겹쳐 별도 가상환경 + PYTHONPATH 순서로 피했지만,
#   이 이미지에는 v1 이 없어서 그럴 필요가 없음)
#  판: Mirobot 컨테이너에서 Thor sm_110 으로 검증된 것과 같게 고정 (curobo_v2_작업기록.md 2장, mirobot_ws/curobo_v2/wheels)
#   - torch 2.9.0+cu130 : PyTorch 공식 휠 (Mirobot 의 isaac-ros 가상환경에 있는 것과 같은 판)
#   - warp 1.17.0+cu13  : GitHub 릴리스의 CUDA 13 빌드 (apt/PyPI 의 옛 판은 V2 가 쓰는 wp.func(module=...) 가 없음)
#   - CUDA 파이썬 라이브러리 13.0 고정 (cuda-bindings 13.0.3, nvrtc 13.0.48): 드라이버(CUDA 13.0)·torch 와 맞춤
#   - numpy 1.26.4 : 우분투 apt 판과 같음 (ROS 파이썬 도구와 섞여도 안전)
#  V2 의존성 줄의 --ignore-installed: apt 가 깐 파이썬 패키지(packaging 24.0 등)는 pip 가 못 지워서 빌드가 멈춘다
#   ("Cannot uninstall packaging 24.0, RECORD file not found") → 지우지 않고 /usr/local 에 새로 깔아 앞에 오게 한다. apt 것은 그대로
ARG TORCH_INDEX=https://download.pytorch.org/whl/cu130
ARG WARP_WHL=https://github.com/NVIDIA/warp/releases/download/v1.17.0/warp_lang-1.17.0+cu13-py3-none-manylinux_2_34_aarch64.whl
ARG CUROBO_COMMIT=78fd485fa82d9b9a063fb4985e371814587e666a
RUN printf '%s\n' \
        'numpy==1.26.4' 'cuda-bindings==13.0.3' 'nvidia-cuda-nvrtc==13.0.48' 'nvidia-cuda-runtime==13.0.48' \
        'nvidia-nvjitlink==13.0.39' 'cuda-toolkit==13.0.*' > /opt/gpu_constraints.txt \
    && PIP="python3 -m pip install --no-cache-dir --break-system-packages -c /opt/gpu_constraints.txt" \
    && $PIP --index-url ${TORCH_INDEX} --extra-index-url https://pypi.org/simple torch==2.9.0+cu130 \
    && $PIP --no-deps "${WARP_WHL}" \
    && $PIP --ignore-installed "cuda-core[cu13]>=0.7" viser numpy-quaternion tqdm importlib_resources trimesh "scipy>=1.7.0" \
        "yourdfpy>=0.0.53" setuptools_scm wheel networkx pyyaml \
    && git clone https://github.com/NVlabs/curobo.git /opt/curobo_v2 \
    && git -C /opt/curobo_v2 checkout -q ${CUROBO_COMMIT} \
    && cd /opt/curobo_v2 && $PIP --no-deps --no-build-isolation -e . \
    && python3 -c "import importlib.util as u, torch, warp; print('torch', torch.__version__, 'warp', warp.__version__, 'curobo', u.find_spec('curobo').origin)"
#  (빌드 중에는 GPU 가 없어서 curobo 는 찾기만 한다. GPU 로 실제 계산하는 확인은 run.sh check 7번)

# --- 3b) [2026-10-27 R-1] CR3A GUI 3D 뷰 (cr3a_gui/scene3d.py) 를 GPU 로 그림: pyqtgraph.opengl + PyOpenGL (순수 파이썬 휠) ---
#  판 = 2026-10-27 Thor 컨테이너에 (pip --user 로) 깔아 확인한 것. numpy 는 위 제약 (1.26.4) 그대로 (pyqtgraph 0.14 는 numpy>=1.25).
#  PyQt5 는 따로 넣지 않음 (지금 컨테이너의 PyQt5 5.15.10 = apt 것, /usr/lib/python3/dist-packages). 빌드 중에는 화면·GPU 가 없어서 판만 확인
RUN python3 -m pip install --no-cache-dir --break-system-packages -c /opt/gpu_constraints.txt \
        pyqtgraph==0.14.0 PyOpenGL==3.1.10 \
    && python3 -c "import importlib.metadata as m; print('pyqtgraph', m.version('pyqtgraph'), 'PyOpenGL', m.version('PyOpenGL'))"

# --- 4) Thor(ARM) 전용 라이브러리 경로 (Mirobot Dockerfile.thor / 공식 isaac-ros-cli Dockerfile 의 arm64 부분) ---
#  tegra: 호스트 GPU 드라이버가 nvidia 런타임으로 여기에 들어온다. sbsa-linux: CUDA 라이브러리.
#  /opt/wmx3/lib: WMX3 공유 라이브러리(libimdll.so). 실제 파일은 컨테이너를 만들 때 호스트 것을 연결 (ldconfig 는 run.sh 가 켤 때 다시 함)
ENV WMX3_SDK_PATH=/opt/wmx3
ENV LD_LIBRARY_PATH="/opt/wmx3/lib:/usr/lib/aarch64-linux-gnu/tegra:/usr/local/cuda/targets/sbsa-linux/lib:/usr/lib/aarch64-linux-gnu/tegra-egl:${LD_LIBRARY_PATH}"
RUN mkdir -p /opt/wmx3 && echo "/opt/wmx3/lib" > /etc/ld.so.conf.d/wmx3.conf

# --- 5) 사용자: 호스트 사용자와 번호(UID/GID)를 맞춘다 → ws 파일 주인이 호스트에서도 mic-735d 로 보임 ---
#  이름은 공식 도커 설정(feature/docker-setup)과 같은 admin. 바탕 이미지의 1000번 사용자(ubuntu)가 있으면 이름만 바꾼다.
ARG USERNAME=admin
ARG UID=1000
ARG GID=1000
RUN set -e; \
    old_group=$(getent group ${GID} | cut -d: -f1 || true); \
    if [ -z "$old_group" ]; then groupadd -g ${GID} ${USERNAME}; \
    elif [ "$old_group" != "${USERNAME}" ]; then groupmod -n ${USERNAME} "$old_group"; fi; \
    old_user=$(getent passwd ${UID} | cut -d: -f1 || true); \
    if [ -z "$old_user" ]; then useradd -m -u ${UID} -g ${GID} -s /bin/bash ${USERNAME}; \
    elif [ "$old_user" != "${USERNAME}" ]; then usermod -l ${USERNAME} -d /home/${USERNAME} -m "$old_user"; fi; \
    usermod -aG sudo,dialout,video ${USERNAME}; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}; chmod 440 /etc/sudoers.d/${USERNAME}; \
    mkdir -p /home/${USERNAME}/workspaces/movensys_ws; chown -R ${UID}:${GID} /home/${USERNAME}/workspaces

# --- 6) 셸 환경: admin / root 둘 다 같은 파일을 읽는다 (WMX 노드는 root 로 띄워야 해서. README "launches require root") ---
#  마지막 줄은 if 문으로: "[ -f ] && source" 로 쓰면 빌드 전(install 없음)에 1 을 돌려줘서 run.sh 의 "source ... && colcon" 이 멈춘다
RUN printf '%s\n' \
      '# [만듦 2026-10-13] wmx_r2 컨테이너 공통 환경 (Dockerfile 6번)' \
      'source /opt/ros/jazzy/setup.bash' \
      'WMX_R2_WS=/home/admin/workspaces/movensys_ws' \
      'if [ -f $WMX_R2_WS/install/setup.bash ]; then source $WMX_R2_WS/install/setup.bash; fi' \
      > /opt/wmx_r2_env.sh \
    && echo 'source /opt/wmx_r2_env.sh' >> /home/${USERNAME}/.bashrc \
    && echo 'source /opt/wmx_r2_env.sh' >> /root/.bashrc
RUN rosdep init || true

USER ${USERNAME}
WORKDIR /home/${USERNAME}/workspaces/movensys_ws
CMD ["bash"]
