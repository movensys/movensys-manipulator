ARG ARCH=amd64
FROM nvcr.io/nvidia/isaac/ros:isaac_ros_6a0af6f39da3232fdca6b60f4b174e8a-${ARCH}
ARG ROS_DISTRO=jazzy

USER root
WORKDIR /workspaces

RUN rm -f /etc/apt/sources.list.d/yarn.list || true

RUN sed -i -E 's|http://(archive\|security)\.ubuntu\.com/ubuntu/|https://\1.ubuntu.com/ubuntu/|g' \
      /etc/apt/sources.list.d/ubuntu.sources

RUN apt-get update && \
    apt-get install -y \
      ros-jazzy-ament-package \
      ros-jazzy-ament-index-cpp \
      ros-jazzy-ament-cmake-core \
      ros-jazzy-ament-index-python \
      ros-jazzy-pal-statistics \
      ros-jazzy-pal-statistics-msgs \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-tf-transformations \
      ros-jazzy-realsense2-camera \
      ros-jazzy-isaac-ros-apriltag \
      ros-jazzy-isaac-ros-realsense \
      ros-jazzy-isaac-ros-depth-image-proc \
      ros-jazzy-isaac-ros-image-proc \
      python3-colcon-common-extensions \
      python3-setuptools \
      ninja-build \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
      ros-jazzy-isaac-ros-cumotion-examples \
      ros-jazzy-isaac-manipulator-ros-python-utils \
      ros-jazzy-isaac-ros-nvblox \
    && rm -rf /var/lib/apt/lists/*

COPY fix_cumotion_planner.sh /tmp/fix_cumotion_planner.sh
RUN chmod +x /tmp/fix_cumotion_planner.sh && /tmp/fix_cumotion_planner.sh && rm /tmp/fix_cumotion_planner.sh

COPY fix_robot_segmenter.sh /tmp/fix_robot_segmenter.sh
RUN chmod +x /tmp/fix_robot_segmenter.sh && /tmp/fix_robot_segmenter.sh && rm /tmp/fix_robot_segmenter.sh

RUN apt-get update && apt-get install -y \
        ros-jazzy-rclcpp-action \
        ros-jazzy-moveit-ros \
        ros-jazzy-moveit-planners \
        ros-jazzy-moveit-plugins \
        ros-jazzy-moveit-setup-assistant \
        ros-jazzy-moveit-configs-utils \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    if [ "$ROS_DISTRO" = "jazzy" ]; then \
      apt-get install -y \
        ros-${ROS_DISTRO}-ros-gz-sim \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-ros-gz-bridge \
        ros-${ROS_DISTRO}-gz-sim-vendor \
        ros-${ROS_DISTRO}-gz-transport-vendor; \
    elif [ "$ROS_DISTRO" = "humble" ]; then \
      apt-get install -y \
        ros-${ROS_DISTRO}-ros-ign-gazebo \
        ros-${ROS_DISTRO}-ign-ros2-control \
        ros-${ROS_DISTRO}-ros-ign-bridge; \
    fi && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/lib/python3/dist-packages/transforms3d \
           /usr/lib/python3/dist-packages/transforms3d-*.egg-info \
           /usr/lib/python3/dist-packages/sympy \
           /usr/lib/python3/dist-packages/sympy-*.egg-info && \
    python3 -m pip install --no-cache-dir --break-system-packages \
        --index-url https://download.pytorch.org/whl/cu130 \
        torch torchvision && \
    python3 -m pip install --no-cache-dir --break-system-packages \
        "numpy<2" \
        opencv-python \
        "transforms3d>=0.4.1" \
        pyyaml \
        ultralytics \
        pyapriltags

COPY fix_curobo_lerp.sh /tmp/fix_curobo_lerp.sh
RUN chmod +x /tmp/fix_curobo_lerp.sh && /tmp/fix_curobo_lerp.sh && rm /tmp/fix_curobo_lerp.sh

ARG CUROBO_V2_COMMIT=78fd485fa82d9b9a063fb4985e371814587e666a
ARG WARP_VERSION=1.17.0
RUN if [ "$(dpkg --print-architecture)" = "arm64" ]; then WARP_PLATFORM=manylinux_2_34_aarch64; \
    else WARP_PLATFORM=manylinux_2_28_x86_64; fi && \
    python3 -m pip freeze | grep -iE '^(torch|torchvision|numpy|nvidia-|cuda-)' | grep -v ' @ ' \
      | grep -viE '^cuda-pathfinder==' \
      > /tmp/curobo_v2_constraints.txt && \
    PIP="python3 -m pip install --no-cache-dir --break-system-packages -c /tmp/curobo_v2_constraints.txt" && \
    $PIP --no-deps \
      "https://github.com/NVIDIA/warp/releases/download/v${WARP_VERSION}/warp_lang-${WARP_VERSION}+cu13-py3-none-${WARP_PLATFORM}.whl" && \
    $PIP "cuda-core[cu13]>=0.7" numpy-quaternion trimesh "scipy>=1.7.0" "yourdfpy>=0.0.53" \
      importlib_resources networkx tqdm setuptools_scm && \
    git clone https://github.com/NVlabs/curobo.git /opt/curobo_v2 && \
    git -C /opt/curobo_v2 checkout -q ${CUROBO_V2_COMMIT} && \
    chmod -R a+rX /opt/curobo_v2 && \
    rm /tmp/curobo_v2_constraints.txt && \
    PYTHONPATH=/opt/curobo_v2 python3 -c "import curobo, warp, torch; print('cuRoboV2', curobo.__version__, 'warp', warp.__version__, 'torch', torch.__version__)"
ENV CUROBO_V2_PATH=/opt/curobo_v2
