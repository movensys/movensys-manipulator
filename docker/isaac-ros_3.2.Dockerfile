ARG ARCH=amd64
FROM isaac_ros_dev-x86_64 AS base-amd64
FROM isaac_ros_dev-aarch64 AS base-arm64
FROM base-${ARCH}
ARG ROS_DISTRO=humble

USER root
WORKDIR /workspaces

RUN rm -f /etc/apt/sources.list.d/yarn.list || true

RUN sed -i 's|http://security.ubuntu.com/ubuntu|http://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    apt-get update

RUN apt-get update && \
    apt-get install -y \
        ros-humble-ament-package \
        ros-humble-ament-index-cpp \
        ros-humble-ament-cmake-core \
        ros-humble-ament-index-python \
        python3-colcon-common-extensions \
        python3-setuptools \
        ros-humble-pal-statistics \
        ros-humble-pal-statistics-msgs \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-tf-transformations \
        ros-humble-isaac-ros-cumotion-examples \
        ros-humble-isaac-ros-apriltag \
        ros-humble-isaac-ros-nvblox \
        ros-humble-isaac-ros-examples \
        ros-humble-isaac-ros-realsense \
        ros-humble-isaac-ros-ess \
        ros-humble-isaac-ros-ess-models-install \
        ros-humble-joint-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-xacro \
        ros-humble-rqt* \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-controller-manager \
        ros-humble-moveit* \
        curl jq tar \
        ninja-build && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y curl gpg && \
    mkdir -p /etc/apt/keyrings && \
    curl -sSf "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xFB0B24895113F120" | \
        gpg --dearmor > /etc/apt/keyrings/librealsense.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsense.gpg] https://librealsense.intel.com/Debian/apt-repo jammy main" \
        > /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && \
    apt-get install -y librealsense2-utils ros-humble-realsense2-camera && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --only-upgrade -y \
        ros-humble-rclcpp-action \
        ros-humble-moveit* \
    && rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update

RUN rosdep update && \
    rosdep install isaac_ros_nvblox

RUN mkdir -p /usr/include && \
    ln -sf /usr/local/lib/python3.10/dist-packages/numpy/core/include/numpy /usr/include/numpy

RUN mkdir -p /home/admin/.cache/torch_extensions && \
    mkdir -p /home/admin/.cache/warp && \
    rm -rf /home/admin/.cache/torch_extensions/py310_cu128

COPY rviz_glsl150/ /opt/ros/humble/share/rviz_rendering/ogre_media/materials/glsl150/
COPY rviz_scripts150/ /opt/ros/humble/share/rviz_rendering/ogre_media/materials/scripts150/

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
        --index-url https://download.pytorch.org/whl/cu124 \
        torch torchvision && \
    python3 -m pip install --no-cache-dir --break-system-packages \
        "numpy<2" \
        opencv-python \
        "transforms3d>=0.4.1" \
        pyyaml \
        ultralytics \
        pyapriltags
