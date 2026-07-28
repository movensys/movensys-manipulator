# Movensys Manipulator

ROS 2 packages, Docker compose configs, and end-to-end examples for driving
[Dobot CR3A/CR5A](https://www.movensys.com/) manipulators with the
[WMX R2](https://github.com/movensys/wmx-r2) motion control stack,
MoveIt 2 / NVIDIA Isaac cuMotion planning, and perception via Nvblox, YOLO,
and AprilTag.

## Overview

This repository is a ROS 2 workspace package set that pairs the WMX motion
engine with a planning and perception stack on top of either Gazebo or
[NVIDIA Isaac Sim](https://github.com/movensys/movensys-simulation). It
supports three execution modes for every example:

- **Simulation** — pure simulation (Isaac Sim or Gazebo), no hardware
- **SIL** — simulation-in-the-loop, simulator visuals + real WMX runtime
- **Real** — control of the real robot via WMX over EtherCAT

The included examples cover trajectory planning (MoveIt 2 OMPL or Isaac
cuMotion), AprilTag-driven pick-and-place, Nvblox obstacle avoidance, YOLO
object detection, and AprilTag with Nvblox combined.

## Repository Layout

```
.
├── movensys_manipulator_description/      # URDF, meshes, Gazebo & RViz launch
├── movensys_manipulator_moveit_config/    # MoveIt 2 config, sim bridge, services
├── movensys_manipulator_isaac_ros_config/ # Isaac cuMotion + Isaac ROS launch
├── movensys_manipulator_perception/       # Nvblox, YOLO, AprilTag pipelines
├── docker/                                # Compose stacks and Dockerfiles
├── doc/                                   # Step-by-step example walkthroughs
└── tools/                                 # Data collection and training utilities
```

## Packages

| Package | Description |
|---------|-------------|
| `movensys_manipulator_description`      | URDF/xacro, meshes, and RViz/Gazebo bring-up for the Dobot CR3A/CR5A arms |
| `movensys_manipulator_moveit_config`    | MoveIt 2 configuration, the `moveit2_api` service node (`/wmx/moveit2/*`), the simulator bridge, and the demo launches (trajectory, AprilTag pick-and-place, obstacle avoidance, YOLO) |
| `movensys_manipulator_isaac_ros_config` | NVIDIA Isaac ROS launches — Isaac cuMotion planning plus Isaac AprilTag and Nvblox perception bridges |
| `movensys_manipulator_perception`       | Perception nodes: AprilTag detection and YOLO OBB cube/dice detectors, with camera bring-up |

Planned trajectories execute on the servos through the
[WMX R2](https://github.com/movensys/wmx-r2) `joint_trajectory_controller`
over EtherCAT; see that repository for the underlying motion-control nodes.

## Examples

Each example has a dedicated walkthrough under [`doc/`](doc/). The numbered
prefix selects the scenario; the trailing letter selects the execution mode.

| #  | Scenario                          | Simulation                                            | SIL                                            | Real                                            |
|----|-----------------------------------|-------------------------------------------------------|------------------------------------------------|-------------------------------------------------|
| 3  | Trajectory planning               | [3a](doc/3a_trajectory_simulation.md)                 | [3b](doc/3b_trajectory_hil.md)                 | [3c](doc/3c_trajectory_real.md)                 |
| 4  | AprilTag pick-and-place           | [4a](doc/4a_apriltag_simulation.md)                   | [4b](doc/4b_apriltag_hil.md)                   | [4c](doc/4c_apriltag_real.md)                   |
| 5  | Nvblox obstacle avoidance         | [5a](doc/5a_nvblox_simulation.md)                     | [5b](doc/5b_nvblox_hil.md)                     | [5c](doc/5c_nvblox_real.md)                     |
| 6  | YOLO object detection             | [6a](doc/6a_yolo_simulation.md)                       | [6b](doc/6b_yolo_hil.md)                       | [6c](doc/6c_yolo_real.md)                       |
| 7  | AprilTag + Nvblox                 | [7a](doc/7a_apriltag_nvblox_simulation.md)            | [7b](doc/7b_apriltag_nvblox_hil.md)            | [7c](doc/7c_apriltag_nvblox_real.md)            |
| 8  | VLA application                   | [8a](doc/8a_vla_simulation.md)                        | —                                              | —                                               |

A ROS 2 API example (`doc/3d_api_example.md`) and host-setup guides
(`doc/1_setup.md`, `doc/2_docker.md`) are also provided. RGB recording and
video conversion commands are in [`doc/8_recording.md`](doc/8_recording.md).

## Requirements

- Ubuntu 22.04 or 24.04
- ROS 2 Humble or Jazzy
- Docker with `docker compose` (the workspace runs inside a container)
- An NVIDIA GPU and Isaac ROS prerequisites for the `isaac-ros_*` images
  (see the [Isaac ROS getting-started guide](https://nvidia-isaac-ros.github.io/getting_started/index.html))
- The [`movensys-simulation`](https://github.com/movensys/movensys-simulation) repo for Isaac Sim scenes

## Quick Start

### 1. Configure the host environment

Add the following to your `~/.bashrc` (adjust the variables for your setup):

```
export ROS_DOMAIN_ID=73                         # any free domain id
export ROS_DISTRO=jazzy                         # {jazzy, humble}
export MOVENSYS_ROS_VERSION=isaac-ros_4.1       # {intel-xpu, isaac-ros_4.1, isaac-ros_3.2, general}
export CPU_ARCH=amd64                           # {amd64, arm64}
export MANIPULATOR_MODEL=dobot_cr3a             # {dobot_cr3a, dobot_cr5a}

export HOST_USER_UID=$(id -u)
export HOST_USER_GID=$(id -g)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

export MOVENSYS_MANIPULATOR_PACKAGES=~/workspaces/movensys_ws/src/movensys-manipulator
export ISAAC_ROS_WS=~/workspaces/isaac_ros-dev

mros() {
  if [ $# -eq 0 ]; then
    docker exec -it -u admin movensys_manipulator_container \
      bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/admin/workspaces/movensys_ws/install/setup.bash && exec bash -i'
  else
    docker exec -it -u admin movensys_manipulator_container \
      bash -lc "source /opt/ros/\${ROS_DISTRO}/setup.bash && source /home/admin/workspaces/movensys_ws/install/setup.bash && $*"
  fi
}
```

```
xhost +local:docker
source ~/.bashrc
```

### 2. Raise CycloneDDS socket buffers

```
sudo tee /etc/sysctl.d/99-network-buffers.conf << 'EOF'
net.core.rmem_max=67108864
net.core.rmem_default=67108864
net.core.wmem_max=67108864
net.core.wmem_default=67108864
EOF

sudo sysctl -p /etc/sysctl.d/99-network-buffers.conf
```

### 3. Clone the repository

```
mkdir -p ~/workspaces/movensys_ws/src
cd ~/workspaces/movensys_ws/src
git clone https://github.com/movensys/movensys-manipulator.git
```

### 4. Build and start the container

```
cd ${MOVENSYS_MANIPULATOR_PACKAGES}/docker
docker compose -f ${MOVENSYS_ROS_VERSION}.yaml -f movensys_manipulator.${CPU_ARCH}.yaml down
docker compose -f ${MOVENSYS_ROS_VERSION}.yaml -f movensys_manipulator.${CPU_ARCH}.yaml build
docker compose -f ${MOVENSYS_ROS_VERSION}.yaml -f movensys_manipulator.${CPU_ARCH}.yaml up -d
```

Verify the container is healthy:

```
docker logs movensys_manipulator_container -f
```

Enter the container shell:

```
mros
```

### 5. Run an example

Pick a walkthrough from the table above (for instance,
[`doc/3a_trajectory_simulation.md`](doc/3a_trajectory_simulation.md)) and
follow its steps. A typical run is: open the matching scene from
`movensys-simulation` in Isaac Sim (or launch Gazebo), start the sim bridge,
launch MoveIt 2 or cuMotion, and trigger the trajectory test.

## Related Repositories

- [wmx-r2](https://github.com/movensys/wmx-r2) — Core WMX R2 motion control packages
- [wmx-r2-doc](https://github.com/movensys/wmx-r2-doc) — Documentation site for the WMX R2 stack
- [movensys-simulation](https://github.com/movensys/movensys-simulation) — Isaac Sim USD scenes used by the examples here
- [movensys-intelligence](https://github.com/movensys/movensys-intelligence) — VLM-driven task planning

## License

Released under the MIT License. See [`LICENSE.txt`](LICENSE.txt) for details.
