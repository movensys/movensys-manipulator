# CR3A real-time obstacle avoidance with cuRoboV2 and WMX R2

Demo code for a Dobot CR3A driven by WMX R2 on an NVIDIA Jetson Thor:
a fixed RealSense camera builds a cuRoboV2 voxel map, the cuRoboV2 MPC replans
at 20 Hz around obstacles, an optional input shaper smooths the command, and the
joint trajectory is streamed to the servos through the WMX3 CyclicBuffer.

This folder is a **snapshot of the code as it ran on Jetson Thor** (WMX simulator
and the real CR3A over EtherCAT, Oct-Nov 2026; last real-robot session 2026-11-02).
It is plain Python and shell, not ROS 2 packages; `COLCON_IGNORE` keeps colcon out.
It has not been run from this repository location yet.

```
camera -> cuRoboV2 Mapper -> cuRoboV2 MPC (20 Hz) -> safety checks -> input shaping
       -> JointTrajectory -> JointTrajectoryController (ros2_control)
       -> WmxSystemHardware -> WMX3 CyclicBuffer (1 ms) -> CR3A
```

## Contents

| Folder | What |
|---|---|
| `cr3a_gui/` | PyQt5 GUI (`cr3a_gui.py`) and its server (`rt_server.py`, `gui_controller.py`): jog through MoveIt Servo, Cartesian goals through cuRobo IK, real-time avoidance on/off, speed scale, input shaping panel, camera calibration and map check, 3D view. `run_gui.sh` starts it, `wmx_r2.sh` starts WMX R2. |
| `cr3a_rt_shaping/` | Real-time avoidance engine used by the GUI server: Mapper (`cr3a_mapper.py`), MPC (`rt_mpc.py`), safety and motion logic, input shaping (`rt_shaping.py`, `shaping.yaml`), streaming (`rt_stream.py`), tool model (`robot_tool/`). `rt_run.py` runs the same engine on its own (ideal robot or WMX simulator). |
| `cr3a_sim/` | CR3A robot model for cuRobo (Dobot URDF and meshes, `cr3a_curobo.yml`), read at runtime by `cr3a_rt_shaping`, plus one-shot planning scripts. |
| `wmx_r2_docker/` | `Dockerfile` and `run.sh` for the `wmx_r2` container (CUDA 13, ROS 2 Jazzy, WMX R2, PyTorch, cuRoboV2; arm64 / Jetson Thor), `wmx_mode.sh` (WMX simulator <-> EtherCAT). |
| `curobo_v2_patch/` | Our changes to cuRoboV2, as one diff against NVlabs/curobo `78fd485`. |

## Dependencies

| What | Where it goes on Thor | Used for |
|---|---|---|
| WMX3 3.7 (host install, `/opt/wmx3`) | host, mounted into the container | motion engine, EtherCAT |
| movensys/wmx-r2 branch `feature/cyclic-buffer-stream-on-devel` | `~/workspaces/movensys_ws/devel_cyclic_ws/src/wmx-r2` | ros2_control position streaming into the CyclicBuffer; `cr3a_gui/wmx_r2.sh` launches it with `stream_position:=true` |
| movensys/wmx-r2 (`devel`) and movensys/movensys-manipulator | `~/workspaces/movensys_ws/src/` | `wmx_r2_message` (encoder display), CR3A description and MoveIt config (jog via MoveIt Servo) |
| NVlabs/curobo `78fd485` + `curobo_v2_patch/` | image `/opt/curobo_v2` (unpatched) and `~/workspaces/movensys_ws/curobo_v2_shaping` (patched) | `run_gui.sh` puts the patched copy first on `PYTHONPATH` |
| Intel RealSense (fixed camera) | USB | camera map |

The cuRoboV2 patch (4 commits, 2 files, +58 lines):

- `fa3a577` input shaper hook: kinematics/collision on shaped y, cspace costs on u
- `77f9373` reference tracking cost (on shaped y when input shaping is on)
- `07cb29a` reference tracking: optional weak tracking on pre-shaping u
- `2283265` input shaper: exact float32 elementwise sum instead of matmul (TF32 rounding broke the MPC)

## Setup on a Jetson Thor

The scripts assume this layout (host `~/workspaces` = container `/home/admin/workspaces`).

```bash
# from this folder
mkdir -p ~/workspaces/movensys_ws/src ~/workspaces/wmx_r2_logs
cp -r cr3a_gui cr3a_rt_shaping cr3a_sim ~/workspaces/movensys_ws/
cp -r wmx_r2_docker ~/workspaces/

# patched cuRoboV2 copy
git clone https://github.com/NVlabs/curobo.git ~/workspaces/movensys_ws/curobo_v2_shaping
git -C ~/workspaces/movensys_ws/curobo_v2_shaping checkout 78fd485fa82d9b9a063fb4985e371814587e666a
git -C ~/workspaces/movensys_ws/curobo_v2_shaping apply "$PWD/curobo_v2_patch/curobo_v2_shaping.patch"

# WMX R2 with CyclicBuffer streaming (its own workspace)
git clone -b feature/cyclic-buffer-stream-on-devel https://github.com/movensys/wmx-r2.git \
    ~/workspaces/movensys_ws/devel_cyclic_ws/src/wmx-r2
# WMX R2 messages and the CR3A MoveIt config
git clone -b devel https://github.com/movensys/wmx-r2.git ~/workspaces/movensys_ws/src/wmx-r2
git clone -b devel https://github.com/movensys/movensys-manipulator.git \
    ~/workspaces/movensys_ws/src/movensys-manipulator

# container (WMX3 must already be installed on the host)
cd ~/workspaces/wmx_r2_docker
./run.sh build && ./run.sh create
./run.sh colcon                       # builds movensys_ws/src
docker exec -u admin -w /home/admin/workspaces/movensys_ws/devel_cyclic_ws wmx_r2 \
    bash -c "source /opt/wmx_r2_env.sh && colcon build --symlink-install"
docker exec -u admin wmx_r2 python3 -m pip install --user --break-system-packages \
    pyqtgraph==0.14.0 PyOpenGL==3.1.10 colorama==0.4.6     # GUI 3D view
```

Check the WMX mode before starting anything: `~/workspaces/wmx_r2_docker/wmx_mode.sh status`
(`simu` = WMX simulator, `ethercat` = real robot).

## Run

```bash
# GUI on the Thor screen (host terminal)
~/workspaces/movensys_ws/cr3a_gui/run_gui.sh

# engine alone, no ROS or WMX (inside the container, in cr3a_rt_shaping/)
python3 rt_run.py --robot ideal --scenario moving3t --out /tmp/x
python3 -m tools.rt_tools ideal /tmp/x

# engine on the WMX simulator with a report (host)
~/workspaces/movensys_ws/cr3a_rt_shaping/tools/rt_demo.sh
```

## Known limitations

- Comments, GUI labels and `cr3a_rt_shaping/README.md` are mostly in Korean.
- Paths assume the layout above, container user `admin` and ROS domain 70.
  The GUI log folder defaults to `/home/mic-735d/workspaces/wmx_r2_logs`;
  on another machine set `CR3A_LOG_ROOT=$HOME/workspaces/wmx_r2_logs`.
- Site files are not included; the GUI creates them: `camera_pose.json`
  (camera calibration), `poses.json` (saved poses), `last_encoders.json`.
- Jog goes through MoveIt Servo with the MoveIt config from movensys-manipulator;
  it does not use the camera map or the planner's safety checks.
- The code was not checked against this repository's `ament_flake8` settings,
  so a pull request would show lint findings.
