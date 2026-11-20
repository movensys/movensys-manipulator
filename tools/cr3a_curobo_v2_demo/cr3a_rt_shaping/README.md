# cr3a_rt_shaping — CR3A 실시간 장애물 회피 + 입력성형 (cuRoboV2 + WMX R2)

카메라 1 대 → cuRoboV2 Mapper 지도 → MPC (20 Hz) → 명령 궤적 → 안전 검사 → 입력성형 → JointTrajectory → R2 CyclicBuffer → WMX.
GUI 서버 (`../cr3a_gui/rt_server.py`·`gui_controller.py`) 가 이 엔진 모듈을 불러 쓰고, `rt_run.py` 는 같은 엔진을 혼자 돌린다
(이상적 모드 · WMX 시뮬레이터). Thor: `~/workspaces/movensys_ws/cr3a_rt_shaping` (컨테이너 `wmx_r2` 안 `/home/admin/workspaces/movensys_ws/…`).
cuRobo 는 사본 `~/workspaces/movensys_ws/curobo_v2_shaping` 을 PYTHONPATH 로. 경위는 `../작업기록/` (입력성형·디버깅 진행사항).

## 실행 (컨테이너 안, 이 폴더에서)

```bash
python3 rt_run.py --robot ideal --scenario moving3t --out /tmp/x    # ROS·WMX 없이 (로봇 = 명령 그대로, 계산 시계 → 같은 입력이면 같은 기록)
python3 rt_run.py --robot ideal --scenario goal_wait --legs 4 --set dist.act_new=0.10 --out /tmp/y    # 시나리오·설정 바꿔 비교
python3 -m tools.rt_tools ideal /tmp/x                              # 요약: 다리, 관절 이동, 안전 검사, 실제 형상 최소 거리
python3 -m tools.rt_tools compare /tmp/x /tmp/y                     # 두 기록이 같은가 (리팩터링 검증 — 이상적 모드는 비트까지 같아야)
python3 -m pytest                                                   # 단위 시험 (pytest.ini; cuRobo·ROS 가 없으면 그 시험은 건너뜀)
```

WMX 시뮬레이터로 끝까지 + 보고서 (★ Thor 호스트에서 ★):

```bash
tools/rt_demo.sh                                     # → ~/workspaces/wmx_r2_logs/is_carrot_cfg_<시각>/report.html
SHAPER=ZVD SHAPER_HZ="8 15" tools/rt_demo.sh         # 성형기 바꾸기 (shaping.yaml 보다 우선) · SCENARIO=… · RT_ARGS="--legs 4"
tools/wmx_batch.sh                                   # 성형기 여러 개를 차례로
```

`rt_demo.sh` 의 안전장치·R2 켜기/끄기·WMX 축 로그는 `../cr3a_gui/wmx_r2.sh up/down` (GUI 와 같은 한 벌): 켜진 WMX 플랫폼이 simu 하나뿐이고
서보를 켠 엔진이 시뮬레이터일 때만 R2 를 띄운다 (실제 로봇 보호 — EtherCAT 으로는 돌리지 않음). R2 디버그 기록은 `R2_TRACE=1` 일 때만.

## 핵심 모듈

| 파일 | 하는 일 |
|---|---|
| `rt_config.py` | 모든 설정값과 근거 (반응 거리, 지도, 계획, MPC, 명령 궤적, 메시지 시간표, 안전 검사, 임무) |
| `rt_run.py` | `Controller`: 주기 (영상 → MPC → 명령 궤적 → 안전 검사 → 메시지), 다리 (계획·따라가기·도착·막힘·비키기), 기록 저장 |
| `rt_world.py` | `World`: 합성 깊이 → 지도, 전역 계획기 (MotionPlanner), 실제 거리 검사 (`clearances`), 충돌 바닥 (`floor_z`) |
| `cr3a_mapper.py` | 카메라 지도: 팔 지우기, 경계 채우기, 바닥 띠·받침 자리 빼기 |
| `rt_mpc.py` | 당근 MPC (기본, `Config.style='carrot'`) — 전역 경로 따라가기 |
| `rt_cmpc.py` | 성형 MPC (C 방식, `style='cmpc'`) — cuRobo 가 성형된 경로를 최적화 (다음 단계) |
| `rt_motion.py` · `rt_stream.py` | 명령 궤적 (numpy) · JointTrajectory 메시지 시간표 + 입력성형 기억 (실행된 u) |
| `rt_shaping.py` · `shaping.yaml` | 입력성형기 (ZV·ZVD·EI·ETM4·ETM5, 1 ms 격자) · `rt_run.py` 의 기본 성형 설정 (GUI 는 `../cr3a_gui/shaper_def.py`) |
| `rt_robot.py` | 이상적 로봇 + 계산 시계 / ROS 로봇 (R2 JTC) + 벽시계 |
| `rt_scenarios.py` | 물체 시나리오 (`moving3t` 기본 · `moving5` · `none` · `goal_*` · `hold_*`) |
| `rt_model.py` · `robot_tool/` | 로봇 형상: `tool` (기본, 끝단 브래킷·D455·그리퍼 — URDF·메시·cuRobo 설정·덮개 구) / `bare` (옛 기록) |
| `depth_sim.py` | 합성 깊이 (로봇 메시 + 상자, warp) |
| `rt_records.py` | 결과 폴더 읽기 (numpy 만): 실행된 u, WMX 축 로그, 성형기 목록·비상정지 구간·충돌 바닥·실제 카메라 (GUI 서버 기록) — 도구가 씀 |

`robot_tool/` 은 모델 파일과 그것을 만드는 순서 (SolidWorks `cad/sw_extract.py`·`sw_fix_missing.py` → `make_tool_meshes.py` →
`cover_spheres.py` → `build_tool_model.py`) 만 둔다.

## tools/ (실행 중인 회피·GUI 서버는 안 씀 — 이 폴더에서 `python3 -m tools.<이름>`)

| 도구 | 하는 일 |
|---|---|
| `rt_report` (+ `rt_viewer_template.html`) | WMX 실행 보고서: 3D 재생 + 그래프 → `report.html`, 수치 → `summary.json` |
| `rt_tools` | 기록 분석: `compare` `ideal` `contacts` `contacts_sum` `shaping` `table` `contact` `arrival` `peaks` `messages` `scenario` `replay` |
| `rt_geom` | 실제 형상 거리 (로봇 메시 ↔ 상자·바닥) — 닿음 판정의 기준 |
| `motion_quality` · `wmx_cmp` | 다리마다 움직임 품질 · WMX 실행 비교 표 (+ 한 주기 계산 시간) |
| `fake_wmx_from_ideal` | 이상적 기록으로 가짜 WMX 축 로그 (보고서 계산 시험용) |
| `check_cmpc` | 성형 MPC 단독 점검 |
| `rt_demo.sh` · `wmx_batch.sh` | WMX 시뮬레이터 실행 + 보고서 (Thor 호스트) |

닿음 셈 (사용자 기준): 로봇이 움직여 닿음 + 멈춘 로봇에 물체가 와서 닿음 (10-16 부터). 카메라가 못 본 물체 (못 보는 면·팔 지우기) 는 셈 안 함.
실제 카메라 기록은 기록의 기둥·시나리오 물체가 가상이라 물체 거리·닿음은 셀 수 없음 (바닥 = 테이블 윗면만).

## 기록 (결과 폴더)

| 파일 | 내용 |
|---|---|
| `rt_run.json` | settings (설정 전체 `config`, 장면, 모델) · cycles (MPC 주기) · frames (영상) · events · legs · newvox (지도 칸) · sent_traj (보낸 명령 점) |
| `u_exec.npz` | 실행된 성형 전 명령 u (1 ms, 세그먼트별). GUI 서버는 성형기를 바꾸면 앞 성형기 것을 `u_exec_k<i>.npz` 로 (목록 = `kernels`) |
| `wmx_axis.csv` · `r2_launch.log` | (WMX) 엔진 축 로그 1 ms · R2 출력 |
| `report.html` · `summary.json` | `tools.rt_report` 결과 |

GUI 서버 기록에는 `kernels` (성형기·바꾼 시각) · `estop_windows` (비상정지 감속 — 성형 없음) · `settings.camera` (real = 실제 카메라) ·
`settings.config.map.table_z` (테이블 윗면 → 충돌 바닥) 가 더 있다. 보고서·도구는 `rt_records` 로 이것을 따른다.

## 실행 비교

- 같은 코드인가: `python3 -m tools.rt_tools compare <A> <B>` (이상적 모드 4 시나리오 기준 기록과 비트까지)
- 닿음·성형 탓: `python3 -m tools.rt_tools contacts <폴더>…`, 설정별 묶음 `contacts_sum`
- WMX 실행: `python3 -m tools.rt_report <폴더>` → `report.html`, 여러 실행 표 `python3 -m tools.wmx_cmp <폴더>…`
- GUI 조작별 검증: `../cr3a_gui` 에서 `PYTHONPATH=../cr3a_rt_shaping python3 -m tools.wmx_check <폴더>`

## archive/ (끝난 일 — 지우지 않고 보관, 경로가 바뀌어 그대로는 안 돎)

- `probe_*.py` — 10-14 성형 MPC (`rt_cmpc`) 진단
- `dev/` — 10-15 ~ 10-17 조사 스크립트 (작업기록 5-21 ~ 5-24). `cyc_time.py` 는 `tools/wmx_cmp.py` 에 합침
- `robot_tool/` — 공구 모델 만들 때의 일회성 확인 (`check_tool_poses`·`make_moving3t`·`tool_render`, 옛 고체만 추출 `cad/`)
- 10-16 WMX 실행 결과 (`results/`) 와 그 목록 페이지 `rt_index.py` 는 `../보고서/runs_20261016/` 으로
