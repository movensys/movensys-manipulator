"""CR3A 실시간 회피의 모든 설정값 (한곳). 값 옆에 근거 — 사용자 결정 / 실측 / 계산. 자세한 경위: 작업기록 wmx_r2_작업기록.md 12장.

바꿀 때: 사용자 결정 값(거리·차단값 등)은 사용자에게 묻는다. 실측으로 고른 값은 근거의 시험을 다시 해 본다.
"""
import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


@dataclass(frozen=True)
class Distances:
    """반응·지우기 거리 (m) — 사용자 결정 2026-10-13, [2026-10-16 바꿈] 사용자 "정지 및 움직이는 물체 모두 움직이는 물체처럼 다뤄.
    MPC 피하는 거리도 6cm로 (7->6) … 팔 지우기 거리는 4cm". 그 전 값 (10-13 ~ 10-16 실행): 새 물체 7 cm, 정지·바닥 2 cm, 팔 지우기 5 cm.
    [2026-10-27 R-1] 그 전 방식 (정지·새 물체를 나누는 지도) 되돌리기는 뺌 — 지도는 구분 없음 (cr3a_mapper 머리말)."""
    act_new: float = 0.06       # 물체에 MPC 가 반응하는 거리 (충돌 구 겉면 기준). 팔 지우기 + 2 cm (보이는 띠): 지우기 안으로 들어온 물체는
                                #  카메라 지도에서 지워지므로 그 전에 반응해야 함 (Mirobot 반응 3 cm / 지우기 4 cm 에서 손을 끝내 못 봄)
    act_static: float = 0.06    # 정지 물체에 반응하는 거리. [2026-10-16] 움직이는 물체와 같게 (구분 없는 지도 — 따로 못 바꿈, Config 검사).
                                #  그 전 2 cm (1회성 계획과 같음). 쓰는 곳: 마지막 접근 판정 (rt_run._command_carrot), 기록 settings
    mask: float = 0.04          # 팔 지우기: 로봇 충돌 구 겉면에서 이 안의 깊이 점은 지움. [2026-10-16] 5 → 4 cm (Mirobot 실물 10-07 에 영상으로
                                #  고른 값). 5 cm = 10-13 CR3A 실제 로봇 보정·시각 오차 여유로 정한 값 (시뮬레이션으로는 실제 오차를 못 잼)
    # 바닥 반응 거리 (MPC·계획기의 바닥 상자). [2026-10-16] 정지 물체는 6 cm 가 됐지만 바닥은 2 cm 그대로 (10-13 사용자 결정 값): 바닥은
    #  카메라로 보는 물체가 아니라 로봇이 서 있는 평면이고, 받침 (base_link) 충돌 구 아래 끝이 z 0.04, Link1 은 z 0.05 라 바닥 반응이
    #  3.5 cm 를 넘으면 로봇 자기 아랫몸이 늘 반응 거리 안 → 지울 수 없는 비용으로 계획이 실패함 (Mirobot step2_make_xrdf 와 같은 일)
    act_floor: float = 0.02


# [2026-10-27 D-2 독립 검토] 바닥 반응 윗끝 (충돌 바닥 높이 + act_floor) 의 한계 = 위 act_floor 설명의 3.5 cm: 넘으면 받침·Link1 충돌 구가
#  늘 바닥 반응 거리 안 → 지울 수 없는 비용으로 계획이 모두 실패. 테이블 높이 (MapCfg.table_z) 로 바닥을 올릴 수 있는 한도
#  (act_floor 2 cm 이면 테이블 +1.5 cm 까지 — cr3a_gui/rt_server.py 는 넘으면 테이블 높이를 안 씀, camera_fit 판정은 '나쁨')
FLOOR_REACT_TOP_MAX_M = 0.035


@dataclass(frozen=True)
class MapCfg:
    center: Tuple[float, float, float] = (0.20, 0.0, 0.38)   # 지도 격자 (받침 기준 m): x −0.45..0.85, y ±0.65, z −0.02..0.78
    size: Tuple[float, float, float] = (1.30, 1.30, 0.80)
    voxel: float = 0.01
    cam_fps: float = 15.0       # R2 realsense 640x480x15 (카메라 1대 — 사용자 결정, 실제 로봇과 같게)
    cam_max_lag_s: float = 0.5  # (실시간) 영상 차례가 이만큼 넘게 밀렸으면 밀린 영상은 건너뛰고 지금부터 다시 셈
    # 지도는 정지·새 물체를 나누지 않음 [2026-10-16 사용자 결정 "정지 및 움직이는 물체 모두 움직이는 물체처럼 … 안 보이는 뒷면 처리도
    #  따로 하지말고 보이는 부분만"]: 보이는 모든 칸 = 물체 지도 — 켤 때 배경을 '정지' 로 두지 않고, 가려진 뒷면 채우기·정지 거리 변환 없음.
    #  팔 지우기 경계 채우기·성형 여유 보존의 '움직이는 물체' 규칙이 기둥에도 똑같이. [2026-10-27 R-1] 10-13 ~ 10-16 방식 (켤 때 1 초 배경 =
    #  정지 물체: 뒷면까지 단단함, 2 cm 반응 / 그 앞에 나타난 것 = 새 물체) 되돌리기 (map.uniform False) 는 뺌
    # [2026-10-16] 받침 자리 빼기 (받침 = 움직이지 않는 로봇 자기 몸): 이 상자 (받침 기준 x0 y0 z0 x1 y1 z1, m) 안의 깊이 점은
    #  지도에 안 넣음 (바닥 띠 빼기와 같은 방식). 팔 지우기를 4 cm 로 줄이자 받침 아랫단 (충돌 구가 z 0.04 위에만 있음) 이 지우기 밖으로 남아
    #  바닥 바로 위 칸 14 개가 됐고, 구분 없는 지도에서는 이 칸에도 경계 채우기가 걸려 받침·Link1 둘레가 막힘 → 출발 자세 '충돌'
    #  (계획 전부 실패, is_dev_tool/ideal_uni6_live_20261016_174033). 값 = Dobot base_link 메시 범위 (x −0.065~0.110, y ±0.066,
    #  z 0~0.065) + 2 cm
    base_exclude: Tuple[float, ...] = (-0.085, -0.086, -0.05, 0.130, 0.086, 0.085)
    # [2026-10-26 사용자 결정 "받침 둘레 케이블 자리 (책상 높이) 를 지도에서 빼고"] 받침 축에서 반지름 r 안 · 높이 z 아래 (r, z, m) 의 깊이 점은
    #  지도에 안 넣음. 실제 카메라에서 받침 옆 책상 위 케이블 (받침에서 9~17 cm, 높이 0~4 cm) 이 받침 빼기 상자 밖에 남아 1 번 링크에서 4.5 cm
    #  → 경계 채우기가 늘 걸려 MPC 지도에서 받침 자리가 닿음 → 계획 거절·비키기 되풀이 (작업기록 5-29, 위 base_exclude 와 같은 일).
    #  None = 안 뺌 (시뮬레이션 기본 — 가상 장면엔 받침 옆에 아무것도 없음). 실제 카메라일 때 cr3a_gui/rt_server.py 가 넣는 값 = (0.20, 0.12):
    #  받침 오른쪽 뒤에서 올라오는 케이블이 높이 11 cm 까지 (깜빡임), 21 cm 부터는 뒤쪽 검은 상자 (실제 물체) 라 그 앞까지.
    #  ⚠️ 이 안에 들어온 낮은 물체는 안 보임
    base_ring: Optional[Tuple[float, float]] = None
    # (팔 지우기 경계 채우기 값의 방향 [2026-10-16 사용자 결정 (나)] 은 늘 켬 — cr3a_mapper._fill_mask_boundary 설명. [2026-10-27 R-1]
    #  10-13 방식 되돌리기 (map.fill_grad False) 는 뺌)
    # 바닥 띠: 테이블 윗면 (table_z) + 이 높이보다 낮은 깊이 점은 지도에 안 넣음 (cr3a_mapper mpc_floor_z_m — 바닥은 rt_world 가 MPC·계획기에
    #  상자로 따로 줌). 1.5 cm = 깊이 잡음 (σ 2 mm) 과 바닥 바로 위 칸 보간을 넘는 값 (2026-10-13). [2026-10-27 R-1] 같은 값을 두 곳 (rt_run
    #  FLOOR_BAND_M·MapSettings.mpc_floor_z_m 기본값) 에 두던 것을 여기 한곳으로 (GUI 서버의 지도 칸 보내기 띠도 같은 값)
    floor_band_m: float = 0.015
    # [2026-10-27 디버깅 D-2, 사용자 "받침 아래 구조물이 URDF 에 없음 … 카메라로 바닥면도 보정"] 테이블 윗면 높이 (받침 원점 기준 m). 실제 카메라면
    #  cr3a_gui/rt_server.py 가 카메라 보정이 잰 값 (camera_pose.json table_z_m) 을 넣음, 시뮬레이션은 0. 쓰는 곳: 지도 바닥 띠
    #  (cr3a_mapper mpc_floor_z_m = 이 값 + floor_band_m), 충돌 바닥 (rt_world: 바닥 상자 윗면·안전 검사 바닥 면 = max(이 값, 0) — 테이블이
    #  원점보다 낮으면 예전처럼 0 에 두어 안전 쪽, 높게 재지면 그만큼 올림. 올리는 한도 = FLOOR_REACT_TOP_MAX_M − dist.act_floor).
    #  [D-2 검토] 받침 둘레 케이블 빼기 (base_ring) 높이는 받침 기준 그대로 — 케이블은 받침 뒤 커넥터로 올라가 높이가 받침에 묶임
    table_z: float = 0.0


@dataclass(frozen=True)
class PlanCfg:
    """전역 계획 (cuRoboV2 MotionPlanner, rt_world) — 1회성 데모(cr3a_sim/plan_avoid.py)와 같은 설정."""
    ik_seeds: int = 32
    trajopt_seeds: int = 6
    attempts: int = 3           # plan_cspace max_attempts
    retry_s: float = 1.0        # 계획 실패면 그 자리를 지키며(MPC 는 계속 피함) 이만큼마다 다시
    # [2026-10-16 사용자 결정 "막혔을 때 돌아가는 로직은 가, 나 를 같이"] 그 전에는 경로를 3 s 못 따라가야 ('막힘', MpcCfg.stuck_s) 멈춘 뒤
    #  다시 계획 → 원래 경로를 지키려는 모습 (WMX 공구 모델 실행: 16 다리 69.2 s 중 38.1 s 가 계획보다 더 걸림, 다시 계획 2 번)
    #  (가) 경로 앞 막힘 감지: 주기마다 따라가는 전역 경로의 [기준 시각 + ahead_from_s, + ahead_to_s] 를 ahead_step_s 마다 실제 거리 지도
    #       (충돌 구 + 바닥 면) 로 보고, 거리 < ahead_block_m 인 점이 있으면 다시 계획. 바로 앞 ahead_from_s 는 뺌 (MPC·안전 검사가 피하는
    #       몫이고 다시 계획 (~0.2 s) 이 끝나기 전에 지나감)
    ahead_check: bool = True
    ahead_from_s: float = 0.2
    ahead_to_s: float = 2.0
    ahead_step_s: float = 0.1
    ahead_block_m: float = 0.0
    #  (나) 멈추지 않고 다시 계획: 로봇은 MPC 로 계속 움직이고 (지금 경로를 따라 피함), 계획이 끝나면 새 경로로 바꿈 (MPC 는 그대로, 새 경로의
    #       가장 가까운 점부터). 실시간 = 계획은 다른 스레드 (별도 CUDA 스트림, 지도는 요청 때 계획기에 복사), 이상적 모드 = 요청 때 풀고
    #       TaskCfg.ideal_plan_s 뒤에 씀. '막힘' (3 s) 도 이 방식. [2026-10-27 R-1] 그 전 방식 (멈춘 뒤 계획) 되돌리기 (live_replan False) 는 뺌
    replan_cooldown_s: float = 1.0  # 경로 앞 막힘 다시 계획 사이 최소 간격 (실패해도) — 지도 잡음·막힌 채 계속 다시 계획하지 않게
    #  멈추지 않는 다시 계획의 계획기 지도 = MPC 지도에서 지금 자세 충돌 구 겉면 live_clear_m 안 칸만 실제 거리로 (rt_world.World.plan_prep
    #  'start'). [2026-10-16 이상적 모드 moving3t] MPC 지도 그대로면 물체가 팔 가까이 있을 때 경계 채우기가 지금 자세 둘레 (구 안까지) 를 막아
    #  출발부터 '충돌' → 4·13 번 다리에서 다시 계획 15 번 실패, 다리 합 76.5 s, 로봇 탓 닿음 2 (is_dev_tool/ideal_uni6_live_20261016_175159).
    #  실제 거리 지도면 실패 0 이지만 팔 지우기 안 (안 보이는) 물체 쪽으로 경로가 나서 로봇 탓 닿음 4, 상자4 −49 mm (ideal_uni6_live2_20261016_175604).
    #  [2026-10-27 R-1] 그 두 지도를 고르는 설정 (live_map 'mpc'·'raw') 은 뺌
    live_clear_m: float = 0.015     # 실제 거리로 바꾸는 폭 (지도 1 cm 칸 1.5 개)
    # [2026-10-16 사용자 결정 (가) "장애물로 인해 일단 막혔는데 계속 다가오면 뒤로 물러서야 한다 … 후진하다가 경로 재탐색하여 돌아가는
    #  그런 느낌으로"] → [2026-10-17 사용자 결정 "그 자리에서 비키기로 바꿔 시험" (작업기록 5-23)] 비키기: 경로를 따라가다 막혀 서 있는데
    #  (기준 시각이 back_stall_s 동안 못 나아감) 물체가 다가오면 (지금 y 자리의 MPC 지도 거리 (back_smooth_s 최소) 가 그 값들의 back_window_s 안
    #  최대보다 back_approach_m 넘게 줄었거나, 0 아래 = 팔 지우기 안으로 들어옴 (경계 채우기)) → 경로 목표만 내려놓고 그 자리 (로봇 명령 자세) 를
    #  MPC 목표로 고정 (rt_mpc.Mpc.back) → MPC 충돌 비용이 물체 반대쪽으로 밀어냄 (지키기와 같음).
    #  그 전 (back_off False): 막혀 서 있으면 목표 (경로 0.4 s 앞 = 물체 쪽) 가 당겨 그 자리에서 버팀 → WMX '물체가 와서 닿음' 3 번 (작업기록 5-21)
    back_off: bool = True
    back_stall_s: float = 0.3       # WMX 상자4: 선 뒤 0.47 s 만에 닿음 → 0.3 s 안에 판단
    back_window_s: float = 0.5
    back_approach_m: float = 0.005  # 0.5 s 에 5 mm = 1 cm/s 넘게 다가옴 (지도 1 cm 칸 보간 잡음보다 큼)
    back_clear_s: float = 0.5
    #  거리 = 이 동안의 최소 (경계 채우기가 영상마다 켜졌다 꺼짐: 15 fps 영상 3 장). '다가옴' 의 최대값도 이렇게 고른 값들에서 (깜빡임을
    #  다가옴으로 안 봄 — 2026-10-17, 그 전 날 값 최대)
    back_smooth_s: float = 0.2
    #  끝: 안 다가옴 (MPC 지도 거리 ≥ 0) 이 back_clear_s 이어지고 지금 자리부터 경로 앞이 back_clear_hold_s 동안 이어 열림 → 경로 계속 (MPC 를
    #  로봇 명령 자리에서 새로 시작하고 그 자리와 가장 가까운 경로 점부터). 막혀 있으면 그 자리에서 기다리며 다시 계획 (쿨다운마다).
    #  비키는 중에도 경로 앞이 ahead_persist_s 이어 막혀 있으면 다시 계획 [2026-10-17 작업기록 5-24: 상자가 팔 지우기 경계에 걸치면 '다가옴'
    #  이 계속 다시 켜져 3 s 동안 다시 계획 없이 서 있었음 (WMX 6 번 다리)]. 비키는 중 새 경로가 오면 그 길로 (같은 방식으로 이음).
    #  back_max_s = 비키기 + 기다림 최대 (넘으면 다시 앞으로 = 보통 막힘 처리).
    #  [2026-10-17 사용자 결정 "비키기 + 기다림 줄이기"] 3 s — 비키기 WMX 에서 6 s 상한까지 기다린 다리가 있어 16 다리 83.6 s (작업기록 5-23)
    back_max_s: float = 3.0
    #  비킨 뒤 다시 앞으로 가려면 지금 자리부터 경로 앞이 이만큼 계속 열려 있어야 (맴도는 물체 앞으로 바로 나가 또 멈추지 않게)
    back_clear_hold_s: float = 0.5
    #  [2026-10-27 R-1] 물러나기 1~5 차 (2026-10-16 지나온 경로를 따라 물러나기: back_style 'path'·back_ref_stall·back_retreat_max_s — 이상적 6 변형
    #  에서 헤매는 다리 (800° 넘음) 13/96 vs 물러나기 끔 3/96) 와 그 사이 판단 되돌리기 (back_resync·back_resume_clear·back_replan_blocked·
    #  back_peak_smooth) 는 뺌 — 모두 지금 기본값 동작 (경위: 작업기록 5-22·5-23·5-24)
    # [2026-10-16 사용자 "너무 흔들 거리고 경로 품질이 좋아보이지 않는데 … 너무 불필요하게 움직이는거같은데" → ①②③ 모두, 기다림 0.5 초
    #  (작업기록 5-22)] 그 전 WMX: 관절 움직임 곧은 길의 3.3 배, 갔다 돌아옴 356 번, 새 경로 24 번 중 14 번이 앞 바꿈 뒤 2 s 안
    #  ① 경로 고집: 경로 앞 막힘이 ahead_persist_s 동안 이어질 때만 다시 계획 (움직이는 물체는 대개 금방 지나감, 0 = 그 전: 한 번 닿으면 바로).
    #     새 경로가 와도 그때 지금 경로 앞이 안 막혔으면 지금 경로 그대로 ('막힘 3 s' 로 요청한 것은 늘 받음)
    ahead_persist_s: float = 0.5
    #  ② 비킬 때 바로 다시 계획하지 않음 (비킨 뒤에도 경로 앞이 막혀 있으면 ① 로). 그 전: 물러날 때마다 요청 (WMX 26 번 중 15 번)
    #  ③ 계획 시작값: 곧은 선 + (멈추지 않는 다시 계획이면) 지금 경로의 남은 부분 + 넘어가기·돌아가기 변형 (J2·J1 가운데를 ±plan_bump_rad) →
    #     cuRobo 가 다 풀고, 성공한 후보 중 관절 움직임이 가장 적은 것 (rt_world.World._solve_seeded). 모두 실패면 cuRobo 기본 (plan_cspace).
    #     그 전: cuRobo 기본만 — 시작값 6 개가 모두 같은 곧은 선이고 (archive/dev/plan_seeds_probe.py), 실패하면 그래프 (무작위 길) 씨앗이라
    #     손목 (J5) 이 곧은 선에서 47~76° 꺾이는 길이 나왔음 (이상적 재현 is_dev_tool/mq/ideal_cur_213353).
    #  [2026-10-27 R-1] ①②③ 되돌리기 (keep_if_clear·back_replan·plan_seeds) 는 뺌
    plan_bump_rad: float = 0.35     # 넘어가기·돌아가기 변형의 가운데 크기 (20°): 기둥 (10 cm) 을 넘는 경로의 J2 차이 정도


@dataclass(frozen=True)
class MpcCfg:
    """cuRoboV2 ModelPredictiveControl — Mirobot 실시간 제어와 같은 방식 (rt_mpc 설명).
    2026-10-13 이상적 모드 비교 (물체 3개, 다리 5개): 이 방식 관절 총 이동 1317°, 한계 넘는 계획 0, 불가능 해 0
    vs '시간 일치' 5 Hz 방식 2732° / 12 / 26, V2 기본 간격 4111° / 66 / 86."""
    rate_hz: float = 20.0       # 주기 = optimization_dt (Mirobot 과 같음). 매듭 수는 V2 기본(16) → 내다봄 16·4·0.05 = 3.2 s
    interpolation_steps: int = 4    # V2 가 4 만 허용
    warm_iters: int = 100       # 주기마다 L-BFGS 반복 (Mirobot 값)
    cold_iters: int = 150
    lookahead_s: float = 0.4    # MPC 목표 = 전역 경로의 (기준 시각 + 이만큼) 점 (Mirobot 값)
    ahead_s: float = 0.3        # 기준 시각은 경로 위 지금 자리보다 이만큼 넘게 앞서지 않음
    follow_tol_rad: float = 0.15    # 경로에서 이보다 벗어나 있으면(피하는 중) 기준 시각을 멈춤
    stuck_s: float = 3.0        # 기준 시각이 이만큼 못 나아가면 "막힘" → 다시 계획 (목표가 막혔으면 기다림)
    # MPC 발산: 해를 못 찾으면서 (infeasible) 당근이 지금 명령에서 이만큼 (관절 최대, rad) 넘게 떨어지면 따르지 않고 직전 목표 유지.
    # 2026-10-14 WMX: 정상 실행 (rt_20261013_222849) 당근-명령 최대 0.83 rad, 발산 때 1~4 rad (관절 2 를 139·206·273° 로 →
    # 로봇이 목표 반대로 104° 까지 젖혀짐, 사용자 지적 rt_20261014_011934 9 번 다리)
    diverge_rad: float = 1.0
    # 안전 검사가 개입한 뒤 MPC 상태를 로봇 명령에 맞추는 방법 = 그 위치에서 MPC 새로 시작 (앞 해 버림 → 다음 주기 cold start, rt_run.cycle).
    # [2026-10-14 원인 찾음] 위치·속도·가속을 넣는 'full' 은 브레이크 중인 명령의 가속 (한계 7.78 rad/s², 명령 저크 500) 을 MPC (저크 한계
    # 62.8~77.8) 출발 상태로 넣어, 3.2 s 계획 끝이 관절 2 −256° 같은 자세로 망가지고 자기 출력 되먹임으로 발산했다 (WMX 실행 5 개의 발산 83 번
    # 중 81 번이 브레이크 뒤 0.6 s 안. rt_20261014_015202 13 번 다리 재현: 가속을 넣으면 발산, 빼면 정상). 이상적 모드 물체 시나리오 7 개 합
    # (full / 위치만 pos / restart): 발산 56 / 16 / 3 번, 발산 유지 주기 333 / 90 / 0, 당근-명령 최대 310 / 259 / 72°, 다리 시간 합
    # 466 / 434 / 428 s, 물체와 닿은 시나리오 5 / 5 / 2 개. 물체 없음 다리 16 개는 셋 다 거의 같음 (작업기록 13-10).
    # [2026-10-27 R-1] 'full'·'pos' 되돌리기 (mpc.resync) 는 뺌
    # [2026-11-02 디버깅 D-6] MPC 관절 위치 한계 비용이 켜지는 거리 (cuRoboV2 lbfgs_mpc.yml cspace activation_distance[0], 관절 범위의
    #  비율 — 한계를 범위 × 이 값만큼 안쪽으로 당겨 그 밖을 비용으로 막음, warp_bound_util.shrink_bounds_with_activation_distance).
    #  cuRobo 기본 0.01 = ±360° 관절 7.2°·J3 (±155°) 3.1° → 그 안쪽 목표에 못 닿음 (J6 300 → 355°: 352.8° 에서 '막힘 → 다시 계획' 6 번 뒤
    #  시간 초과) + 한계 근처에 서 있다 실시간 회피를 켜면 2~3° 저절로 움직임 (−355° → −352.6°). 실제 한계는 추종기 벽이 지킴
    #  (rt_motion.track_target: 한계 0.06° 앞으로 자르고 그 앞에서 설 수 있는 속도로만) → 0 (사용자 "그래 그럼 고쳐줘")
    pos_limit_activation: float = 0.0


@dataclass(frozen=True)
class CMpcCfg:
    """[2026-10-14] 성형된 경로를 최적화하는 시간표 MPC (rt_cmpc, C 방식) 설정."""
    # MPC 를 다시 푸는 주기 (한 번 = 매듭 2 개 실행). 메시지·안전 검사·지도는 MpcCfg.rate_hz (20 Hz) 그대로.
    #  [2026-10-14 실측, 물체 없이 A→B] 20 Hz (점 간격 6.25 ms)·10 Hz (12.5 ms) 는 cuRobo 최적화가 방향을 못 바꿔 목표에 못 감,
    #  5 Hz (25 ms) + 기준 궤적 따라가기만 도착 (성형 없음 3.2 s, ZVD 8 Hz 3.2 s, ZV 3.8 s — 경로 1.65 s)
    rate_hz: float = 5.0
    horizon_s: float = 3.2          # 내다보는 시간 = 매듭 수 × 매듭 간격 (5 Hz: 매듭 100 ms → 32 매듭)
    warm_iters: int = 100
    cold_iters: int = 150
    lookahead_s: float = 0.1        # 목표 = 전역 경로의 (기준 시각 + 이만큼) 점 (시간표 MPC: 2026-10-13 consistent 방식 값)
    ahead_s: float = 0.1            # 기준 시각은 경로 위 지금 자리보다 이만큼 넘게 앞서지 않음
    follow_tol_rad: float = 0.15    # 경로에서 이보다 벗어나 있으면 (피하는 중) 기준 시각을 멈춤
    stuck_s: float = 3.0            # 기준 시각이 이만큼 못 나아가면 "막힘"
    # 관절 목표 비용의 중간 점 가중 (lbfgs_mpc.yml cspace_non_terminal_weight_factor, V2 기본 0.05). 시간표 MPC 는 '내다봄 끝에만
    # 닿으면 되는' 목표라 0.05 면 느긋하게 감 (2026-10-14 실측: 물체 없이 A→B 6 s 동안 속도 한계의 13 %, 목표 80° 앞)
    nonterminal: float = 0.05
    bound_scale: float = 1.0        # 한계 (위치·속도·가속·저크) 비용 가중 배율 (V2 기본 1000·1000·1000·100)
    # [2026-10-14 진단] 목표가 멀면 (24°) 첫 풀기에서 궤적이 전혀 안 움직임 — 최적화 첫 걸음이 커서 한계 비용이 폭증해 선 탐색이
    #  모든 걸음을 거부하는 것으로 보임 (점 간격이 작을수록 같은 걸음의 가속이 1/dt² 로 커짐). 조정용:
    target_scale: float = 1.0       # 관절 목표 비용 가중 배율 (V2 기본 cspace_target_weight 1000)
    ls_scales: str = ''             # 선 탐색 걸음 배율 (V2 기본 '0,0.1,0.5,1'), 빈칸 = 기본
    # [2026-10-14] 시간 기준 궤적 따라가기: 상태 궤적 점 j 마다 기준 = 전역 경로의 (기준 시각 + j·점 간격) 점, 비용 track_w·|y − 기준|²
    #  (y = 성형된 궤적 → 성형 지연을 최적화기가 미리 보상). 0 = 끔 (목표 한 점 방식: 내다봄 끝에만 닿으면 되는 목표라 느긋함)
    track_w: float = 300.0          # (위 설명) 실측: 100 은 도착이 느림 (0.15° 에서 오래), 300 은 3.2 s 에 0.1° 안
    # 성형 전 u 도 (지연 없는) 경로를 약하게 따라가게 하는 가중. [2026-10-14 실측] y 비용만 걸면 성형기가 지우는 주파수 성분 (8 Hz 근처) 이
    #  u 에 섞여도 비용에 안 보여 그 방향이 막히지 않음 → 8 Hz 성형기 5 종 모두 u 가 크게 흔들림 (8+15 Hz 만 도착). 0 = 끔
    track_u_w: float = 0.0
    # MPC 모델의 저크 한계 (rad/s³, 0 = cr3a_curobo.yml max_jerk = 가속 × 10 = 62.8~77.8). [2026-10-14 진단] 시간표 MPC 는 매듭 간격
    #  25 ms (20 Hz) 라 궤적을 조금만 바꿔도 이 저크 한계를 넘어 비용이 폭증 → 방향을 못 바꾸는 것으로 의심. 명령 저크 한계 (MotionCfg.jerk
    #  500) 와 같게 해 보는 값
    jerk: float = 0.0
    force_shaper: bool = False      # (진단) 성형기가 1 칸 (항등) 이어도 cuRobo 성형 경로 (W 행렬) 를 지나게
    # [2026-10-15] 첫 실행 구간 저크 자유화 (작업기록 5-13). cuRobo 3 차 B-스플라인은 첫 매듭 구간의 제어점 4 개를 시작 상태 (위치·
    #  속도·가속) 로 다 정해 그 구간은 q₀ + v₀t + ½a₀t² (저크 0) → 그 안에서 되먹이면 MPC 가 아무것도 못 바꿈 (10-13 측정, 커널
    #  bspline_boundary_constraint.cuh 확인). 시작 상태를 한 매듭 앞 시각으로 되돌려 (등가속 역외삽) 주면 고정 구간 = 이미 지난 구간,
    #  실행하는 둘째 구간의 저크는 첫 자유 제어점이 정함 (CUDA 수정 없음) → 매듭 간격과 상관없이 rate_hz 마다 다시 계획.
    #  knot_s > 0 = 이 방식 (매듭 간격 knot_s, 점 간격 knot_s/4), 0 = 2026-10-14 방식 (한 주기 = 매듭 2 개)
    knot_s: float = 0.0


@dataclass(frozen=True)
class MotionCfg:
    """로봇에 보내는 명령 궤적 (rt_motion)."""
    # 저크 한계 (rad/s³). 목적: R2 1 ms 변환을 거친 엔진 1주기 가속이 maxAcc 를 넘지 않게 — 가속 |a| ≤ 7.78 에서 꺾임을
    # 10 ± 5 ms 3차 Hermite 로 다시 만든 최악 (2026-10-13 계산): J 77.8 → 7.95, 500 → 8.89, 1200 → 10.44, 계단 → 12.96.
    # Dobot 사양 아님. cr3a_curobo.yml 의 max_jerk (가속 × 10) 는 MPC 궤적에서 4° 벗어나 회피가 나빠져서 안 씀 (실측)
    jerk: float = 500.0
    # [2026-10-13 사용자 지적: 도착 때 목표를 살짝 지나쳤다 돌아옴] 명령 궤적의 멈춤 속도를 저크까지 넣어 계산 (rt_motion.stop_speed) +
    # 마지막 접근에서 명령이 목표를 넘지 않게 (rt_run.Controller._command_carrot, rt_motion.clamp_final). 물체 없이 다리 16 개 (이상적 모드):
    # 지나침 평균 0.33 → 0.20°, 최대 0.70 → 0.32°, 다리 시간 그대로 (2.12 s). [시도 후 뺌] 멈출 때 감속을 한계 × 0.8 로 두면 관절 하나
    # 계산으로는 지나침 0.05° 이하였지만, 실제로는 명령이 당근보다 더 늦어 기둥 쪽을 가로질러 안전 검사 → MPC 다시 맞춤 → MPC 발산 (당근이
    # 목표에서 286°) 으로 다리 평균 2.12 → 2.92 s. [2026-10-27 R-1] 끄는 되돌리기 (motion.final_clamp) 는 뺌
    # 마지막 접근 (위 목표 넘지 않기가 켜지는 때) 에만 멈춤 속도를 감속 한계 × 이 값으로 (rt_motion.track_target stop_decel). 남은 지나침
    # 0.2~0.3° 는 명령이 목표 지점에 들어가며 서는 동작의 몫 (관절 하나 계산: 0.8 이면 0.05° 이하). 경로 전체에 쓰면 기둥 근처에서
    # 명령이 당근보다 늦어 위 '시도 후 뺌' 문제가 생겨서 마지막 접근에만 (그때는 주변 반응 거리 안에 물체가 없음)
    final_stop_decel: float = 0.8
    step_s: float = 0.005       # 보내는 점 간격. 25 ms 점 사이를 JTC 가 5차식으로 이으며 꺾인 가속을 출렁여 (JTC 10.4) 5 ms 로
    sub: int = 5                # 점 사이 적분 단계 (1 ms)
    cycles: int = 2             # 한 메시지가 덮는 주기 수 (다음 메시지가 한 주기 늦어도 이어짐) + 그 뒤 멈춤 꼬리
    tail_dt: float = 0.01       # 멈춤 꼬리 점 간격


@dataclass(frozen=True)
class StreamCfg:
    """JointTrajectory 메시지 시간표 (rt_stream)."""
    lead_s: float = 0.05        # 계산 시작 = 메시지 시작(스탬프) − lead. 계산·발행이 이보다 짧아야 함 (MPC ~13 ms + 검사 ~10 ms)
    pre_s: float = 0.10         # 메시지 앞에 붙이는 앞 메시지 점의 범위 (Jazzy JTC 갈아타기 튐 방지: rt_stream 설명). 앞 메시지는 지난
                                # 스탬프부터라 평소 붙는 것은 한 주기 (5 ms 점 10 개), 안 보낸 주기 뒤에는 두 주기
    late_before_s: float = 0.01     # 계산 전 스탬프까지 이보다 덜 남았으면 다음 주기 격자로 미룸
    late_send_s: float = 0.003      # 보낼 때 스탬프까지 이보다 덜 남았으면 안 보냄 (앞 메시지 계속)


@dataclass(frozen=True)
class GuardCfg:
    """마지막 안전 검사 (rt_run.Controller._guard) — 사용자 결정 2026-10-13: 여유 거리 0 (실제 충돌만 막음)."""
    margin_m: float = 0.0       # 보낼 궤적(+꼬리)의 실제 거리 지도 최소 거리가 이보다 작으면 막음
    # 새 물체는 보낼 궤적(+멈춤 꼬리)이 이 안으로 들어가면 막음. [시험 후 0] 사용자 제안 2026-10-14 "로봇 몸 6 cm 에 닿으면 정지 후 동작"
    # (팔 지우기 5 cm 안은 안전 검사가 못 보므로 보이는 동안 멈추게) 을 0.06 으로 이상적 모드 7 개 시나리오에 돌려 봄: 닿은 시나리오
    # 4/7 (0 일 때 2/7), 다리 평균 6.6 s (5.1 s), 기둥 −8.8 mm (−3.8) — 너무 자주 멈춰 오히려 나빠서 끔 (작업기록 13-6)
    new_margin_m: float = 0.0
    # [시도 후 뺌 2026-10-14] 못 피하면 멈춤 (사용자 "자연스러운 회피가 최선, 안 되면 정지 후 감"): (1) MPC 지도 '닿음'·해 없음·관절 한계에
    # 멈추고 MPC 다시 맞춤 → 한 실행에 74 번 멈추며 MPC 를 흔들어 발산 → 뒤로 젖혀짐·움찔 (사용자 지적), (2) 해 없음 + 보이는 새 물체로
    # 다가갈 때만 멈추고 MPC 는 안 흔듦 → 물체 시나리오 7 개 중 닿은 시나리오 5 개로 안 줄어듦 (멈춤 없이도 5/7). 둘 다 뺌 (작업기록 13-9)
    # [2026-10-15] 성형 여유 보존 (작업기록 5-14, rt_run.Controller._cp_slack): 성형된 y 가 장애물에서 cp_thr 안일 때, 최근 성형기 길이
    #  (125 ms) 동안 성형 전 u 가 간 것보다 더 가까이 가면 안전 검사 개입. → y 가 닿으면 그 직전 u 도 그만큼 가까웠다는 뜻이라, 성형
    #  때문에 생기는 닿음 (u 는 안 닿는데 y 만 닿음) 을 원리적으로 막음 (사용자 기준 2026-10-15 "성형으로 인해 발생하지 않던 충돌은 없어야").
    #  0 = 끔 (2026-10-14 방식). 15 mm 근거: 충돌 구 밖으로 메시 최대 8 mm (R2 13-7) + 지도 1 cm 칸 보간 ~3 mm.
    #  [사용자 결정 2026-10-15] 4 차 규칙 채택 (성형이 켜져 있을 때만 동작, 성형 없음은 그대로) — 시나리오 20 개 × 참 형상·카메라에서 성형 탓
    #  닿음 21 → 4 (남은 4 = 구 ↔ 메시 차이로 1 mm 이하 기둥 스침, '그대로 둠' 사용자 결정), 카메라 다리 평균 +3 % (작업기록 5-15)
    cp_thr: float = 0.015
    cp_eps: float = 0.0005          # 비교 허용 (지도 float16·보간 잡음)
    # y 가 비교 대상 u 와 다른 자세이고 u 가 cp_thr 안까지 왔을 때 y 가 더 떨어져 있어야 하는 여유 (카메라 지도 오차가 자리마다 달라서:
    #  1 차 시험에서 y −0.4 / u +0.3 mm 가 빠져나감). [시도 후 고침 2026-10-15] 같은 자세 (서 있음) 에도 2 mm 를 걸었더니 '자기보다
    #  2 mm 멀어야' 라 늘 위반 → 다리 절반 실패 (is_dev/cp2_broken)
    cp_margin: float = 0.001
    cp_margin_zone: float = 0.003   # 위 여유는 u 가 이 거리 안까지 붙었을 때만 (나란히 지나가는 것까지 막지 않게. 이 안은 충돌 구 오차로 원래도 스칠 수 있는 거리)
    # [사용자 결정 2026-10-16] 위 여유 (cp_margin) 는 성형된 y 가 지금 자리보다 장애물에 더 다가갈 때만 (그 점의 거리 < 지금 거리 − cp_eps).
    #  WMX devel 실행에서 로봇이 기둥에 지도로 0.2~1.9 mm 붙었을 때 기둥을 따라 미끄러지는 후보가 모두 1 mm 여유에 걸려 막힘 → ETM5 다리
    #  시간 초과 (410 번 개입 중 402 번이 이 조건), EI 다리 24 s (작업기록 5-17). [2026-10-27 R-1] 2026-10-15 4 차 규칙 (늘 여유) 되돌리기
    #  (guard.cp_margin_approach False) 는 뺌
    # [2026-10-15] 움직이는 물체 몫 (작업기록 5-15): y 는 u 의 길을 최대 성형기 길이만큼 늦게 지나가므로 그동안 물체가 다가올 수 있음.
    #  새 물체 거리를 u·y 가 각자 지나는 시각의 예측 거리 (거리 − cp_vobj × 지금부터 그 시각까지) 로 보고, u 가 (최악에도) 안 닿는데
    #  y 만 닿게 되는 후보를 막음. [시도 후 고침] 'y 가 u 보다 나쁘면 안 됨' 으로 비교하면 y 는 늘 늦으므로 물체 근처에서는 움직이기만
    #  해도 위반 (is_dev/cp2_broken). 0 = 끔. 값 = 물체가 다가올 수 있는 최고 속도 가정 (m/s) — 사용자 결정 2026-10-15: 0.15 m/s (시험
    #  시나리오 물체 최고 0.144 m/s 를 덮는 값. 사람 손은 더 빠를 수 있음 — 실제 적용 때 다시 볼 값)
    cp_vobj: float = 0.15
    # [2026-10-16 사용자 "(다) 적용 유무를 네가 검증해봐. 시뮬레이션 해서"] 움직이는 물체 규칙 (2)(3) 은 성형된 y 의 점이 지금 y 자리보다 물체
    #  지도에서 더 가까울 때만 (물러나는 점에는 안 걸음). 물러날 때 y 가 u 보다 늦어 '물체 0.15 m/s 가정' 예측 거리가 u 보다 작아져 (멀어지는
    #  속도 < 0.3 m/s 면) 물러나는 움직임도 막았다 → 앞 메시지 계속·멈춤 → 선 채 닿음 (WMX 상자2 60 s, 작업기록 5-21).
    #  [2026-10-27 R-1] 10-15 4 차 규칙 (모든 점) 되돌리기 (guard.cp_dyn_approach False) 는 뺌


@dataclass(frozen=True)
class TaskCfg:
    # 1회성 데모 (cr3a_sim/plan_avoid.py, IK 결과): A·B 손끝 (0.42, ±0.34, 0.20) 아래를 봄 — 공구 없는 모델 ('bare') 의 A·B
    qa: Tuple[float, ...] = tuple(np.radians([53.92, -38.76, -66.34, 22.53, 84.02, 53.34]))
    qb: Tuple[float, ...] = tuple(np.radians([-26.51, -40.04, -63.93, 21.14, 96.29, -26.31]))
    # [2026-10-16 사용자 결정] 끝단 공구 모델 ('tool', rt_model) 의 A·B: 그리퍼 끝 (Link6 + 192.5 mm) 을 위 손끝 자리 (0.42, ±0.34, 0.20) 에,
    #  같은 공구 방향 (플랜지는 19 cm 위). 위 qa·qb 그대로면 손가락 끝이 바닥 위 7 mm (바닥 반응 2 cm 안) 라 도착 못 함.
    #  A = 같은 방향 IK 해. B 는 관절 한계 안에 같은 방향 해가 없어서 끝 위치는 맞추고 공구 축을 2.2° 기울인 해 (기울기 최소)
    qa_tool: Tuple[float, ...] = tuple(np.radians([56.31, -40.67, -12.99, -29.16, 83.72, 55.72]))
    qb_tool: Tuple[float, ...] = tuple(np.radians([-28.78, -42.61, -10.82, -27.41, 96.28, -26.31]))
    arrive_tol_rad: float = math.radians(0.1)   # 도착 = 측정 관절이 목표에서 0.1° 안에 0.3 s (Mirobot 실시간 제어와 같음)
    arrive_hold_s: float = 0.3
    leg_timeout_s: float = 25.0
    hold_at_goal_s: float = 1.0     # 도착한 뒤 다음 다리까지 쉼
    end_hold_s: float = 2.0         # 마지막 다리 뒤 지키기
    legs: int = 16                  # GO 뒤 A↔B 다리 수 (물체가 움직이는 ~60 s 를 덮게)
    pillar: Tuple = ('기둥', (0.42, 0.0, 0.15), (0.10, 0.10, 0.30))   # 정지 물체 (배경에 들어감)
    ideal_plan_s: float = 0.2       # 이상적 모드에서 전역 계획에 걸린 것으로 칠 시간 (실제 걸린 시간을 쓰면 실행마다 달라짐)

    def goals(self, model: str):
        """로봇 모델 (rt_model) 의 A·B 관절값."""
        return (self.qa_tool, self.qb_tool) if model == 'tool' else (self.qa, self.qb)


@dataclass(frozen=True)
class Config:
    dist: Distances = field(default_factory=Distances)
    map: MapCfg = field(default_factory=MapCfg)
    plan: PlanCfg = field(default_factory=PlanCfg)
    mpc: MpcCfg = field(default_factory=MpcCfg)
    motion: MotionCfg = field(default_factory=MotionCfg)
    stream: StreamCfg = field(default_factory=StreamCfg)
    guard: GuardCfg = field(default_factory=GuardCfg)
    task: TaskCfg = field(default_factory=TaskCfg)
    cmpc: CMpcCfg = field(default_factory=CMpcCfg)
    # [2026-10-14] 명령 u 를 만드는 방식: 'carrot' = Mirobot 당근 MPC + 한계 안 추종기 (rt_mpc, 검증된 회피) /
    #  'cmpc' = 성형된 경로를 최적화하는 시간표 MPC (rt_cmpc, C 방식 — 연구 중: 물체 없이는 도착하지만 기둥 근처에서 헤매 다리 6~8 s,
    #  작업기록 5장). 사용자 결정 2026-10-14 (다): 이번 단계는 'carrot' + 실행 기억 성형 + 성형 궤적 안전 검사, C 는 다음 단계.
    #  어느 쪽이든 보내는 명령 = 성형기 (rt_stream) 를 거친 y
    style: str = 'carrot'
    # [2026-10-16] 로봇 형상 모델 (rt_model): 'tool' = 끝단 브래킷·D455·그리퍼 붙음 (사용자 CAD), 'bare' = Dobot URDF 만 (그 전 실행)
    model: str = 'tool'

    def __post_init__(self):
        # 시간표 불변식 (rt_stream): 안 보낸 주기·늦은 주기는 앞 메시지의 '주기 배수' 시각 상태에서 다시 시작하므로, 그 시각이 메시지
        # 점 격자 (명령 점 step_s, 꼬리 점 tail_dt) 위에 있어야 함. 명령 점은 한 주기 뒤 (다음 스탬프) 상태를 포함해야 함
        dt = 1.0 / self.mpc.rate_hz
        for name, h in (('motion.step_s', self.motion.step_s), ('motion.tail_dt', self.motion.tail_dt)):
            assert abs(dt / h - round(dt / h)) < 1e-6, f'주기 {dt} s 가 {name} {h} s 의 배수가 아님'
        assert self.motion.cycles >= 2, 'motion.cycles >= 2 (다음 주기 상태가 명령 점 안에 있어야 함)'
        assert 0.0 < self.stream.late_send_s < self.stream.late_before_s < self.stream.lead_s <= dt
        assert self.style in ('cmpc', 'carrot'), f'style = {self.style!r}'
        assert self.model in ('tool', 'bare'), f'model = {self.model!r}'
        assert self.dist.act_new > self.dist.mask, '반응 거리는 팔 지우기보다 커야 함 (지우기 안 물체는 안 보임)'
        assert 0.0 < self.dist.act_floor <= self.dist.act_new
        assert max(self.map.table_z, 0.0) + self.dist.act_floor <= FLOOR_REACT_TOP_MAX_M + 1e-9, \
            f'충돌 바닥 (map.table_z {self.map.table_z:+.4f}) + dist.act_floor 가 {FLOOR_REACT_TOP_MAX_M} m 를 넘음 — 받침·Link1 이 늘 바닥 반응 거리 안 (계획 실패)'
        assert abs(self.dist.act_static - self.dist.act_new) < 1e-9, '지도가 정지·움직임을 나누지 않음 (2026-10-16) → dist.act_static = dist.act_new'
        assert self.plan.ahead_to_s > self.plan.ahead_from_s >= 0.0 and self.plan.ahead_step_s > 0.0
        assert self.plan.back_stall_s > 0.0 and self.plan.back_window_s > 0.0 and self.plan.back_clear_s > 0.0
        assert 0.0 < self.plan.back_smooth_s <= self.plan.back_window_s and self.plan.back_max_s > self.plan.back_clear_s
        assert abs(self.motion.step_s / self.motion.sub - 0.001) < 1e-12, '명령 궤적 적분 단계 = 1 ms (입력성형 격자) 이어야 함'
