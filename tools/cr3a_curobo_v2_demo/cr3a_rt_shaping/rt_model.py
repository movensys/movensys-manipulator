"""로봇 형상 모델 (한곳) — [2026-10-16] 끝단 공구를 붙인 모델을 더함.

  'tool' = Dobot CR3A + 끝단 공구 (브래킷 robot_braket + RealSense D455 + DH-Robotics AG-160-95 그리퍼, 사용자 CAD
           0_IIFES Demo.SLDASM → robot_tool/make_tool_meshes.py). 메시 = CAD 그대로 (Link6 좌표), cuRobo 충돌 구 = 팔 60 개
           (예전 그대로) + 공구 전체를 빈틈없이 덮는 큰 구 (robot_tool/cover_spheres.py, 사용자 결정 2026-10-16). 기본값.
  'bare' = Dobot URDF 만 (2026-10-16 오전까지의 모든 실행).

고르는 곳: rt_run.py --model (rt_config.Config.model). 실행 기록 rt_run.json 의 settings.model 에 남고, 보고서·판정 도구
(rt_report, rt_tools) 는 그 기록의 모델을 씀 (기록에 없으면 'bare' = 예전 실행). 기록 없이 쓰는 도구는 환경 변수 CR3A_MODEL.
"""
import os
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIM = HERE.parent / 'cr3a_sim'
TOOLDIR = HERE / 'robot_tool'
DEFAULT = 'tool'


@dataclass(frozen=True)
class Model:
    name: str
    urdf: Path          # 깊이 영상 (depth_sim)·실제 형상 거리 (닿음 판정)·보고서 3D 의 메시
    asset: Path         # URDF 메시 경로 package://… 의 기준 폴더
    yml: Path           # cuRoboV2 로봇 설정 (충돌 구·한계): MPC·계획기·안전 검사·팔 지우기
    tip_m: float        # 손끝 = Link6 원점에서 공구 축 (+z) 으로 이만큼 (bare 0 = 플랜지 면, tool 0.1925 = 그리퍼 손가락 끝)
    view_dir: str = ''  # 보고서 3D 에 넣을 가벼운 메시 폴더 (메시 파일과 같은 폴더 아래, 있으면 그것)


MODELS = {
    'bare': Model('bare', SIM / 'dobot_rviz' / 'urdf' / 'cr3_robot.urdf', SIM, SIM / 'cr3a_curobo.yml', 0.0),
    'tool': Model('tool', TOOLDIR / 'cr3_robot_tool.urdf', TOOLDIR, TOOLDIR / 'cr3a_tool_curobo.yml', 0.1925, 'view'),
}


def get(name=None) -> Model:
    """이름 → 모델. None = 환경 변수 CR3A_MODEL, 그것도 없으면 DEFAULT."""
    name = name or os.environ.get('CR3A_MODEL') or DEFAULT
    if name not in MODELS:
        raise KeyError(f'로봇 모델 {name!r}: {sorted(MODELS)}')
    return MODELS[name]


def of_run(rec) -> Model:
    """실행 기록 (rt_run.json dict) → 그 실행의 모델 (기록에 없으면 'bare' = 2026-10-16 오전까지 실행)."""
    return MODELS[((rec or {}).get('settings') or {}).get('model', 'bare')]
