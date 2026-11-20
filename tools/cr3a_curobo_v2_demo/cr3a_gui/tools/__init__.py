"""CR3A GUI 도구 (GUI·서버는 이 폴더를 안 씀) — [2026-10-27 R-1 6 단계] 맨 위에서 옮겨 모음. cr3a_gui 에서 모듈로 돌린다 (wmx_r2 컨테이너 안):
  PYTHONPATH=../cr3a_rt_shaping python3 -m tools.wmx_check <기록 폴더> [서버 로그]   GUI WMX 실행 검증 (기록 읽기 = cr3a_rt_shaping/rt_records)
  python3 -m tools.cam_capture <시리얼> <폴더> [장 수]                               RealSense 한 장 찍기 (camera_io 를 씀)
"""
