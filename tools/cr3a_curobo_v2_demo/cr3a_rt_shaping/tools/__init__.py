"""cr3a_rt_shaping 도구 (실행 중인 회피·GUI 서버는 이 폴더를 안 씀) — [2026-10-27 R-1 6 단계] 맨 위에서 옮겨 모음.

저장소 맨 위 (cr3a_rt_shaping) 에서 모듈로 돌린다 — 엔진 모듈 (rt_config·rt_model·…) 과 기록 읽기 (rt_records) 를 그대로 불러옴:
  python3 -m tools.rt_report <결과 폴더>          WMX 실행 보고서 (report.html + summary.json)
  python3 -m tools.rt_tools <명령> …              기록 분석 (compare·ideal·contacts·shaping·…)
  python3 -m tools.motion_quality <폴더> …        다리마다 움직임 품질
  python3 -m tools.wmx_cmp <폴더> …               WMX 실행 비교 표 · 한 주기 계산 시간
  python3 -m tools.fake_wmx_from_ideal <이상적 결과> <새 폴더>   보고서 시험용 가짜 WMX 축 로그
  python3 -m tools.check_cmpc [...]               성형 MPC (rt_cmpc, C 방식) 단독 점검
  tools/rt_demo.sh · tools/wmx_batch.sh           (Thor 호스트) WMX 시뮬레이터 실행 + 보고서
"""
