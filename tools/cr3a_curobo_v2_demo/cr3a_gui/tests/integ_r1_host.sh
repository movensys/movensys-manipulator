#!/bin/bash
# [2026-10-28 R-1 통합 시험] Thor 호스트 (mic-735d) 에서: 코어 파일 목록 → 컨테이너 안 tests/integ_r1.sh → 새 코어 파일·호스트에서 본 프로세스.
#   bash <통합 폴더>/cr3a_gui/tests/integ_r1_host.sh [결과 폴더] [A] [B]     (통합 폴더 = 이 파일의 ../.., 컨테이너에서도 같은 경로)
#   ssh 로 떼어 돌리기: setsid -f bash …/integ_r1_host.sh <결과 폴더> > /dev/null 2>&1 < /dev/null  ('&' 는 SIGINT 무시를 물려줌)
# 끝나면 <결과 폴더>/DONE (끝 코드) — 기다림은 이 파일로. pgrep -f 를 쓰지 않음 (ssh 로 부르면 원격 셸 명령 줄이 패턴과 맞음).
set -u
INTEG=$(cd "$(dirname "$0")/../.." && pwd)
OUT=${1:-$INTEG/out_$(date +%m%d_%H%M%S)}
shift || true
CORES=/var/lib/apport/coredump
mkdir -p "$OUT"
ls "$CORES" 2>/dev/null | sort > "$OUT/cores_before.txt"
echo "### 호스트 $(date '+%F %T') · 결과 $OUT · 코어 파일 $(wc -l < "$OUT/cores_before.txt") 개 (전)"
docker exec -u admin wmx_r2 bash "$INTEG/cr3a_gui/tests/integ_r1.sh" "$OUT" "$@" 2>&1 | tee "$OUT/integ.log"
rc=${PIPESTATUS[0]}
ls "$CORES" 2>/dev/null | sort > "$OUT/cores_after.txt"
NEW=$(comm -13 "$OUT/cores_before.txt" "$OUT/cores_after.txt")
if [ -n "$NEW" ]; then echo "FAIL  ⑥ 새 코어 파일: $NEW" | tee -a "$OUT/integ.log"; rc=1
else echo "PASS  ⑥ 새 코어 파일 없음 ($CORES)" | tee -a "$OUT/integ.log"; fi
# 호스트에서 본 프로세스 (컨테이너 것도 보임): WMX 엔진·R2·Servo·카메라 노드·서버·GUI
LEFT=$(ps -eo pid=,args= | awk '/[w]mx3_engine|[w]mx_r2_package\/|[r]os2_control_node|[s]ervo_node|[r]ealsense2_camera_node|[r]t_server\.py|[c]r3a_gui\.py|[w]mx_axis_log/' | cut -c1-160)
if [ -n "$LEFT" ]; then
  { echo "FAIL  ⑥ 호스트에서 본 남은 프로세스:"; echo "$LEFT" | sed 's/^/    /'; } | tee -a "$OUT/integ.log"
  rc=1
else
  echo "PASS  ⑥ 호스트에서 본 엔진·R2·Servo·카메라 노드·서버·GUI 없음" | tee -a "$OUT/integ.log"
fi
echo "### 호스트 끝 코드 $rc · 로그 $OUT/integ.log" | tee -a "$OUT/integ.log"
echo "$rc" > "$OUT/DONE"
exit $rc
