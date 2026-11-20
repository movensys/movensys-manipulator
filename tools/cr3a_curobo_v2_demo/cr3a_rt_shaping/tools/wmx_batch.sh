#!/bin/bash
# [2026-10-14] 입력성형 WMX 시뮬레이터 비교 실행 — rt_demo.sh 를 성형기마다 차례로 (R2·엔진은 한 번에 하나만).  ★ Thor 에서 실행 ★
#   [2026-10-27 R-1] tools/ 로 옮김 (rt_demo.sh 와 같은 폴더): ~/workspaces/movensys_ws/cr3a_rt_shaping/tools/wmx_batch.sh
#   ./wmx_batch.sh                    기본 목록: 성형 없음, ZVD·ZV·EI·ETM4·ETM5 (8 Hz), ZVD 8+15 Hz — 모두 당근 MPC (rt_config.Config.style)
#   LIST="none ZVD:8" ./wmx_batch.sh  목록 바꾸기 (방식:주파수[,주파수…], none = 성형 없음)
#   RT_ARGS="--legs 16" 등은 rt_demo.sh 로 그대로 넘어감. 결과 폴더 목록 = ~/workspaces/wmx_r2_logs/is_batch_<시각>.txt
#   R2 는 devel_cyclic_ws 하나 ([2026-10-27 R-1] R2=cyclic 뺌 — 폴더 이름의 _devel 은 예전 이름과 맞추려고 그대로)
#   MODEL=bare·SCENARIO·STYLE·R2_TRACE 도 rt_demo.sh 로 넘어감 (기본 tool = 끝단 공구 붙음, 2026-10-16). 폴더 이름에 방식·모델 (_tool / _bare)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
LIST=${LIST:-"none ZVD:8 ZV:8 EI:8 ETM4:8 ETM5:8 ZVD:8,15"}
IDX=$HOME/workspaces/wmx_r2_logs/is_batch_$(date +%Y%m%d_%H%M%S).txt
for item in $LIST; do
  meth=${item%%:*}
  hz=""
  [ "$item" != "$meth" ] && hz=$(echo "${item#*:}" | tr ',' ' ')
  ts=$(date +%Y%m%d_%H%M%S)
  tag=$(echo "${meth}${hz:+_$hz}" | tr ' ' '+')
  out=$HOME/workspaces/wmx_r2_logs/is_${STYLE:-carrot}_${tag}_devel_${MODEL:-tool}_$ts
  echo "[$(date +%H:%M:%S)] === $item → $out"
  if [ "$meth" = none ]; then
    OUT=$out SHAPER=none "$HERE/rt_demo.sh"
  else
    OUT=$out SHAPER=$meth SHAPER_HZ="$hz" "$HERE/rt_demo.sh"
  fi
  rc=$?
  echo "$item $out rc=$rc" >> "$IDX"
  [ $rc = 0 ] || echo "[$(date +%H:%M:%S)] ✗ $item 실패 (rc=$rc) — 다음으로"
done
echo "[$(date +%H:%M:%S)] 끝: $IDX"
cat "$IDX"
