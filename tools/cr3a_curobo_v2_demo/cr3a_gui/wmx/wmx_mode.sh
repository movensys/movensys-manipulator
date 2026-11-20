#!/bin/bash
# [만듦 2026-10-16] WMX 엔진 플랫폼 전환 — 공용 설정 /opt/wmx3/Module.ini (EtherCAT ↔ simu).  ★ Thor 호스트에서 ★
#   사용자 결정 (2026-10-15): 시뮬레이션을 돌릴 때만 simu, 끝나면 EtherCAT 으로 되돌림 (동료가 실제 CR3A 를 EtherCAT 으로 씀).
#   ./wmx_mode.sh status     켜진 플랫폼, 엔진이 떠 있는지, 보관본·표시 파일
#   ./wmx_mode.sh simu       EtherCAT 판을 Module.ini.ethercat 으로 보관하고 simu 만 켬
#                            rc 0 = 지금 바꿈 / 1 = 이미 simu (안 바꿈) / 2 = 예상 밖 설정 / 3 = 엔진이 떠 있음 (누가 쓰는 중)
#   ./wmx_mode.sh ethercat   보관한 EtherCAT 판으로 되돌림 (rc 0 = 되돌림 또는 이미 EtherCAT)
#   Module.ini 는 엔진이 켜질 때만 읽힌다. 그래서 엔진이 떠 있으면 simu 로 바꾸지 않는다 (남의 실행을 건드리지 않게).
#   [2026-10-26] 동료가 10-25 에 Module.ini 를 EtherCAT 항목 하나만으로 다시 씀 (simu 항목 없음) → simu 항목이 없으면 WMX 기본 틀
#                (wmx-gateway 안의 기본 Module.ini: [Platform 1] Location = ./platform/simu, DllName = simu_platform.so) 대로
#                EtherCAT 항목 다음에 덧붙여 켬. 되돌리기는 늘 보관본을 그대로 씀 (바이트 그대로).
set -u
INI=${WMX_INI:-/opt/wmx3/Module.ini}   # WMX_INI = 시험용 사본 경로 (평소에는 쓰지 않음)
SAVE=$INI.ethercat

platforms() {  # 켜진 [Platform] 의 DllName (공백으로 이음)
  awk -F= '
    function flush() { if (sec ~ /^\[Platform/ && dis == 0 && dll != "") print dll }
    /^[ \t]*\[/ { flush(); sec=$0; gsub(/[ \t\r]/, "", sec); dll=""; dis=0; next }
    { k=$1; gsub(/[ \t\r]/, "", k); v=$2; gsub(/[ \t\r]/, "", v)
      if (k == "DllName") dll=v; if (k == "disable") dis=v+0 }
    END { flush() }' "$1" | sort -u | tr '\n' ' '
}
engine_pid() { ps -C wmx3_engine -o stat=,pid= 2>/dev/null | awk '$1 !~ /^Z/ {print $2; exit}'; }

# [Platform] 마다 disable 을 다시 씀: ec_platform.so → $1, simu_platform.so → $2 (다른 줄은 그대로). $3 = 읽을 파일 (기본 $INI)
rewrite() {
  awk -v ec="$1" -v sim="$2" '
    function flush(   i) {
      for (i = 1; i <= n; i++) {
        line = buf[i]
        if (sec ~ /^\[Platform/ && line ~ /^[ \t]*disable[ \t]*=/) {
          if (dll == "ec_platform.so") line = "disable = " ec
          else if (dll == "simu_platform.so") line = "disable = " sim
        }
        print line
      }
      n = 0
    }
    /^[ \t]*\[/ { flush(); sec=$0; gsub(/[ \t\r]/, "", sec); dll="" }
    { buf[++n] = $0
      if ($0 ~ /^[ \t]*DllName[ \t]*=/) { v=$0; sub(/^[^=]*=/, "", v); gsub(/[ \t\r]/, "", v); dll=v } }
    END { flush() }' "${3:-$INI}"
}

# simu 항목이 없으면 마지막 [Platform] 항목 뒤 (다음 항목 앞) 에 WMX 기본 틀의 simu 항목을 덧붙임 (있으면 그대로)
add_simu() {
  awk '
    function block() { print "[Platform 1]"; print "Location = ./platform/simu"; print "DllName = simu_platform.so"
                       print "NumOfMaster = 1"; print "disable = 1"; print ""; done = 1 }
    /^[ \t]*DllName[ \t]*=[ \t]*simu_platform\.so/ { have = 1 }
    { lines[++n] = $0
      if ($0 ~ /^[ \t]*\[/) { s = $0; gsub(/[ \t\r]/, "", s); sec[n] = s } }
    END {
      for (i in sec) if (sec[i] ~ /^\[Platform/ && i + 0 > last) last = i + 0     # (sec[i] 를 i 마다 부르면 빈 칸이 생겨 in 검사가 틀림)
      for (i = 1; i <= n; i++) {
        if (!have && !done && last && i > last && (i in sec)) block()
        print lines[i]
      }
      if (!have && !done) { print ""; block() }
    }'
}

case "${1:-status}" in
  status)
    echo "Module.ini 켜진 플랫폼: $(platforms "$INI")"
    EP=$(engine_pid); echo "엔진: ${EP:+켜짐 (PID $EP)}${EP:-꺼짐}"
    [ -f "$SAVE" ] && echo "EtherCAT 보관본: $SAVE ($(platforms "$SAVE"))"
    exit 0 ;;
  simu)
    P=$(platforms "$INI")
    [ "$P" = "simu_platform.so " ] && { echo "이미 simu 만 켜져 있음 (안 바꿈)"; exit 1; }
    [ "$P" = "ec_platform.so " ] || { echo "✗ 예상 밖 설정 (켜진 플랫폼: ${P:-없음}) → 안 바꿈"; exit 2; }
    EP=$(engine_pid)
    [ -n "$EP" ] && { echo "✗ WMX 엔진이 떠 있음 (PID $EP) = 누가 쓰는 중 → 안 바꿈"; exit 3; }
    cp -p "$INI" "$SAVE" || exit 2
    TMP=$(mktemp) || exit 2
    add_simu < "$INI" > "$TMP.in" && rewrite 1 0 "$TMP.in" > "$TMP"; rm -f "$TMP.in"
    [ "$(platforms "$TMP")" = "simu_platform.so " ] || { echo "✗ 바꾼 내용 확인 실패 → 안 바꿈"; rm -f "$TMP"; exit 2; }
    cat "$TMP" > "$INI"; rm -f "$TMP"     # 제자리에 써서 주인·권한 유지
    echo "simu 로 바꿈 (EtherCAT 판 보관: $SAVE)"; exit 0 ;;
  ethercat)
    P=$(platforms "$INI")
    if [ "$P" = "ec_platform.so " ]; then
      [ -f "$SAVE" ] && cmp -s "$SAVE" "$INI" && rm -f "$SAVE"     # 남은 보관본 (지금 파일과 같음) 정리
      echo "이미 EtherCAT"; exit 0
    fi
    [ -f "$SAVE" ] || { echo "✗ 보관본 없음 ($SAVE) → 손으로 확인 필요"; exit 2; }
    [ "$(platforms "$SAVE")" = "ec_platform.so " ] || { echo "✗ 보관본이 EtherCAT 판이 아님 → 손으로 확인 필요"; exit 2; }
    cat "$SAVE" > "$INI"
    [ "$(platforms "$INI")" = "ec_platform.so " ] || { echo "✗ 되돌린 뒤 확인 실패"; exit 2; }
    # [2026-10-26] /opt/wmx3 에 남는 파일이 없게 (사용자: 공용 폴더에 따로 만든 파일을 남기지 않음) — 바이트 그대로 되돌렸으면 보관본 지움
    cmp -s "$SAVE" "$INI" && rm -f "$SAVE"
    EP=$(engine_pid)
    [ -n "$EP" ] && echo "⚠ 엔진이 아직 떠 있음 (PID $EP): 지금 엔진은 그대로이고, 다음에 켜질 때 EtherCAT 으로 켜짐"
    echo "EtherCAT 으로 되돌림"; exit 0 ;;
  *) echo "사용법: $0 status|simu|ethercat"; exit 2 ;;
esac
