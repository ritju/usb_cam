#!/usr/bin/env bash
# Copyright 2026 Evan Flynn
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# ffplay_switch_diag.sh — isolate WHY switching OOMs.
#
#   mode hold    one resolution, never restart   -> baseline leak
#   mode reopen  SAME resolution, kill/restart   -> open/close cost alone
#   mode switch  alternate 640x480 / 1280x1024   -> full problem
#
# Each mode × each interval samples MemAvailable and records whether
# memory recovers after all ffplay processes exit.

set -u

usage() {
  cat <<'EOF'
Usage: ffplay_switch_diag.sh [--help]

Host-side switch-rate / leak attribution (no ROS).

MODES (run in order, default all three):
  hold     play one resolution continuously; no restart
  reopen   FIXED resolution; every INTERVAL: open camera for REOPEN_HOLD
           seconds then close (pure open/close frequency, no resize)
  switch   alternate LOW_RES / HIGH_RES every INTERVAL

Environment:
  MODES             space list (default: "hold reopen switch")
  INTERVALS         seconds between events (default: "30 10 5 2")
                    hold mode ignores this and runs once for HOLD_SECONDS
  PER_INTERVAL_RUNS events per (mode,interval) cell (default: 6)
  REOPEN_HOLD       seconds each reopen cycle keeps the camera open
                    (default: 1). INTERVAL is the period between opens.
  HOLD_SECONDS      duration for hold mode (default: 60)
  PER_INTERVAL_RUNS events per (mode,interval) cell (default: 6)
  LOW_RES           default 640x480
  HIGH_RES          default 1280x1024
  DEVICE            default /dev/rgb_camera_back
  SIDECARS          if set (e.g. "front left right") they stay up the
                    whole diag and must be started BEFORE any experiment
  SAMPLE_SEC        MemAvailable sample period (default: 1)
  OUTDIR            default /tmp/ffplay_switch_diag_<ts>
  FFPLAY            default ffplay
  FRAMERATE         default empty (omit)
  INPUT_FORMAT      default empty (omit/auto)

Output:
  <OUTDIR>/mem_<mode>_<interval>.csv   timestamp,MemAvailable_kb
  <OUTDIR>/summary.txt                 per-cell recovery + FAIL

How to read:
  hold   MemAvailable flat                  -> no continuous leak
         MemAvailable slides down           -> capture-side leak
  reopen drop at each restart, recovers?     -> open/close transient
         never recovers                      -> UVC/vb2 leak on close
  switch worse than reopen at same interval  -> resolution renegotiation
         same as reopen                      -> frequency of open/close
  FINDING column in summary names the cell where MemAvailable never
  returned to the pre-cell baseline (leak) or the first OOM.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

MODES="${MODES:-hold reopen switch}"
INTERVALS="${INTERVALS:-30 10 5 2}"
HOLD_SECONDS="${HOLD_SECONDS:-60}"
PER_INTERVAL_RUNS="${PER_INTERVAL_RUNS:-6}"
LOW_RES="${LOW_RES:-640x480}"
HIGH_RES="${HIGH_RES:-1280x1024}"
DEVICE="${DEVICE:-/dev/rgb_camera_back}"
SIDECARS="${SIDECARS:-}"
SAMPLE_SEC="${SAMPLE_SEC:-1}"
FFPLAY="${FFPLAY:-ffplay}"
FRAMERATE="${FRAMERATE:-}"
INPUT_FORMAT="${INPUT_FORMAT:-}"
TS="$(date +%Y%m%d-%H%M%S)"
OUTDIR="${OUTDIR:-/tmp/ffplay_switch_diag_${TS}}"

if ! command -v "$FFPLAY" >/dev/null 2>&1; then
  echo "ERROR: ffplay not found" >&2
  exit 1
fi
if [[ ! -c "$DEVICE" ]]; then
  echo "ERROR: $DEVICE missing — run on the host" >&2
  exit 1
fi

mkdir -p "$OUTDIR"
SUMMARY="$OUTDIR/summary.txt"

pids=()
sidecar_pids=()
sampler_pid=""
back_pid=""
mem_csv=""
active_sample=0

mem_available_kb() {
  awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0
}

slab_unreclaim_kb() {
  awk '/SUnreclaim:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0
}

cleanup() {
  trap - EXIT INT TERM
  [[ -n "$sampler_pid" ]] && kill "$sampler_pid" 2>/dev/null || true
  if declare -F stop_back >/dev/null 2>&1; then
    stop_back
  fi
  stop_all_sidecars
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_sampler() {
  mem_csv="$1"
  active_sample=1
  echo "epoch,MemAvailable_kb,SUnreclaim_kb" >"$mem_csv"
  (
    while [[ $active_sample -eq 1 ]]; do
      echo "$(date +%s),$(mem_available_kb),$(slab_unreclaim_kb)" >>"$mem_csv"
      sleep "$SAMPLE_SEC"
    done
  ) &
  sampler_pid=$!
}

stop_sampler() {
  active_sample=0
  if [[ -n "$sampler_pid" ]]; then
    kill "$sampler_pid" 2>/dev/null || true
    wait "$sampler_pid" 2>/dev/null || true
    sampler_pid=""
  fi
}

start_ffplay() {
  local dev="$1" w="$2" h="$3" log="$4"
  local -a args=(-loglevel warning -f v4l2 -video_size "${w}x${h}")
  [[ -n "$INPUT_FORMAT" ]] && args+=(-input_format "$INPUT_FORMAT")
  [[ -n "$FRAMERATE" ]] && args+=(-framerate "$FRAMERATE")
  args+=(-i "$dev")
  "$FFPLAY" "${args[@]}" >"$log" 2>&1 &
  echo $!
}

stop_pid() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

stop_back() {
  stop_pid "$back_pid"
  back_pid=""
}

start_back() {
  stop_back
  back_pid="$(start_ffplay "$DEVICE" "$1" "$2" "$3")"
}

start_all_sidecars() {
  [[ -z "$SIDECARS" ]] && return 0
  local name dev pid
  for name in $SIDECARS; do
    dev="/dev/rgb_camera_${name}"
    [[ -c "$dev" ]] || { echo "WARN: $dev missing" | tee -a "$SUMMARY"; continue; }
    pid="$(start_ffplay "$dev" 640 480 "$OUTDIR/sidecar_${name}.log")"
    sidecar_pids+=("$pid")
  done
  echo "[sidecars] started {$SIDECARS}" >>"$OUTDIR/sidecars.log"
}

stop_all_sidecars() {
  local pid
  for pid in "${sidecar_pids[@]:-}"; do
    stop_pid "$pid"
  done
  sidecar_pids=()
}

oom_since_lines() {
  dmesg 2>/dev/null | tail -n +"$(($1 + 1))" 2>/dev/null | grep -E \
    'Out of memory|oom-kill|Killed process|uvcvideo|Failed to resubmit|UVC non compliance|vb2' \
    || true
}

parse_mem_min() {
  # $1=csv -> min MemAvailable
  awk -F, 'NR>1 && $2+0>0 { if (min=="" || $2+0<min) min=$2+0 } END { print min+0 }' "$1"
}

# --- banner -----------------------------------------------------------------

{
  echo "ffplay switch diagnostic ($(date))"
  echo "  device=$DEVICE modes='$MODES' intervals='$INTERVALS'"
  echo "  hold=${HOLD_SECONDS}s runs/cell=$PER_INTERVAL_RUNS"
  echo "  sidecars=${SIDECARS:-none}  logs=$OUTDIR"
} | tee "$SUMMARY"

# Sidecars first (operator order), then any experiment on back.
start_all_sidecars

TOTAL_FAILS=0

run_cell() {
  # $1=mode $2=interval_or_hold
  local mode="$1" spec="$2"
  local csv="$OUTDIR/mem_${mode}_${spec}.csv"
  local base_kb min_kb end_kb recover_kb mark0 markers status finding
  local n i res w h log rc phase_end

  base_kb="$(mem_available_kb)"
  echo | tee -a "$SUMMARY"
  echo "=== mode=$mode spec=${spec}  MemAvailable start=${base_kb} kB  SUnreclaim=$(slab_unreclaim_kb) ===" \
    | tee -a "$SUMMARY"

  start_sampler "$csv"
  mark0=$(dmesg 2>/dev/null | wc -l)

  status="PASS"
  finding=""

  case "$mode" in
    hold)
      w="${LOW_RES%x*}"; h="${LOW_RES#*x}"
      log="$OUTDIR/hold_${spec}.log"
      start_back "$w" "$h" "$log"
      sleep "$spec"
      stop_back
      ;;
    reopen)
      # Pure open/close cadence at a FIXED resolution. No renegotiation.
      # Each period `spec`: open -> capture REOPEN_HOLD seconds -> close
      # -> idle until the next period. Isolates "how often we open/close".
      w="${LOW_RES%x*}"; h="${LOW_RES#*x}"
      local hold="${REOPEN_HOLD:-1}"
      for ((i = 1; i <= PER_INTERVAL_RUNS; i++)); do
        local cycle_start
        cycle_start=$(date +%s)
        log="$OUTDIR/reopen_${spec}_${i}.log"
        start_back "$w" "$h" "$log"
        sleep "$hold"
        stop_back
        # pad out to a full `spec` period between opens
        local remain=$(( spec - ( $(date +%s) - cycle_start ) ))
        if [[ $remain -gt 0 ]]; then
          sleep "$remain"
        fi
        echo "  reopen $i/$PER_INTERVAL_RUNS (open ${hold}s every ${spec}s) mem=$(mem_available_kb)" \
          | tee -a "$SUMMARY"
      done
      ;;
    switch)
      res="$LOW_RES"
      for ((i = 1; i <= PER_INTERVAL_RUNS; i++)); do
        w="${res%x*}"; h="${res#*x}"
        log="$OUTDIR/switch_${spec}_${i}_${w}x${h}.log"
        # switching resolution: stop back first, then re-open (order rule)
        start_back "$w" "$h" "$log"
        phase_end=$(( $(date +%s) + spec ))
        while [[ $(date +%s) -lt $phase_end ]]; do
          sleep 1
        done
        stop_back
        if [[ "$res" == "$LOW_RES" ]]; then res="$HIGH_RES"; else res="$LOW_RES"; fi
        echo "  switch $i/$PER_INTERVAL_RUNS mem=$(mem_available_kb)" | tee -a "$SUMMARY"
      done
      ;;
    *)
      echo "  unknown mode $mode" | tee -a "$SUMMARY"
      ;;
  esac

  stop_sampler
  min_kb="$(parse_mem_min "$csv")"
  end_kb="$(mem_available_kb)"

  # Let the UVC/vb2 stack settle, then see if memory came back.
  sleep 5
  recover_kb="$(mem_available_kb)"

  markers="$(oom_since_lines "$mark0")"
  if [[ -n "$markers" ]]; then
    status="FAIL"
    TOTAL_FAILS=$((TOTAL_FAILS + 1))
  fi

  if [[ "$recover_kb" -lt $((base_kb - 20480)) ]]; then
    # lost more than 20 MB and did not get it back
    status="FAIL"
    TOTAL_FAILS=$((TOTAL_FAILS + 1))
    finding="LEAK: start=${base_kb}kB min=${min_kb}kB recovered=${recover_kb}kB (delta $((base_kb - recover_kb))kB)"
  elif [[ -n "$markers" ]]; then
    finding="OOM markers (see dmesg excerpt)"
  else
    finding="no leak, no OOM (delta $((base_kb - recover_kb))kB)"
  fi

  {
    echo "  status=$status  start=${base_kb} min=${min_kb} end=${end_kb} recovered=${recover_kb}"
    echo "  FINDING: $finding"
    if [[ -n "$markers" ]]; then
      echo "  dmesg:"
      printf '%s\n' "$markers" | sed 's/^/    /'
    fi
  } | tee -a "$SUMMARY"
}

# --- experiment matrix ------------------------------------------------------

for mode in $MODES; do
  if [[ "$mode" == "hold" ]]; then
    run_cell hold "$HOLD_SECONDS"
  else
    for iv in $INTERVALS; do
      run_cell "$mode" "$iv"
    done
  fi
done

echo | tee -a "$SUMMARY"
echo "======== COMPARISON ========" | tee -a "$SUMMARY"
echo "hold     : baseline leak / continuous capture" | tee -a "$SUMMARY"
echo "reopen   : open/close frequency only (fixed res, REOPEN_HOLD per period)" | tee -a "$SUMMARY"
echo "switch   : + resolution renegotiation at the same period" | tee -a "$SUMMARY"
echo | tee -a "$SUMMARY"
if [[ $TOTAL_FAILS -eq 0 ]]; then
  echo "RESULT: no leak and no OOM in this matrix — increase PER_INTERVAL_RUNS or tighten INTERVALS." \
    | tee -a "$SUMMARY"
  exit 0
fi
echo "RESULT: $TOTAL_FAILS failing cells — read FINDING lines above." | tee -a "$SUMMARY"
echo "If only switch fails  -> resolution renegotiation in UVC/vb2." | tee -a "$SUMMARY"
echo "If reopen+switch fail -> open/close frequency (kernel buffer leak or USB re-enum)." | tee -a "$SUMMARY"
echo "If hold fails         -> continuous capture leak, not switching." | tee -a "$SUMMARY"
exit 1
