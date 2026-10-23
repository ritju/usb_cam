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

# ffplay_back_toggle.sh — one experiment only (host-side, no ROS):
#
#   rgb_camera_(front|left|right)  streaming continuously
#   rgb_camera_back                opened/closed every INTERVAL seconds
#
# Resolution of the back camera is a parameter; it is NOT switched during
# the run. Use this to see whether intermittent back open/close alone OOMs.

set -u

usage() {
  cat <<'EOF'
Usage: ffplay_back_toggle.sh [resolution]

  resolution   back camera WxH (default: 640x480)
               e.g. 640x480  1280x1024

Keeps rgb_camera_(front|left|right) playing. Only rgb_camera_back is
toggled: open for ON_SECONDS, then close, every INTERVAL seconds.

Environment:
  INTERVAL       seconds between back open events (default: 5)
  ON_SECONDS     how long back stays open each cycle (default: 1)
  CYCLES         number of open/close cycles (default: 36 = 3 min at 5s)
  SIDECARS       names (default: front left right)
  BACK_DEVICE    default /dev/rgb_camera_back
  FRAMERATE      default empty (omit -framerate)
  INPUT_FORMAT   default empty (omit -input_format)
  OUTDIR         default /tmp/ffplay_back_toggle_<ts>
  FFPLAY         default ffplay

Example:
  ./scripts/ffplay_back_toggle.sh
  ./scripts/ffplay_back_toggle.sh 1280x1024
  INTERVAL=2 ON_SECONDS=1 CYCLES=60 ./scripts/ffplay_back_toggle.sh 640x480

Start order: sidecars first, then back.
Stop order:  back first, then sidecars (Ctrl-C too).

Output:
  <OUTDIR>/summary.txt   per-cycle MemAvailable + dmesg OOM/uvc markers
  <OUTDIR>/mem.csv       1s samples of MemAvailable / SUnreclaim
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

RES="${1:-640x480}"
if [[ ! "$RES" =~ ^[0-9]+x[0-9]+$ ]]; then
  echo "ERROR: resolution must look like 640x480, got '$RES'" >&2
  exit 1
fi
W="${RES%x*}"
H="${RES#*x}"

INTERVAL="${INTERVAL:-5}"
ON_SECONDS="${ON_SECONDS:-1}"
CYCLES="${CYCLES:-36}"
SIDECARS="${SIDECARS:-front left right}"
BACK_DEVICE="${BACK_DEVICE:-/dev/rgb_camera_back}"
FRAMERATE="${FRAMERATE:-}"
INPUT_FORMAT="${INPUT_FORMAT:-}"
FFPLAY="${FFPLAY:-ffplay}"
TS="$(date +%Y%m%d-%H%M%S)"
OUTDIR="${OUTDIR:-/tmp/ffplay_back_toggle_${TS}}"

if ! command -v "$FFPLAY" >/dev/null 2>&1; then
  echo "ERROR: ffplay not found" >&2
  exit 1
fi
if [[ ! -c "$BACK_DEVICE" ]]; then
  echo "ERROR: $BACK_DEVICE missing — run on the host" >&2
  exit 1
fi

mkdir -p "$OUTDIR"
SUMMARY="$OUTDIR/summary.txt"
MEMCSV="$OUTDIR/mem.csv"

sidecar_pids=()
back_pid=""
sampler_pid=""
active_sample=0

mem_available_kb() {
  awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0
}
slab_unreclaim_kb() {
  awk '/SUnreclaim:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0
}

stop_pid() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

# Operator order: back first when stopping, sidecars first when starting.
stop_back() {
  stop_pid "$back_pid"
  back_pid=""
}
stop_sidecars() {
  local pid
  for pid in "${sidecar_pids[@]:-}"; do
    stop_pid "$pid"
  done
  sidecar_pids=()
}
cleanup() {
  trap - EXIT INT TERM
  active_sample=0
  stop_pid "$sampler_pid"
  stop_back
  stop_sidecars
}
trap cleanup EXIT INT TERM

start_ffplay() {
  local dev="$1" w="$2" h="$3" log="$4"
  local -a args=(-loglevel warning -f v4l2 -video_size "${w}x${h}")
  [[ -n "$INPUT_FORMAT" ]] && args+=(-input_format "$INPUT_FORMAT")
  [[ -n "$FRAMERATE" ]] && args+=(-framerate "$FRAMERATE")
  args+=(-i "$dev")
  "$FFPLAY" "${args[@]}" >"$log" 2>&1 &
  echo $!
}

start_sidecars() {
  local name dev pid
  for name in $SIDECARS; do
    dev="/dev/rgb_camera_${name}"
    if [[ ! -c "$dev" ]]; then
      echo "WARN: sidecar $dev missing — skip" | tee -a "$SUMMARY"
      continue
    fi
    pid="$(start_ffplay "$dev" 640 480 "$OUTDIR/sidecar_${name}.log")"
    sidecar_pids+=("$pid")
    echo "[sidecars] $(date +%T) start $name pid=$pid" >>"$OUTDIR/cameras.log"
  done
}

start_back() {
  back_pid="$(start_ffplay "$BACK_DEVICE" "$W" "$H" "$OUTDIR/back_last.log")"
  echo "[back] $(date +%T) start pid=$back_pid ${W}x${H}" >>"$OUTDIR/cameras.log"
}

start_sampler() {
  echo "epoch,MemAvailable_kb,SUnreclaim_kb" >"$MEMCSV"
  active_sample=1
  (
    while [[ $active_sample -eq 1 ]]; do
      echo "$(date +%s),$(mem_available_kb),$(slab_unreclaim_kb)" >>"$MEMCSV"
      sleep 1
    done
  ) &
  sampler_pid=$!
}

oom_since_lines() {
  dmesg 2>/dev/null | tail -n +"$(($1 + 1))" 2>/dev/null | grep -E \
    'Out of memory|oom-kill|Killed process|uvcvideo|Failed to resubmit|UVC non compliance|vb2' \
    || true
}

# --- run --------------------------------------------------------------------

{
  echo "ffplay back-toggle ($(date))"
  echo "  sidecars={$SIDECARS} continuous"
  echo "  back=$BACK_DEVICE res=${W}x${H} open ${ON_SECONDS}s every ${INTERVAL}s × ${CYCLES}"
  echo "  logs=$OUTDIR"
} | tee "$SUMMARY"

BASE_KB="$(mem_available_kb)"
echo "MemAvailable start=${BASE_KB} kB SUnreclaim=$(slab_unreclaim_kb)" | tee -a "$SUMMARY"

start_sampler
start_sidecars
# sidecars first, then back (operator order)
sleep 1

FAILS=0
MARK0=$(dmesg 2>/dev/null | wc -l)

for ((i = 1; i <= CYCLES; i++)); do
  cycle_start=$(date +%s)
  start_back
  sleep "$ON_SECONDS"
  stop_back

  markers="$(oom_since_lines "$MARK0")"
  MARK0=$(dmesg 2>/dev/null | wc -l)
  status="ok"
  if [[ -n "$markers" ]]; then
    status="FAIL"
    FAILS=$((FAILS + 1))
  fi

  echo "cycle $i/$CYCLES mem=$(mem_available_kb) $status" | tee -a "$SUMMARY"
  if [[ -n "$markers" ]]; then
    printf '%s\n' "$markers" | sed 's/^/    dmesg: /' | tee -a "$SUMMARY"
  fi

  # pad to a full INTERVAL between open events
  remain=$(( INTERVAL - ( $(date +%s) - cycle_start ) ))
  if [[ $remain -gt 0 && $i -lt $CYCLES ]]; then
    sleep "$remain"
  fi
done

# teardown: back already down; stop sidecars, then measure recovery
stop_sidecars
active_sample=0
stop_pid "$sampler_pid"
sleep 5
RECOVER_KB="$(mem_available_kb)"
FINAL_MARKERS="$(oom_since_lines 0)"

{
  echo
  echo "start=${BASE_KB}kB  recovered=${RECOVER_KB}kB  delta=$((BASE_KB - RECOVER_KB))kB"
  if [[ $((BASE_KB - RECOVER_KB)) -gt 20480 ]]; then
    echo "FINDING: LEAK — memory did not return after all streams stopped"
    FAILS=$((FAILS + 1))
  fi
  if [[ -n "$FINAL_MARKERS" ]]; then
    echo "dmesg (session):"
    printf '%s\n' "$FINAL_MARKERS" | sed 's/^/  /'
  fi
  echo
  if [[ $FAILS -eq 0 ]]; then
    echo "RESULT: PASS ($CYCLES cycles, no OOM, no leak)"
  else
    echo "RESULT: FAIL ($FAILS) — see $OUTDIR"
    echo "If this OOMs, intermittent back open/close under sidecar load is enough."
  fi
} | tee -a "$SUMMARY"

exit $(( FAILS > 0 ? 1 : 0 ))
