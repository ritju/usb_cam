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

# ffplay_oom_probe.sh — multi-camera OOM probe, runs on the HOST (outside
# any container), no ROS / no undistort.
#
#   rgb_camera_(front|left|right)  continuously played in the background
#   rgb_camera_back                resolution switched every SWITCH_INTERVAL
#
# Goal: reproduce resolution-switch OOM under real multi-camera load and
# attribute the fault:
#   probe FAILs  -> UVC / libavdevice / REQBUFS (not usb_cam)
#   probe PASSes -> our open/grab path still at fault

set -u

usage() {
  cat <<'EOF'
Usage: ffplay_oom_probe.sh [--help]

Runs on the host against the /dev/rgb_camera_* nodes.

  Continuous play (background ffplay, restarted if it dies):
    /dev/rgb_camera_front
    /dev/rgb_camera_left
    /dev/rgb_camera_right

  Resolution switch (foreground, every SWITCH_INTERVAL seconds):
    /dev/rgb_camera_back
    walks RESOLUTIONS, one step per interval

Environment:
  SWITCH_INTERVAL   seconds between back-camera resolution switches (default: 5)
  RESOLUTIONS       space-separated WxH list for the back camera
                    (default: "640x480 800x600 1280x720 1280x1024 1920x1080")
  DURATION          total probe runtime seconds (default: 300 = 5 min)
  FRAMERATE         fps for every ffplay (default: 10)
  INPUT_FORMAT      FFmpeg input_format (default: mjpeg = mjpeg2rgb)
  SIDECARS          sidecar device basenames (default: "front left right")
  BACK_DEVICE       back device path (default: /dev/rgb_camera_back)
  RESIZE_ON_START   1 = a switch happens immediately (default: 1)
  OUTDIR            log directory (default: /tmp/ffplay_oom_probe_<ts>)
  SHOW=1            open sidecar windows (needs DISPLAY); back always -nodisp
  FFPLAY            ffplay binary (default: ffplay)

Example (host):
  ./scripts/ffplay_oom_probe.sh
  SWITCH_INTERVAL=5 DURATION=600 ./scripts/ffplay_oom_probe.sh
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

SWITCH_INTERVAL="${SWITCH_INTERVAL:-5}"
RESOLUTIONS="${RESOLUTIONS:-640x480 800x600 1280x720 1280x1024 1920x1080}"
DURATION="${DURATION:-300}"
FRAMERATE="${FRAMERATE:-10}"
INPUT_FORMAT="${INPUT_FORMAT:-mjpeg}"
SIDECARS="${SIDECARS:-front left right}"
BACK_DEVICE="${BACK_DEVICE:-/dev/rgb_camera_back}"
RESIZE_ON_START="${RESIZE_ON_START:-1}"
FFPLAY="${FFPLAY:-ffplay}"
SHOW="${SHOW:-0}"
TS="$(date +%Y%m%d-%H%M%S)"
OUTDIR="${OUTDIR:-/tmp/ffplay_oom_probe_${TS}}"

if ! command -v "$FFPLAY" >/dev/null 2>&1; then
  echo "ERROR: ffplay not found (FFPLAY=$FFPLAY)" >&2
  exit 1
fi

if [[ ! -c "$BACK_DEVICE" ]]; then
  echo "ERROR: back device $BACK_DEVICE missing or not a char device" >&2
  echo "Run this on the host where /dev/rgb_camera_* exist." >&2
  exit 1
fi

mkdir -p "$OUTDIR"
SUMMARY="$OUTDIR/summary.txt"

# --- helpers ---------------------------------------------------------------

pids=()
sidecar_logs=()

cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

mem_available_kb() {
  awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0
}

dmesg_oom_lines() {
  dmesg 2>/dev/null | tail -n 3000 | grep -E \
    'Out of memory|oom-kill|Killed process|uvcvideo|Failed to resubmit|UVC non compliance|vb2' \
    || true
}

ffplay_cmd() {
  # $1=device $2=width $3=height $4=extra nodisp?
  local dev="$1" w="$2" h="$3"
  local -a disp=(-nodisp)
  if [[ "$SHOW" == "1" && -n "${DISPLAY:-}" && "$dev" != "$BACK_DEVICE" ]]; then
    disp=()
  fi
  "$FFPLAY" -loglevel warning \
    -f v4l2 \
    -input_format "$INPUT_FORMAT" \
    -video_size "${w}x${h}" \
    -framerate "$FRAMERATE" \
    -i "$dev" \
    -autoexit \
    "${disp[@]}"
}

# Keep one ffplay alive on a device at a fixed geometry (sidecars).
keep_alive() {
  local name="$1" dev="$2" w="$3" h="$4"
  local log="$OUTDIR/sidecar_${name}.log"
  sidecar_logs+=("$log")
  (
    while true; do
      echo "[sidecar $name] $(date +%T) play ${w}x${h} on $dev" >>"$log"
      ffplay_cmd "$dev" "$w" "$h" >>"$log" 2>&1
      echo "[sidecar $name] $(date +%T) ffplay exited rc=$?" >>"$log"
      sleep 1
    done
  ) &
  pids+=($!)
}

# --- banner ----------------------------------------------------------------

{
  echo "ffplay multi-camera OOM probe ($(date))"
  echo "  back=$BACK_DEVICE switch every ${SWITCH_INTERVAL}s resolutions=$RESOLUTIONS"
  echo "  sidecars=$SIDECARS (continuous) format=$INPUT_FORMAT fps=$FRAMERATE"
  echo "  duration=${DURATION}s logs=$OUTDIR"
} | tee "$SUMMARY"

# --- start sidecar players -------------------------------------------------

for name in $SIDECARS; do
  dev="/dev/rgb_camera_${name}"
  if [[ ! -c "$dev" ]]; then
    echo "WARN: sidecar $dev missing — skipping" | tee -a "$SUMMARY"
    continue
  fi
  # Sidecars hold a mid resolution so only the back camera stresses switches.
  keep_alive "$name" "$dev" 640 480
  echo "sidecar $name started on $dev" | tee -a "$SUMMARY"
done

if [[ "$SHOW" != "1" ]]; then
  echo "  (sidecars use -nodisp; SHOW=1 to open windows)" | tee -a "$SUMMARY"
fi

# --- back camera: switch resolution every SWITCH_INTERVAL ------------------

RES_LIST=($RESOLUTIONS)
NRES=${#RES_LIST[@]}
if [[ $NRES -eq 0 ]]; then
  echo "ERROR: RESOLUTIONS is empty" >&2
  exit 1
fi

idx=0
if [[ "$RESIZE_ON_START" != "1" ]]; then
  idx=0
fi

FAILS=0
SWITCHES=0
DEADLINE=$(( $(date +%s) + DURATION ))
DMESG_BASE=$(dmesg 2>/dev/null | wc -l)

while [[ $(date +%s) -lt $DEADLINE ]]; do
  res="${RES_LIST[$idx]}"
  W="${res%x*}"
  H="${res#*x}"
  SWITCHES=$((SWITCHES + 1))
  LOG="$OUTDIR/back_${SWITCHES}_${W}x${H}.log"
  MEM_BEFORE="$(mem_available_kb)"
  MSG_BEFORE=$(dmesg 2>/dev/null | wc -l)

  echo "[back] $(date +%T) switch -> ${W}x${H} (${SWITCH_INTERVAL}s)" | tee -a "$SUMMARY"

  # Run for one switch interval; kill when time is up so we can resize.
  ffplay_cmd "$BACK_DEVICE" "$W" "$H" >"$LOG" 2>&1 &
  FPID=$!
  pids+=($FPID)

  slept=0
  while [[ $slept -lt $SWITCH_INTERVAL ]]; do
    if ! kill -0 "$FPID" 2>/dev/null; then
      break
    fi
    sleep 1
    slept=$((slept + 1))
  done

  RC=0
  if kill -0 "$FPID" 2>/dev/null; then
    kill "$FPID" 2>/dev/null || true
    wait "$FPID" 2>/dev/null || true
    # SIGTERM from us after a full interval is normal, not a failure.
  else
    wait "$FPID" 2>/dev/null
    RC=$?
  fi
  # drop the finished pid from the list to keep it short
  new_pids=()
  for p in "${pids[@]:-}"; do
    [[ "$p" == "$FPID" ]] || new_pids+=("$p")
  done
  pids=("${new_pids[@]:-}")

  MEM_AFTER="$(mem_available_kb)"
  NEW_DMESG="$(dmesg 2>/dev/null | tail -n +"$((MSG_BEFORE + 1))" 2>/dev/null || true)"
  MARKERS="$(printf '%s\n' "$NEW_DMESG" | grep -E \
    'Out of memory|oom-kill|Killed process|uvcvideo|Failed to resubmit|UVC non compliance|vb2' \
    || true)"

  STATUS="PASS"
  # rc 0 = played the interval; 137/143 = our kill after interval (ok)
  if [[ $RC -ne 0 && $RC -ne 137 && $RC -ne 143 ]]; then
    STATUS="FAIL"
    FAILS=$((FAILS + 1))
  fi
  if [[ -n "$MARKERS" ]]; then
    STATUS="FAIL"
    FAILS=$((FAILS + 1))
  fi

  {
    echo "  status=$STATUS rc=$RC MemAvailable ${MEM_BEFORE} -> ${MEM_AFTER} kB"
    if [[ -n "$MARKERS" ]]; then
      echo "  dmesg:"
      printf '%s\n' "$MARKERS" | sed 's/^/    /'
    fi
    if [[ "$STATUS" == "FAIL" && -s "$LOG" ]]; then
      echo "  ffplay stderr (tail):"
      tail -n 15 "$LOG" | sed 's/^/    /'
    fi
  } | tee -a "$SUMMARY"

  # Sidecar health: any that died while we were not looking are a signal.
  for name in $SIDECARS; do
    dev="/dev/rgb_camera_${name}"
    [[ -c "$dev" ]] || continue
  done

  idx=$(( (idx + 1) % NRES ))
done

# --- final OOM scan --------------------------------------------------------

FINAL_MARKERS="$(dmesg_oom_lines)"
if [[ -n "$FINAL_MARKERS" ]]; then
  echo | tee -a "$SUMMARY"
  echo "dmesg OOM/uvc markers during run:" | tee -a "$SUMMARY"
  printf '%s\n' "$FINAL_MARKERS" | sed 's/^/  /' | tee -a "$SUMMARY"
  FAILS=$((FAILS + 1))
fi

cleanup

echo | tee -a "$SUMMARY"
if [[ $FAILS -eq 0 ]]; then
  echo "RESULT: PASS ($SWITCHES back switches, sidecars held, no OOM markers)" | tee -a "$SUMMARY"
  echo "If usb_cam still OOMs on this host, the bug is in our open/grab path." | tee -a "$SUMMARY"
  exit 0
fi

echo "RESULT: FAIL ($FAILS issues, $SWITCHES back switches) — see $OUTDIR" | tee -a "$SUMMARY"
echo "If ffplay also OOMs with sidecars running, fault is UVC/libavdevice/REQBUFS." | tee -a "$SUMMARY"
exit 1
