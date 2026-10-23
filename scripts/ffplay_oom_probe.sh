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

# ffplay_oom_probe.sh — drive ffplay through v4l2 resolution switches to see
# whether OOM reproduces outside usb_cam (no ROS, no undistort).
#
# Exit 0 = all runs clean; 1 = at least one FAIL (ffplay error or OOM markers).

set -u

usage() {
  cat <<'EOF'
Usage: ffplay_oom_probe.sh [device] [input_format]

  device         v4l2 node (default: /dev/rgb_camera_back)
  input_format   FFmpeg input_format (default: mjpeg, matches pixel_format mjpeg2rgb)

Environment:
  RESOLUTIONS  space-separated WxH list
                 (default: "640x480 800x600 1280x720 1280x1024 1920x1080")
  FRAMERATE    fps passed to ffplay (default: 10)
  SECONDS      capture seconds per resolution (default: 2)
  CYCLES       how many times to walk the resolution list (default: 3)
  OUTDIR       log directory (default: /tmp/ffplay_oom_probe_<timestamp>)
  SHOW=1       open a window instead of -nodisp
  FFPLAY       ffplay binary (default: ffplay)

Example:
  ./scripts/ffplay_oom_probe.sh /dev/rgb_camera_back mjpeg
  CYCLES=10 RESOLUTIONS="1280x1024 640x480" ./scripts/ffplay_oom_probe.sh
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

DEVICE="${1:-/dev/rgb_camera_back}"
INPUT_FORMAT="${2:-mjpeg}"
RESOLUTIONS="${RESOLUTIONS:-640x480 800x600 1280x720 1280x1024 1920x1080}"
FRAMERATE="${FRAMERATE:-10}"
SECONDS="${SECONDS:-2}"
CYCLES="${CYCLES:-3}"
FFPLAY="${FFPLAY:-ffplay}"
TS="$(date +%Y%m%d-%H%M%S)"
OUTDIR="${OUTDIR:-/tmp/ffplay_oom_probe_${TS}}"
SHOW="${SHOW:-0}"

if ! command -v "$FFPLAY" >/dev/null 2>&1; then
  echo "ERROR: ffplay not found (FFPLAY=$FFPLAY)" >&2
  exit 1
fi

if [[ ! -e "$DEVICE" ]]; then
  echo "ERROR: device $DEVICE does not exist" >&2
  echo "Plug in the camera or pass a valid path: $0 /dev/video0 mjpeg" >&2
  exit 1
fi

if [[ ! -c "$DEVICE" ]]; then
  echo "ERROR: $DEVICE is not a character device" >&2
  exit 1
fi

mkdir -p "$OUTDIR"
SUMMARY="$OUTDIR/summary.txt"
DMESG_MARK="$OUTDIR/dmesg_mark.txt"

echo "ffplay OOM probe" | tee "$SUMMARY"
echo "  device=$DEVICE input_format=$INPUT_FORMAT framerate=$FRAMERATE" | tee -a "$SUMMARY"
echo "  resolutions=$RESOLUTIONS cycles=$CYCLES seconds=$SECONDS" | tee -a "$SUMMARY"
echo "  logs=$OUTDIR" | tee -a "$SUMMARY"
echo | tee -a "$SUMMARY"

if command -v dmesg >/dev/null 2>&1; then
  dmesg | tail -5 > "$DMESG_MARK" 2>/dev/null || true
fi

mem_available_kb() {
  awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0
}

oom_markers_since() {
  # Lines that indicate the failure class we are hunting.
  dmesg 2>/dev/null | tail -n 2000 | grep -E \
    'Out of memory|oom-kill|Killed process|uvcvideo|Failed to resubmit|UVC non compliance|vb2' \
    || true
}

DISPLAY_ARGS=(-nodisp -autoexit)
if [[ "$SHOW" == "1" && -n "${DISPLAY:-}" ]]; then
  DISPLAY_ARGS=(-autoexit)
fi

FAILS=0
RUN=0

for ((cycle = 1; cycle <= CYCLES; cycle++)); do
  for res in $RESOLUTIONS; do
    RUN=$((RUN + 1))
    W="${res%x*}"
    H="${res#*x}"
    LOG="$OUTDIR/run_${cycle}_${W}x${H}.log"
    MEM_BEFORE="$(mem_available_kb)"
    DMESG_BEFORE="$(dmesg 2>/dev/null | wc -l)"

    echo "[$RUN] cycle=$cycle ${W}x${H} (${INPUT_FORMAT} @ ${FRAMERATE}fps, ${SECONDS}s)" | tee -a "$SUMMARY"

    # Same semantic surface as FfmpegCapture::open format_opts.
    "$FFPLAY" \
      -loglevel warning \
      -f v4l2 \
      -input_format "$INPUT_FORMAT" \
      -video_size "${W}x${H}" \
      -framerate "$FRAMERATE" \
      -i "$DEVICE" \
      -t "$SECONDS" \
      "${DISPLAY_ARGS[@]}" \
      >"$LOG" 2>&1
    RC=$?

    MEM_AFTER="$(mem_available_kb)"
    NEW_DMESG="$(dmesg 2>/dev/null | tail -n +"$((DMESG_BEFORE + 1))" 2>/dev/null || true)"
    MARKERS="$(printf '%s\n' "$NEW_DMESG" | grep -E \
      'Out of memory|oom-kill|Killed process|uvcvideo|Failed to resubmit|UVC non compliance|vb2' \
      || true)"

    STATUS="PASS"
    if [[ $RC -ne 0 ]]; then
      STATUS="FAIL"
      FAILS=$((FAILS + 1))
    fi
    if [[ -n "$MARKERS" ]]; then
      STATUS="FAIL"
      FAILS=$((FAILS + 1))
    fi

    {
      echo "  exit=$RC status=$STATUS MemAvailable ${MEM_BEFORE} -> ${MEM_AFTER} kB"
      if [[ -n "$MARKERS" ]]; then
        echo "  dmesg markers:"
        printf '%s\n' "$MARKERS" | sed 's/^/    /'
      fi
      if [[ $RC -ne 0 ]]; then
        echo "  ffplay stderr (tail):"
        tail -n 20 "$LOG" | sed 's/^/    /'
      fi
    } | tee -a "$SUMMARY"
  done
done

echo | tee -a "$SUMMARY"
if [[ $FAILS -eq 0 ]]; then
  echo "RESULT: PASS ($RUN runs, no OOM markers)" | tee -a "$SUMMARY"
  echo "If this passes but usb_cam still OOMs, the bug is in our open/grab path." | tee -a "$SUMMARY"
  exit 0
fi

echo "RESULT: FAIL ($FAILS issues across $RUN runs) — see $OUTDIR" | tee -a "$SUMMARY"
echo "If ffplay also OOMs, the fault is UVC/libavdevice/REQBUFS, not usb_cam." | tee -a "$SUMMARY"
exit 1
