// Copyright 2026 Evan Flynn
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef USB_CAM__TIMESTAMPS_HPP_
#define USB_CAM__TIMESTAMPS_HPP_

extern "C" {
#include <libavutil/avutil.h>
#include <libavutil/mathematics.h>
#include <libavutil/rational.h>
}

#include <ctime>

namespace usb_cam
{
namespace timestamps
{

/// @brief Wall-clock realtime now
inline timespec wallclock_now()
{
  timespec ts{};
  clock_gettime(CLOCK_REALTIME, &ts);
  return ts;
}

inline bool is_plausible_realtime(const double seconds)
{
  // 2000-01-01 and not more than ~60s in the future
  constexpr double kMin = 946684800.0;
  const timespec now = wallclock_now();
  const double now_s = static_cast<double>(now.tv_sec) + static_cast<double>(now.tv_nsec) * 1e-9;
  return seconds >= kMin && seconds <= now_s + 60.0;
}

/// @brief Convert FFmpeg pts in stream time_base to a wall-clock timespec.
/// Falls back to CLOCK_REALTIME when pts is missing or implausible.
inline timespec pts_to_timespec(const int64_t pts, const AVRational time_base)
{
  if (pts == AV_NOPTS_VALUE || time_base.num <= 0 || time_base.den <= 0) {
    return wallclock_now();
  }

  const double seconds = pts * av_q2d(time_base);
  if (!is_plausible_realtime(seconds)) {
    return wallclock_now();
  }

  timespec ts{};
  ts.tv_sec = static_cast<time_t>(seconds);
  ts.tv_nsec = static_cast<decltype(ts.tv_nsec)>(
    (seconds - static_cast<double>(ts.tv_sec)) * 1e9);
  if (ts.tv_nsec < 0) {
    ts.tv_nsec = 0;
  }
  if (ts.tv_nsec >= 1000000000L) {
    ts.tv_sec += 1;
    ts.tv_nsec -= 1000000000L;
  }
  return ts;
}

}  // namespace timestamps
}  // namespace usb_cam

#endif  // USB_CAM__TIMESTAMPS_HPP_
