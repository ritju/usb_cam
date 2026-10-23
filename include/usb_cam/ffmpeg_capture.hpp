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

#ifndef USB_CAM__FFMPEG_CAPTURE_HPP_
#define USB_CAM__FFMPEG_CAPTURE_HPP_

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "usb_cam/format_mapping.hpp"
#include "usb_cam/logging.hpp"

namespace usb_cam
{

/// @brief Reject a packet payload before it reaches copy/decode.
/// @return empty string when valid, otherwise a short reason for logging.
inline std::string packet_payload_error(size_t payload_size, size_t bound)
{
  if (payload_size == 0) {
    return "empty payload (size=0)";
  }
  if (bound != 0 && payload_size > bound) {
    return "payload " + std::to_string(payload_size) +
           " exceeds accepted bound " + std::to_string(bound);
  }
  return "";
}

/// @brief Heuristic for a UVC "abnormal data" frame that still has a legal
/// size. Strong signals: every byte identical (0x00 / 0xFF / 0x80 dumps),
/// or a raw_mjpeg payload missing the JPEG SOI marker.
/// @return empty string when the frame looks plausible, else the reason.
inline std::string frame_corruption_reason(
  const char * data, size_t size, bool passthrough_jpeg)
{
  if (data == nullptr || size == 0) {
    return "empty frame buffer";
  }
  if (passthrough_jpeg) {
    // JPEG SOI; a live UVC MJPEG frame must start with FFD8.
    const unsigned char * u = reinterpret_cast<const unsigned char *>(data);
    if (size < 2 || u[0] != 0xFF || u[1] != 0xD8) {
      return "mjpeg payload missing SOI marker";
    }
    return "";
  }
  const unsigned char first = static_cast<unsigned char>(data[0]);
  // Only treat as garbage when the whole frame is one byte value and that
  // value is one of the classic filler patterns. A static scene still has
  // lighting noise, so full-run equality is a strong signal.
  size_t i = 1;
  for (; i < size; ++i) {
    if (static_cast<unsigned char>(data[i]) != first) {
      break;
    }
  }
  if (i == size && (first == 0x00 || first == 0xFF || first == 0x80 || first == 0x7F)) {
    return "frame is solid 0x" + std::to_string(first) + " fill";
  }
  return "";
}

struct capture_config_t
{
  std::string device_path;
  int width{640};
  int height{480};
  double framerate{30.0};
  format_mapping::capture_format_t format;
};

/// @brief FFmpeg-based V4L2 capture. Owns one AVFormatContext (one fd).
/// Includes a recovery watchdog: sustained soft-fails or corrupt frames
/// escalate through flush → stream restart → full reopen (the same
/// S_FMT/STREAMON round-trip an external `ffplay` performs).
class FfmpegCapture
{
public:
  FfmpegCapture();
  ~FfmpegCapture();

  FfmpegCapture(const FfmpegCapture &) = delete;
  FfmpegCapture & operator=(const FfmpegCapture &) = delete;

  /// @brief Open device and negotiate stream/decoder. Throws on failure.
  void open(const capture_config_t & config);

  void close();

  void start_capturing();
  void stop_capturing();
  bool is_capturing() const {return m_is_capturing;}

  /// @brief Grab one frame into dest (tightly packed).
  /// Soft per-frame failures (read error, decode error, empty/oversize payload)
  /// drop the frame, note it, and return false without throwing. Hard
  /// precondition failures (not open, empty dest) still throw.
  /// On success writes the capture timestamp to `stamp` and returns true.
  /// Corrupt-looking frames still copy through but count toward auto-recovery.
  bool grab(char * dest, size_t dest_size, timespec & stamp);

  /// @brief Force one recovery step now (used by tests and the node).
  /// @return true if a recovery action ran.
  bool recover_now();

  /// @brief Route library messages to a sink. Library stays rclcpp-free;
  /// default sink is iostream. nullptr restores the default.
  inline void set_logger(log_fn_t fn)
  {
    m_logger = std::move(fn);
  }

  int width() const {return m_width;}
  int height() const {return m_height;}
  int step() const {return m_step;}
  size_t frame_size() const {return m_frame_size;}
  std::string encoding() const {return m_encoding;}
  const std::string & device_path() const {return m_device_path;}
  /// @brief Payload bytes written by the last grab (JPEG size for passthrough).
  size_t last_payload_size() const {return m_last_payload_size;}
  bool is_passthrough() const {return m_pass_through;}
  size_t dropped_frame_count() const {return m_dropped_frames;}
  size_t recovery_count() const {return m_recoveries;}
  int recovery_level() const {return m_recovery_level;}

private:
  void ensure_sws(const int src_format);
  void drain_stale();
  void log(log_level_t level, const std::string & msg) const;
  void note_dropped_frame(const std::string & err);
  void note_corrupt_frame(const std::string & err);
  void maybe_auto_recover();

  std::string m_device_path;
  capture_config_t m_config{};
  bool m_have_config{false};
  format_mapping::capture_format_t m_format;
  std::string m_encoding;

  AVFormatContext * m_fmt_ctx{nullptr};
  AVCodecContext * m_dec_ctx{nullptr};
  SwsContext * m_sws{nullptr};
  AVFrame * m_frame{nullptr};
  AVPacket * m_packet{nullptr};

  int m_video_stream_index{-1};
  AVRational m_time_base{0, 1};
  int m_width{0};
  int m_height{0};
  int m_step{0};
  size_t m_frame_size{0};
  int m_sws_src_format{-1};
  bool m_is_capturing{false};
  bool m_pass_through{false};
  size_t m_last_payload_size{0};
  size_t m_dropped_frames{0};
  log_fn_t m_logger;
  /// Debounce close→open so frequent switching cannot hammer the UVC device.
  std::chrono::steady_clock::time_point m_last_close{};
  bool m_ever_closed{false};

  /// Watchdog state (auto-recovery).
  int m_soft_fail_streak{0};
  int m_corrupt_streak{0};
  size_t m_recoveries{0};
  int m_recovery_level{0};
  std::chrono::steady_clock::time_point m_last_recovery{};
  /// C4: open-fail backoff so garbage-sizeimage retries cannot thrash OOM.
  int m_open_fail_streak{0};
  std::chrono::steady_clock::time_point m_last_open_fail{};
};

}  // namespace usb_cam

#endif  // USB_CAM__FFMPEG_CAPTURE_HPP_
