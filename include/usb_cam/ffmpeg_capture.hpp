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

#include <cstdint>
#include <string>

#include "usb_cam/format_mapping.hpp"

namespace usb_cam
{

struct capture_config_t
{
  std::string device_path;
  int width{640};
  int height{480};
  double framerate{30.0};
  format_mapping::capture_format_t format;
};

/// @brief FFmpeg-based V4L2 capture. Owns one AVFormatContext (one fd).
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

  /// @brief Grab one frame into dest (tightly packed). Returns capture timestamp.
  timespec grab(char * dest, size_t dest_size);

  int width() const {return m_width;}
  int height() const {return m_height;}
  int step() const {return m_step;}
  size_t frame_size() const {return m_frame_size;}
  std::string encoding() const {return m_encoding;}
  const std::string & device_path() const {return m_device_path;}
  /// @brief Payload bytes written by the last grab (JPEG size for passthrough).
  size_t last_payload_size() const {return m_last_payload_size;}
  bool is_passthrough() const {return m_pass_through;}

private:
  void ensure_sws(const int src_format);
  void drain_stale();

  std::string m_device_path;
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
};

}  // namespace usb_cam

#endif  // USB_CAM__FFMPEG_CAPTURE_HPP_
