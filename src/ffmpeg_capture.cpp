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

#include "usb_cam/ffmpeg_capture.hpp"

extern "C" {
#include <libavdevice/avdevice.h>
}

#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include "usb_cam/timestamps.hpp"

namespace usb_cam
{

namespace
{

/// One-time FFmpeg bring-up. FFmpeg 4.x does not expose libavdevice's
/// grabbers (video4linux2, x11grab, ...) through av_find_input_format
/// until avdevice_register_all() has run; FFmpeg 5+ registers them
/// automatically and dropped the symbol.
void init_ffmpeg_once()
{
  static bool done = false;
  if (done) {
    return;
  }
  av_log_set_level(AV_LOG_ERROR);
#if LIBAVDEVICE_VERSION_INT < AV_VERSION_INT(59, 0, 0)
  avdevice_register_all();
#endif
  done = true;
}

std::string framerate_to_string(const double framerate)
{
  // Prefer integer form when close enough; FFmpeg accepts "30" or "30000/1001"
  const int rounded = static_cast<int>(framerate + 0.5);
  if (rounded > 0 && std::abs(framerate - rounded) < 0.01) {
    return std::to_string(rounded);
  }
  return std::to_string(framerate);
}

}  // namespace

FfmpegCapture::FfmpegCapture()
{
  init_ffmpeg_once();
}

FfmpegCapture::~FfmpegCapture()
{
  close();
}

void FfmpegCapture::open(const capture_config_t & config)
{
  close();

  m_device_path = config.device_path;
  m_format = config.format;
  m_encoding = m_format.ros_encoding;
  m_width = config.width;
  m_height = config.height;
  m_pass_through =
    (m_format.policy == format_mapping::decode_policy_t::PACKET_PASSTHROUGH);

  // FFmpeg 4.x avformat_open_input takes a non-const AVInputFormat*
  AVInputFormat * ifmt = const_cast<AVInputFormat *>(av_find_input_format("video4linux2"));
  if (!ifmt) {
    throw std::runtime_error("FFmpeg video4linux2 demuxer not available");
  }

  AVDictionary * opts = nullptr;
  av_dict_set(
    &opts, "video_size",
    (std::to_string(config.width) + "x" + std::to_string(config.height)).c_str(), 0);
  av_dict_set(&opts, "input_format", m_format.input_format.c_str(), 0);
  av_dict_set(&opts, "pixel_format", m_format.input_format.c_str(), 0);
  av_dict_set(&opts, "framerate", framerate_to_string(config.framerate).c_str(), 0);
  av_dict_set(&opts, "timestamps", "mono2abs", 0);
  av_dict_set(&opts, "probesize", "32768", 0);
  av_dict_set(&opts, "analyzeduration", "0", 0);

  int ret = avformat_open_input(&m_fmt_ctx, m_device_path.c_str(), ifmt, &opts);
  if (opts) {
    AVDictionaryEntry * e = nullptr;
    while ((e = av_dict_get(opts, "", e, AV_DICT_IGNORE_SUFFIX))) {
      log(log_level_t::WARN, std::string("unknown FFmpeg option ignored: ") + e->key);
    }
    av_dict_free(&opts);
  }
  if (ret < 0) {
    char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
    av_strerror(ret, errbuf, sizeof(errbuf));
    throw std::runtime_error(
            "Failed to open `" + m_device_path + "` with input_format `" +
            m_format.input_format + "`: " + errbuf);
  }

  ret = avformat_find_stream_info(m_fmt_ctx, nullptr);
  if (ret < 0) {
    close();
    throw std::runtime_error("Failed to find stream info for " + m_device_path);
  }

  m_video_stream_index = av_find_best_stream(m_fmt_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if (m_video_stream_index < 0) {
    close();
    throw std::runtime_error("No video stream in " + m_device_path);
  }

  AVStream * stream = m_fmt_ctx->streams[m_video_stream_index];
  m_time_base = stream->time_base;

  // Re-read negotiated geometry (driver may adjust)
  if (stream->codecpar->width > 0 && stream->codecpar->height > 0) {
    if (stream->codecpar->width != m_width || stream->codecpar->height != m_height) {
      log(
        log_level_t::INFO,
        "device negotiated " + std::to_string(stream->codecpar->width) + "x" +
        std::to_string(stream->codecpar->height) + " (requested " +
        std::to_string(m_width) + "x" + std::to_string(m_height) + ")");
      m_width = stream->codecpar->width;
      m_height = stream->codecpar->height;
    }
  }

  if (!m_pass_through) {
    const AVCodec * codec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec) {
      close();
      throw std::runtime_error(
              "No decoder for codec id " +
              std::to_string(stream->codecpar->codec_id));
    }
    m_dec_ctx = avcodec_alloc_context3(codec);
    if (!m_dec_ctx) {
      close();
      throw std::runtime_error("avcodec_alloc_context3 failed");
    }
    ret = avcodec_parameters_to_context(m_dec_ctx, stream->codecpar);
    if (ret < 0) {
      close();
      throw std::runtime_error("avcodec_parameters_to_context failed");
    }
    ret = avcodec_open2(m_dec_ctx, codec, nullptr);
    if (ret < 0) {
      close();
      throw std::runtime_error("avcodec_open2 failed");
    }
  }

  m_frame = av_frame_alloc();
  m_packet = av_packet_alloc();
  if (!m_frame || !m_packet) {
    close();
    throw std::runtime_error("Failed to allocate frame/packet");
  }

  const int out_fmt = static_cast<int>(m_format.out_pix_fmt);
  if (out_fmt != AV_PIX_FMT_NONE) {
    m_step = av_image_get_linesize(m_format.out_pix_fmt, m_width, 0);
    if (m_step <= 0) {
      close();
      throw std::runtime_error("Invalid output linesize");
    }
    m_frame_size = static_cast<size_t>(m_step) * static_cast<size_t>(m_height);
  } else {
    // passthrough: caller sizes buffer; keep a conservative step
    m_step = m_width * 3;
    m_frame_size = static_cast<size_t>(m_step) * static_cast<size_t>(m_height);
  }

  m_is_capturing = true;
  drain_stale();
}

void FfmpegCapture::close()
{
  m_is_capturing = false;
  if (m_sws) {
    sws_freeContext(m_sws);
    m_sws = nullptr;
  }
  if (m_packet) {
    av_packet_free(&m_packet);
  }
  if (m_frame) {
    av_frame_free(&m_frame);
  }
  if (m_dec_ctx) {
    avcodec_free_context(&m_dec_ctx);
  }
  if (m_fmt_ctx) {
    avformat_close_input(&m_fmt_ctx);
  }
  m_video_stream_index = -1;
  m_sws_src_format = -1;
  m_step = 0;
  m_frame_size = 0;
}

void FfmpegCapture::start_capturing()
{
  if (m_is_capturing) {
    return;
  }
  m_is_capturing = true;
  drain_stale();
}

void FfmpegCapture::stop_capturing()
{
  m_is_capturing = false;
  if (m_fmt_ctx) {
    avformat_flush(m_fmt_ctx);
  }
}

void FfmpegCapture::drain_stale()
{
  if (!m_fmt_ctx || !m_packet) {
    return;
  }
  // Drop a few buffered packets so the next grab is fresh
  for (int i = 0; i < 3; ++i) {
    if (av_read_frame(m_fmt_ctx, m_packet) < 0) {
      break;
    }
    av_packet_unref(m_packet);
  }
}

void FfmpegCapture::ensure_sws(const int src_format)
{
  if (m_sws && m_sws_src_format == src_format) {
    return;
  }
  if (m_sws) {
    sws_freeContext(m_sws);
    m_sws = nullptr;
  }
  m_sws = sws_getContext(
    m_width, m_height, static_cast<AVPixelFormat>(src_format),
    m_width, m_height, m_format.out_pix_fmt,
    SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
  if (!m_sws) {
    throw std::runtime_error("sws_getContext failed for src format " + std::to_string(src_format));
  }
  m_sws_src_format = src_format;
}

void FfmpegCapture::log(log_level_t level, const std::string & msg) const
{
  if (m_logger) {
    m_logger(level, msg);
    return;
  }
  const char * prefix = "usb_cam: ";
  switch (level) {
    case log_level_t::DEBUG:
    case log_level_t::DEBUG_THROTTLE:
      std::cout << prefix << msg << std::endl;
      break;
    case log_level_t::INFO:
    case log_level_t::INFO_THROTTLE:
      std::cout << prefix << msg << std::endl;
      break;
    case log_level_t::WARN:
    case log_level_t::WARN_THROTTLE:
      std::cerr << prefix << "WARN: " << msg << std::endl;
      break;
    case log_level_t::ERROR:
      std::cerr << prefix << "ERROR: " << msg << std::endl;
      break;
  }
}

void FfmpegCapture::note_dropped_frame(const std::string & err)
{
  ++m_dropped_frames;
  // First drop and every 30th go to ERROR so long drop runs stay visible;
  // the rest are throttled warnings.
  if (m_dropped_frames == 1 || (m_dropped_frames % 30) == 0) {
    log(
      log_level_t::ERROR,
      "dropping frame (" + std::to_string(m_dropped_frames) + " total): " + err);
  } else {
    log(log_level_t::WARN_THROTTLE, "dropping frame: " + err);
  }
}

bool FfmpegCapture::grab(char * dest, size_t dest_size, timespec & stamp)
{
  if (!m_fmt_ctx || !m_packet || !m_frame) {
    throw std::runtime_error("FfmpegCapture not open");
  }
  if (!m_is_capturing) {
    stamp = timestamps::wallclock_now();
    return false;
  }
  if (!dest || dest_size == 0) {
    throw std::invalid_argument("grab: empty destination");
  }

  for (int attempt = 0; attempt < 64; ++attempt) {
    const int ret = av_read_frame(m_fmt_ctx, m_packet);
    if (ret < 0) {
      char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
      av_strerror(ret, errbuf, sizeof(errbuf));
      note_dropped_frame(std::string("av_read_frame failed: ") + errbuf);
      stamp = timestamps::wallclock_now();
      return false;
    }

    if (m_packet->stream_index != m_video_stream_index) {
      av_packet_unref(m_packet);
      continue;
    }

    if (m_pass_through) {
      const size_t n = static_cast<size_t>(m_packet->size);
      const std::string payload_err = packet_payload_error(n, dest_size);
      if (!payload_err.empty() || m_packet->data == nullptr) {
        note_dropped_frame(
          payload_err.empty() ? "passthrough packet has null data" : payload_err);
        av_packet_unref(m_packet);
        stamp = timestamps::wallclock_now();
        return false;
      }
      std::memcpy(dest, m_packet->data, n);
      m_last_payload_size = n;
      int64_t ppts = m_packet->pts;
      if (ppts == AV_NOPTS_VALUE) {
        ppts = m_packet->dts;
      }
      stamp = timestamps::pts_to_timespec(ppts, m_time_base);
      av_packet_unref(m_packet);
      return true;
    }

    // Send the packet; EAGAIN means the decoder is full and we must drain
    // output before the packet can be accepted. Keep the packet alive until
    // it is accepted or explicitly dropped so nothing is lost silently.
    int ret2 = avcodec_send_packet(m_dec_ctx, m_packet);
    if (ret2 == AVERROR(EAGAIN)) {
      ret2 = avcodec_receive_frame(m_dec_ctx, m_frame);
      if (ret2 == 0) {
        // Decoder drained one earlier frame; resend the held packet now
        // that there is room, then fall through to process the drained frame.
        const int resend = avcodec_send_packet(m_dec_ctx, m_packet);
        if (resend < 0 && resend != AVERROR(EAGAIN)) {
          char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
          av_strerror(resend, errbuf, sizeof(errbuf));
          note_dropped_frame(std::string("avcodec_send_packet resend failed: ") + errbuf);
        }
        av_packet_unref(m_packet);
        // Process m_frame below.
      } else {
        av_packet_unref(m_packet);
        if (ret2 == AVERROR(EAGAIN)) {
          // Decoder wants more input but refused this packet: drop and retry.
          note_dropped_frame("decoder refused packet (EAGAIN on send and receive)");
          continue;
        }
        char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
        av_strerror(ret2, errbuf, sizeof(errbuf));
        note_dropped_frame(std::string("avcodec_receive_frame failed: ") + errbuf);
        stamp = timestamps::wallclock_now();
        return false;
      }
    } else if (ret2 < 0) {
      av_packet_unref(m_packet);
      char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
      av_strerror(ret2, errbuf, sizeof(errbuf));
      note_dropped_frame(std::string("avcodec_send_packet failed: ") + errbuf);
      stamp = timestamps::wallclock_now();
      return false;
    } else {
      av_packet_unref(m_packet);
      ret2 = avcodec_receive_frame(m_dec_ctx, m_frame);
      if (ret2 == AVERROR(EAGAIN)) {
        continue;
      }
      if (ret2 < 0) {
        char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
        av_strerror(ret2, errbuf, sizeof(errbuf));
        note_dropped_frame(std::string("avcodec_receive_frame failed: ") + errbuf);
        stamp = timestamps::wallclock_now();
        return false;
      }
    }

    const int src_fmt = m_frame->format;
    int64_t pts = m_frame->best_effort_timestamp;
    if (pts == AV_NOPTS_VALUE) {
      pts = m_frame->pts;
    }
    if (pts == AV_NOPTS_VALUE) {
      pts = m_frame->pkt_dts;
    }
    stamp = timestamps::pts_to_timespec(pts, m_time_base);

    if (m_format.policy == format_mapping::decode_policy_t::Y10_PACK) {
      // gray16le samples carry 10-bit values; keep the low 10 bits >> 2
      for (int y = 0; y < m_height; ++y) {
        const uint8_t * src_row = m_frame->data[0] +
          static_cast<size_t>(y) * static_cast<size_t>(m_frame->linesize[0]);
        uint8_t * dst_row = reinterpret_cast<uint8_t *>(dest) +
          static_cast<size_t>(y) * static_cast<size_t>(m_step);
        for (int x = 0; x < m_width; ++x) {
          const uint16_t sample = static_cast<uint16_t>(src_row[2 * x]) |
            (static_cast<uint16_t>(src_row[2 * x + 1]) << 8);
          dst_row[x] = static_cast<uint8_t>((sample & 0x3FF) >> 2);
        }
      }
    } else if (src_fmt == static_cast<int>(m_format.out_pix_fmt)) {
      // Same format: copy rows handling linesize padding
      const int out_step = m_step;
      for (int y = 0; y < m_height; ++y) {
        std::memcpy(
          dest + static_cast<size_t>(y) * static_cast<size_t>(out_step),
          m_frame->data[0] + static_cast<size_t>(y) * static_cast<size_t>(m_frame->linesize[0]),
          static_cast<size_t>(out_step));
      }
    } else {
      ensure_sws(src_fmt);
      uint8_t * dst_data[4] = {nullptr, nullptr, nullptr, nullptr};
      int dst_linesize[4] = {0, 0, 0, 0};
      av_image_fill_arrays(
        dst_data, dst_linesize,
        reinterpret_cast<uint8_t *>(dest), m_format.out_pix_fmt, m_width, m_height, 1);
      sws_scale(
        m_sws, m_frame->data, m_frame->linesize, 0, m_height,
        dst_data, dst_linesize);
    }

    av_frame_unref(m_frame);
    m_last_payload_size = m_frame_size;
    return true;
  }

  note_dropped_frame("no video frame decoded after 64 read attempts");
  stamp = timestamps::wallclock_now();
  return false;
}

}  // namespace usb_cam
