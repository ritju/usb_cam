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

#include <algorithm>
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

void set_log_level_once()
{
  static bool done = false;
  if (!done) {
    av_log_set_level(AV_LOG_ERROR);
    done = true;
  }
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
  set_log_level_once();
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
      std::cerr << "usb_cam: unknown FFmpeg option ignored: " << e->key << std::endl;
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
      std::cerr << "usb_cam: device negotiated " << stream->codecpar->width << "x"
                << stream->codecpar->height << " (requested " << m_width << "x"
                << m_height << ")" << std::endl;
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

timespec FfmpegCapture::grab(char * dest, size_t dest_size)
{
  if (!m_fmt_ctx || !m_packet || !m_frame) {
    throw std::runtime_error("FfmpegCapture not open");
  }
  if (!m_is_capturing) {
    return timestamps::wallclock_now();
  }
  if (!dest || dest_size == 0) {
    throw std::invalid_argument("grab: empty destination");
  }

  for (int attempt = 0; attempt < 64; ++attempt) {
    const int ret = av_read_frame(m_fmt_ctx, m_packet);
    if (ret < 0) {
      char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
      av_strerror(ret, errbuf, sizeof(errbuf));
      throw std::runtime_error(std::string("av_read_frame failed: ") + errbuf);
    }

    if (m_packet->stream_index != m_video_stream_index) {
      av_packet_unref(m_packet);
      continue;
    }

    if (m_pass_through) {
      const size_t n = std::min(static_cast<size_t>(m_packet->size), dest_size);
      std::memcpy(dest, m_packet->data, n);
      m_last_payload_size = n;
      int64_t ppts = m_packet->pts;
      if (ppts == AV_NOPTS_VALUE) {
        ppts = m_packet->dts;
      }
      const timespec stamp = timestamps::pts_to_timespec(ppts, m_time_base);
      av_packet_unref(m_packet);
      return stamp;
    }

    int ret2 = avcodec_send_packet(m_dec_ctx, m_packet);
    av_packet_unref(m_packet);
    if (ret2 < 0) {
      continue;
    }

    ret2 = avcodec_receive_frame(m_dec_ctx, m_frame);
    if (ret2 == AVERROR(EAGAIN)) {
      continue;
    }
    if (ret2 < 0) {
      throw std::runtime_error("avcodec_receive_frame failed");
    }

    const int src_fmt = m_frame->format;
    int64_t pts = m_frame->best_effort_timestamp;
    if (pts == AV_NOPTS_VALUE) {
      pts = m_frame->pts;
    }
    if (pts == AV_NOPTS_VALUE) {
      pts = m_frame->pkt_dts;
    }
    const timespec stamp = timestamps::pts_to_timespec(pts, m_time_base);

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
          dst_row[x] = static_cast<uint8_t>(sample >> 2);
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
    return stamp;
  }

  throw std::runtime_error("grab: no video frame decoded");
}

}  // namespace usb_cam
