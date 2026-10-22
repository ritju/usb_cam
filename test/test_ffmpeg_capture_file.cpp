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

#include <gtest/gtest.h>

extern "C" {
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
}

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "usb_cam/ffmpeg_capture.hpp"
#include "usb_cam/format_mapping.hpp"
#include "usb_cam/timestamps.hpp"

namespace
{

/// @brief Build a tiny YUV4MPEG2 (Y4M) raw video fixture with a gray frame.
std::string write_y4m_fixture(const int width, const int height)
{
  const std::string path = "/tmp/usb_cam_test.y4m";
  std::ofstream out(path, std::ios::binary);
  out << "YUV4MPEG2 W" << width << " H" << height << " F30:1 Ip A1:1 C420jpeg\n";
  out << "FRAME\n";
  std::vector<char> y(static_cast<size_t>(width * height), static_cast<char>(0x80));
  std::vector<char> u(static_cast<size_t>(width * height / 4), static_cast<char>(0x80));
  std::vector<char> v(static_cast<size_t>(width * height / 4), static_cast<char>(0x80));
  out.write(y.data(), y.size());
  out.write(u.data(), u.size());
  out.write(v.data(), v.size());
  return path;
}

}  // namespace

TEST(test_ffmpeg_capture_file, linesize_of_rgb24)
{
  const int w = 64, h = 48;
  const int step = av_image_get_linesize(AV_PIX_FMT_RGB24, w, 0);
  EXPECT_EQ(step, w * 3);
  EXPECT_EQ(static_cast<size_t>(step) * h, 9216u);
}

TEST(test_ffmpeg_capture_file, y4m_fixture_parses_via_ffmpeg)
{
  const int w = 64, h = 48;
  const std::string path = write_y4m_fixture(w, h);

  AVFormatContext * fmt = nullptr;
  int ret = avformat_open_input(&fmt, path.c_str(), nullptr, nullptr);
  ASSERT_GE(ret, 0) << "failed to open fixture " << path;
  ret = avformat_find_stream_info(fmt, nullptr);
  ASSERT_GE(ret, 0);
  const int idx = av_find_best_stream(fmt, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  ASSERT_GE(idx, 0);
  EXPECT_EQ(fmt->streams[idx]->codecpar->width, w);
  EXPECT_EQ(fmt->streams[idx]->codecpar->height, h);
  avformat_close_input(&fmt);
}

TEST(test_ffmpeg_capture_file, file_stream_pts_are_usable)
{
  // Sanity: a realistic v4l2-like time_base conversion
  const AVRational tb{1, 1000000};
  const int64_t pts = 1700000000LL * 1000000LL;
  const timespec ts = usb_cam::timestamps::pts_to_timespec(pts, tb);
  EXPECT_EQ(ts.tv_sec, 1700000000);
}

TEST(test_ffmpeg_capture_file, packet_payload_error_gates_empty_and_oversize)
{
  // Happy path
  EXPECT_EQ(usb_cam::packet_payload_error(1024, 2048), "");

  // Empty payload
  EXPECT_NE(usb_cam::packet_payload_error(0, 2048), "");

  // Oversize passthrough packet must be rejected, not truncated
  EXPECT_NE(usb_cam::packet_payload_error(4096, 2048), "");

  // bound == 0 disables that particular bound
  EXPECT_EQ(usb_cam::packet_payload_error(4096, 0), "");
}

TEST(test_ffmpeg_capture_file, grab_throws_when_not_open)
{
  usb_cam::FfmpegCapture capture;
  char buf[16] = {0};
  timespec stamp{};
  EXPECT_THROW(capture.grab(buf, sizeof(buf), stamp), std::runtime_error);

  // Empty destination is a hard precondition failure too.
  usb_cam::FfmpegCapture capture2;
  EXPECT_THROW(capture2.grab(nullptr, 0, stamp), std::runtime_error);
}

TEST(test_ffmpeg_capture_file, video4linux2_demuxer_registered_after_init)
{
  // Regression: FFmpeg 4.x hides libavdevice grabbers from
  // av_find_input_format until avdevice_register_all() runs. Without that
  // call FfmpegCapture::open always threw "demuxer not available".
  // Constructing FfmpegCapture runs the one-time bring-up.
  usb_cam::FfmpegCapture capture;
  const AVInputFormat * ifmt = av_find_input_format("video4linux2");
  ASSERT_NE(ifmt, nullptr);
  EXPECT_NE(std::string(ifmt->name).find("video4linux2"), std::string::npos);
}
