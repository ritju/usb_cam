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

#include <stdexcept>
#include <string>

#include "usb_cam/format_mapping.hpp"

using usb_cam::format_mapping::decode_policy_t;
using usb_cam::format_mapping::resolve_capture_format;
using usb_cam::format_mapping::ros_encoding_from_av_device_format;
using usb_cam::format_mapping::supported_names;

TEST(test_format_mapping, all_supported_names_resolve)
{
  const auto names = supported_names();
  EXPECT_GE(names.size(), 11u);
  for (const auto & name : names) {
    EXPECT_NO_THROW(resolve_capture_format(name)) << name;
  }
}

TEST(test_format_mapping, mjpeg2rgb_preferred_over_av_device_format)
{
  // Deploy YAMLs pair pixel_format=mjpeg2rgb with av_device_format=YUV422P.
  // Capture must use mjpeg, not yuv422p.
  const auto fmt = resolve_capture_format("mjpeg2rgb");
  EXPECT_EQ(fmt.input_format, "mjpeg");
  EXPECT_EQ(fmt.policy, decode_policy_t::DECODE_SWS);
  EXPECT_EQ(fmt.out_pix_fmt, AV_PIX_FMT_RGB24);
  EXPECT_EQ(fmt.ros_encoding, "rgb8");
  EXPECT_EQ(fmt.bytes_per_pixel, 3);
}

TEST(test_format_mapping, raw_names)
{
  EXPECT_EQ(resolve_capture_format("yuyv").input_format, "yuyv422");
  EXPECT_EQ(resolve_capture_format("yuyv").ros_encoding, "yuv422_yuy2");
  EXPECT_EQ(resolve_capture_format("uyvy2rgb").out_pix_fmt, AV_PIX_FMT_RGB24);
  EXPECT_EQ(resolve_capture_format("mono8").ros_encoding, "mono8");
  EXPECT_EQ(resolve_capture_format("mono16").out_pix_fmt, AV_PIX_FMT_GRAY16LE);
  EXPECT_EQ(resolve_capture_format("rgb8").input_format, "rgb24");
  EXPECT_EQ(resolve_capture_format("raw_mjpeg").policy, decode_policy_t::PACKET_PASSTHROUGH);
}

TEST(test_format_mapping, unknown_name_throws)
{
  EXPECT_THROW(resolve_capture_format("bananas"), std::invalid_argument);
  try {
    resolve_capture_format("bananas");
    FAIL();
  } catch (const std::invalid_argument & e) {
    EXPECT_NE(std::string(e.what()).find("bananas"), std::string::npos);
    EXPECT_NE(std::string(e.what()).find("mjpeg2rgb"), std::string::npos);
  }
}

TEST(test_format_mapping, av_device_format_encoding_table)
{
  EXPECT_EQ(ros_encoding_from_av_device_format("YUV422P"), "yuv422");
  EXPECT_EQ(ros_encoding_from_av_device_format("RGB24"), "rgb8");
  EXPECT_EQ(ros_encoding_from_av_device_format("GRAY8"), "mono8");
  EXPECT_EQ(ros_encoding_from_av_device_format("NOPE"), "unknown");
}
