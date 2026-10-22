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

#include <unistd.h>

#include <fstream>
#include <string>

#include "usb_cam/device_preflight.hpp"

TEST(test_device_preflight, sizeimage_bound_uses_device_bpp)
{
  // yuyv422 is 2 bytes/device-pixel; bound = 4 * w * h * 2
  EXPECT_EQ(
    usb_cam::preflight::sizeimage_bound("yuyv422", 640, 480),
    4u * 640u * 480u * 2u);
  // mjpeg worst case treated as 3 bytes/pixel
  EXPECT_EQ(
    usb_cam::preflight::sizeimage_bound("mjpeg", 1280, 1024),
    4u * 1280u * 1024u * 3u);
  // Absolute cap keeps a large request from becoming a huge bound
  EXPECT_EQ(
    usb_cam::preflight::sizeimage_bound("rgb24", 4096, 4096),
    usb_cam::preflight::kAbsSizeimageCap);
  EXPECT_EQ(usb_cam::preflight::sizeimage_bound("mjpeg", 0, 480), 0u);
}

TEST(test_device_preflight, sizeimage_error_rejects_garbage)
{
  const size_t bound = usb_cam::preflight::sizeimage_bound("yuyv422", 640, 480);

  // Healthy value
  EXPECT_EQ(usb_cam::preflight::sizeimage_error(640 * 480 * 2, bound), "");
  // Driver filled 0 is legal (allocator falls back to geometry)
  EXPECT_EQ(usb_cam::preflight::sizeimage_error(0, bound), "");

  // Classic UVC garbage near 2^32 (matches dmesg 4294838624 class)
  EXPECT_NE(usb_cam::preflight::sizeimage_error(4294838624u, bound), "");
  // Plain oversize
  EXPECT_NE(usb_cam::preflight::sizeimage_error(bound + 1, bound), "");
}

TEST(test_device_preflight, wait_device_ready_regular_file_is_ready)
{
  const std::string path = "/tmp/usb_cam_preflight_ready";
  {
    std::ofstream out(path);
    out << "x";
  }
  std::string detail;
  EXPECT_TRUE(usb_cam::preflight::wait_device_ready(path, 200, 10, &detail));
  EXPECT_FALSE(usb_cam::preflight::wait_device_ready(
      "/tmp/usb_cam_preflight_missing_node", 50, 10, &detail));
  EXPECT_NE(detail.find("not ready"), std::string::npos);
  ::unlink(path.c_str());
}

TEST(test_device_preflight, probe_sizeimage_missing_device_fails_cleanly)
{
  uint32_t sizeimage = 0;
  std::string err;
  EXPECT_FALSE(usb_cam::preflight::probe_sizeimage(
      "/tmp/usb_cam_preflight_no_device", &sizeimage, &err));
  EXPECT_FALSE(err.empty());
}

TEST(test_device_preflight, read_vm_peak_is_nonzero)
{
  EXPECT_GT(usb_cam::preflight::read_vm_peak_kb(), 0u);
}
