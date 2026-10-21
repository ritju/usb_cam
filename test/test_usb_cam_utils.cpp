// Copyright 2023 Evan Flynn
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
//    * Neither the name of the Evan Flynn nor the names of its
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
#include <libavutil/pixfmt.h>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "usb_cam/utils.hpp"
#include "usb_cam/formats/utils.hpp"
#include "usb_cam/usb_cam.hpp"


using usb_cam::utils::io_method_from_string;


TEST(test_usb_cam_utils, test_io_method_from_string) {
  usb_cam::utils::io_method_t test_io;

  test_io = io_method_from_string("mmap");
  EXPECT_EQ(test_io, usb_cam::utils::IO_METHOD_MMAP);

  test_io = io_method_from_string("read");
  EXPECT_EQ(test_io, usb_cam::utils::IO_METHOD_READ);

  test_io = io_method_from_string("userptr");
  EXPECT_EQ(test_io, usb_cam::utils::IO_METHOD_USERPTR);

  test_io = io_method_from_string("bananas");
  EXPECT_EQ(test_io, usb_cam::utils::IO_METHOD_UNKNOWN);
}

TEST(test_usb_cam_utils, test_clip_value) {
  // Clip values to 0 if -128<=val<0
  for (int i = -128; i < 0; i++) {
    EXPECT_EQ(0, usb_cam::formats::CLIPVALUE(i));
  }
  // Do not clip values between 0<val<255
  // Just return the value as unsigned char
  for (int i = 0; i < 255; i++) {
    EXPECT_EQ(i, usb_cam::formats::CLIPVALUE(i));
  }
  // Clip values to 255 if 255<=val<=383
  for (int i = 255; i <= 383; i++) {
    EXPECT_EQ(255, usb_cam::formats::CLIPVALUE(i));
  }
  // Test outlier cases val < -128 and val > 383
  // these will use the old method (non-array method)
  EXPECT_EQ(0, usb_cam::formats::CLIPVALUE(-129));
  EXPECT_EQ(255, usb_cam::formats::CLIPVALUE(400));
}

TEST(test_usb_cam_utils, test_monotonic_to_real_time) {
  // Get timeval to use for test

  const time_t test_time_t = usb_cam::utils::get_epoch_time_shift_us();

  EXPECT_NE(test_time_t, 0);
}

TEST(test_usb_cam_utils, test_device_sizeimage_estimate) {
  // Packed 4:2:2 formats: 2 bytes per device pixel
  EXPECT_EQ(
    usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_YUYV, 640, 480),
    640u * 480u * 2u);
  EXPECT_EQ(
    usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_UYVY, 640, 480),
    640u * 480u * 2u);

  // MJPEG worst-case bound treated as 2 bytes/pixel
  EXPECT_EQ(
    usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_MJPEG, 1280, 1024),
    1280u * 1024u * 2u);

  // M420: 4:2:0 packed, bounded by 2 bytes/pixel
  EXPECT_EQ(
    usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_M420, 640, 480),
    640u * 480u * 2u);

  // 8-bit mono / RGB332: 1 byte per device pixel
  EXPECT_EQ(
    usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_GREY, 640, 480),
    640u * 480u * 1u);
  EXPECT_EQ(
    usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_RGB332, 640, 480),
    640u * 480u * 1u);

  // 16-bit mono: 2 bytes per device pixel
  EXPECT_EQ(
    usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_Y16, 640, 480),
    640u * 480u * 2u);

  // Packed 10-bit Y10: stored in 2 bytes
  EXPECT_EQ(
    usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_Y10, 640, 480),
    640u * 480u * 2u);

  // Zero dimensions
  EXPECT_EQ(usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_YUYV, 0, 480), 0u);
  EXPECT_EQ(usb_cam::utils::device_sizeimage_estimate(V4L2_PIX_FMT_YUYV, 640, 0), 0u);

  // Unknown fourcc falls back to 4 bytes/pixel (generous ceiling)
  EXPECT_EQ(
    usb_cam::utils::device_sizeimage_estimate(0xdeadbeefu, 640, 480),
    640u * 480u * 4u);
}

namespace
{

int count_open_fds()
{
  DIR * dir = opendir("/proc/self/fd");
  if (dir == nullptr) {
    return -1;
  }
  int count = 0;
  while (readdir(dir) != nullptr) {
    ++count;
  }
  closedir(dir);
  // readdir also yields "." and ".."
  return count - 2;
}

}  // namespace

TEST(test_usb_cam_utils, test_available_devices_closes_probe_fds) {
  // Fake sysfs + dev tree so the probe opens a regular file and must close it.
  const std::string root = "/tmp/usb_cam_fd_leak_test";
  const std::string sysfs = root + "/sys/";
  const std::string device_dir = root + "/devices/fake0/";
  const std::string dev_prefix = root + "/dev/";

  std::filesystem::remove_all(root);
  std::filesystem::create_directories(sysfs);
  std::filesystem::create_directories(device_dir);
  std::filesystem::create_directories(dev_prefix);

  {
    std::ofstream uevent(device_dir + "uevent");
    uevent << "DEVNAME=fakevid0\n";
  }
  {
    std::ofstream dev(dev_prefix + "fakevid0");
    dev << "x";
  }
  std::filesystem::create_directory_symlink(
    std::filesystem::path("../devices/fake0"), std::filesystem::path(sysfs + "video0"));

  const int before = count_open_fds();
  ASSERT_GT(before, 0);

  // Run several times; a leak grows the count by one open per device per call.
  for (int i = 0; i < 5; ++i) {
    auto devices = usb_cam::utils::available_devices(sysfs, dev_prefix);
    (void)devices;  // fake file is not a V4L2 node; QUERYCAP fails, fd must still close
    const int after = count_open_fds();
    EXPECT_EQ(after, before) << "available_devices leaked an fd on iteration " << i;
  }

  std::filesystem::remove_all(root);
}

TEST(test_usb_cam_utils, test_configure_throws_std_exception_on_bad_device) {
  // Regression for H4: bare `throw strerror(errno)` (char*) escaped
  // `catch (const std::exception&)` in the node, skipping shutdown().
  usb_cam::UsbCam cam;
  usb_cam::parameters_t params;
  params.device_name = "/dev/usb_cam_definitely_missing_device";
  params.pixel_format_name = "yuyv2rgb";
  params.image_width = 640;
  params.image_height = 480;

  bool caught_std_exception = false;
  try {
    cam.configure(params, usb_cam::utils::IO_METHOD_MMAP);
  } catch (const std::exception & e) {
    caught_std_exception = true;
    EXPECT_NE(std::string(e.what()).size(), 0u);
  } catch (...) {
    FAIL() << "configure threw a non-std::exception type";
  }
  EXPECT_TRUE(caught_std_exception);

  // A non-character path must also throw std::exception, not char*.
  usb_cam::UsbCam cam2;
  params.device_name = "/tmp";
  caught_std_exception = false;
  try {
    cam2.configure(params, usb_cam::utils::IO_METHOD_MMAP);
  } catch (const std::exception & e) {
    caught_std_exception = true;
  } catch (...) {
    FAIL() << "configure threw a non-std::exception type for a non-char path";
  }
  EXPECT_TRUE(caught_std_exception);
}

TEST(test_usb_cam_utils, test_frame_payload_error) {
  // Happy path
  EXPECT_EQ(usb_cam::frame_payload_error(0, 4, 1024, 2048, 0, 4096), "");

  // Index out of range
  EXPECT_NE(usb_cam::frame_payload_error(4, 4, 1024, 2048, 0, 4096), "");

  // Driver error flag
  EXPECT_NE(
    usb_cam::frame_payload_error(0, 4, 1024, 2048, V4L2_BUF_FLAG_ERROR, 4096), "");

  // Empty payload
  EXPECT_NE(usb_cam::frame_payload_error(0, 4, 0, 2048, 0, 4096), "");

  // bytesused > length
  EXPECT_NE(usb_cam::frame_payload_error(0, 4, 3000, 2048, 0, 4096), "");

  // bytesused > accepted bound
  EXPECT_NE(usb_cam::frame_payload_error(0, 4, 5000, 8000, 0, 4096), "");

  // max_buffer_bytes == 0 disables that particular bound
  EXPECT_EQ(usb_cam::frame_payload_error(0, 4, 5000, 8000, 0, 0), "");
}

TEST(test_usb_cam_utils, test_throttle_log_levels) {
  // The node bridge maps *_THROTTLE to RCLCPP_*_THROTTLE with this window.
  EXPECT_EQ(usb_cam::kLogThrottleMs, 5000);

  // Throttle variants must be distinct from their plain counterparts so the
  // node switch cannot silently collapse them.
  EXPECT_NE(
    static_cast<int>(usb_cam::log_level_t::DEBUG_THROTTLE),
    static_cast<int>(usb_cam::log_level_t::DEBUG));
  EXPECT_NE(
    static_cast<int>(usb_cam::log_level_t::INFO_THROTTLE),
    static_cast<int>(usb_cam::log_level_t::INFO));
  EXPECT_NE(
    static_cast<int>(usb_cam::log_level_t::WARN_THROTTLE),
    static_cast<int>(usb_cam::log_level_t::WARN));
}

TEST(test_usb_cam_utils, test_set_logger_routes_messages) {
  usb_cam::UsbCam cam;
  std::vector<std::pair<usb_cam::log_level_t, std::string>> captured;
  cam.set_logger(
    [&captured](usb_cam::log_level_t level, const std::string & msg) {
      captured.emplace_back(level, msg);
    });

  // set_pixel_format on an unknown format logs the supported list, then throws.
  usb_cam::parameters_t params;
  params.pixel_format_name = "not_a_real_format";
  params.device_name = "/dev/null";
  try {
    cam.configure(params, usb_cam::utils::IO_METHOD_MMAP);
  } catch (const std::exception &) {
  }

  ASSERT_FALSE(captured.empty());
  bool saw_error = false;
  for (const auto & entry : captured) {
    EXPECT_FALSE(entry.second.empty());
    if (entry.first == usb_cam::log_level_t::ERROR) {
      saw_error = true;
    }
  }
  EXPECT_TRUE(saw_error);

  // Without a logger, logging must not crash.
  usb_cam::UsbCam bare;
  bare.set_logger(nullptr);
  // configure will fail on the device path; just ensure no terminate.
  try {
    bare.configure(params, usb_cam::utils::IO_METHOD_MMAP);
  } catch (const std::exception &) {
  }
}
