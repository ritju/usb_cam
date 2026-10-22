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

#include <string>
#include <utility>
#include <vector>

#include "usb_cam/logging.hpp"
#include "usb_cam/usb_cam.hpp"

TEST(test_logging, test_throttle_log_levels) {
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

TEST(test_logging, test_set_logger_routes_messages) {
  usb_cam::UsbCam cam;
  std::vector<std::pair<usb_cam::log_level_t, std::string>> captured;
  cam.set_logger(
    [&captured](usb_cam::log_level_t level, const std::string & msg) {
      captured.emplace_back(level, msg);
    });

  // configure on an unknown format logs the supported list, then throws.
  usb_cam::parameters_t params;
  params.pixel_format_name = "not_a_real_format";
  params.device_name = "/dev/null";
  try {
    cam.configure(params);
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
    bare.configure(params);
  } catch (const std::exception &) {
  }
}
