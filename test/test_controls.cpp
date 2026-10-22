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

#include <string>
#include <vector>

#include "usb_cam/usb_cam.hpp"

TEST(test_controls, all_known_controls_return_false_without_throwing)
{
  usb_cam::UsbCam cam;
  const std::vector<std::string> names = {
    "brightness", "contrast", "saturation", "sharpness", "gain",
    "white_balance_temperature_auto", "white_balance_temperature",
    "exposure_auto", "exposure_absolute", "focus_auto", "focus_absolute",
  };
  for (const auto & name : names) {
    EXPECT_FALSE(cam.set_camera_control(name, 1)) << name;
  }
}

TEST(test_controls, unknown_control_returns_false)
{
  usb_cam::UsbCam cam;
  EXPECT_FALSE(cam.set_camera_control("not_a_control", 42));
}

TEST(test_controls, configure_throws_std_exception_on_bad_device)
{
  // Regression: bare `throw strerror(errno)` (char*) escaped
  // `catch (const std::exception&)` in the node, skipping shutdown().
  usb_cam::UsbCam cam;
  usb_cam::parameters_t params;
  params.device_name = "/dev/usb_cam_definitely_missing_device";
  params.pixel_format_name = "yuyv2rgb";
  params.image_width = 640;
  params.image_height = 480;

  bool caught_std_exception = false;
  try {
    cam.configure(params);
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
    cam2.configure(params);
  } catch (const std::exception & e) {
    caught_std_exception = true;
  } catch (...) {
    FAIL() << "configure threw a non-std::exception type for a non-char path";
  }
  EXPECT_TRUE(caught_std_exception);
}
