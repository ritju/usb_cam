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

#include <filesystem>
#include <fstream>
#include <string>

#include "usb_cam/crashlog.hpp"

TEST(test_crashlog, test_last_gasp_and_maps_snapshot) {
  const std::string dir = "/tmp/usb_cam_crashlog_test";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  usb_cam::crashlog::write_last_gasp(dir.c_str(), 11);
  ASSERT_TRUE(std::filesystem::exists(dir + "/usb_cam_last_gasp.log"));
  {
    std::ifstream in(dir + "/usb_cam_last_gasp.log");
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("sig=11"), std::string::npos);
    EXPECT_NE(content.find("last_gasp"), std::string::npos);
  }

  ASSERT_TRUE(std::filesystem::exists(dir + "/usb_cam_maps_at_crash.txt"));
  {
    std::ifstream in(dir + "/usb_cam_maps_at_crash.txt");
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    // /proc/self/maps always has at least one mapping line
    EXPECT_NE(content.find('\n'), std::string::npos);
  }

  // Second call appends, does not truncate
  usb_cam::crashlog::write_last_gasp(dir.c_str(), 6);
  {
    std::ifstream in(dir + "/usb_cam_last_gasp.log");
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_NE(content.find("sig=11"), std::string::npos);
    EXPECT_NE(content.find("sig=6"), std::string::npos);
  }

  std::filesystem::remove_all(dir);
}

TEST(test_crashlog, test_ros_log_dir_default) {
  const std::string d = usb_cam::crashlog::ros_log_dir();
  EXPECT_FALSE(d.empty());
  EXPECT_NE(d.find("/.ros/log"), std::string::npos);
}

TEST(test_crashlog, test_install_crash_handlers_caches_full_path) {
  usb_cam::crashlog::install_crash_handlers();
  const std::string cached = usb_cam::crashlog::crash_log_dir_storage();
  // Regression: sizeof(char*) once truncated this to 7 bytes ("/home/s").
  EXPECT_GT(cached.size(), 16u);
  EXPECT_EQ(cached, usb_cam::crashlog::ros_log_dir());
  EXPECT_NE(cached.find("/.ros/log"), std::string::npos);
}
