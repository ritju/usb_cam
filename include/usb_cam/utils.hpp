// Copyright 2014 Robert Bosch, LLC
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
//    * Neither the name of the Robert Bosch, LLC nor the names of its
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

#ifndef USB_CAM__UTILS_HPP_
#define USB_CAM__UTILS_HPP_

extern "C" {
#include <fcntl.h>  // for open()
}

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>  // for access

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <map>

#include "linux/videodev2.h"

#include "usb_cam/constants.hpp"

namespace fs = std::filesystem;


namespace usb_cam
{
namespace utils
{


/// @brief Read more on IO methods here: https://lwn.net/Articles/240667/
typedef enum
{
  /// @brief read method copies the video frame between user and kernal space
  IO_METHOD_READ,
  /// @brief memory mapped buffers allocated in kernel space
  IO_METHOD_MMAP,
  /// @brief memory buffers allocated in the user space
  IO_METHOD_USERPTR,
  IO_METHOD_UNKNOWN,
} io_method_t;

struct buffer
{
  char * start;
  size_t length;
};


/// @brief Get epoch time shift in microseconds
/// @details Run this at start of process to calculate epoch time shift
/// @ref https://stackoverflow.com/questions/10266451/where-does-v4l2-buffer-timestamp-value-starts-counting
inline time_t get_epoch_time_shift_us()
{
  struct timeval epoch_time;
  struct timespec monotonic_time;

  gettimeofday(&epoch_time, NULL);
  clock_gettime(CLOCK_MONOTONIC, &monotonic_time);

  const int64_t uptime_us =
    monotonic_time.tv_sec * 1000000 + static_cast<int64_t>(
    std::round(monotonic_time.tv_nsec / 1000.0));
  const int64_t epoch_us =
    epoch_time.tv_sec * 1000000 + epoch_time.tv_usec;

  return static_cast<time_t>(epoch_us - uptime_us);
}

/// @brief Calculate image timestamp from buffer time and epoch time shift.
/// In this, the buffer time is first converted into microseconds before the epoch time shift,
/// which is to be given in microseconds is added to it. Afterwards it is split into seconds
/// and nanoseconds for the image timestamp.
inline timespec calc_img_timestamp(const timeval & buffer_time, const time_t & epoch_time_shift_us)
{
  timespec img_timestamp;

  int64_t buffer_time_us = (buffer_time.tv_sec * 1000000) + buffer_time.tv_usec;
  buffer_time_us += epoch_time_shift_us;

  img_timestamp.tv_sec = (buffer_time_us / 1000000);
  img_timestamp.tv_nsec = (buffer_time_us % 1000000) * 1000;

  return img_timestamp;
}

inline int xioctl(int fd, uint64_t request, void * arg)
{
  int r = 0;

  do {
    r = ioctl(fd, request, arg);
    continue;
  } while (-1 == r && EINTR == errno);

  return r;
}


inline io_method_t io_method_from_string(const std::string & str)
{
  if (str == "mmap") {
    return io_method_t::IO_METHOD_MMAP;
  } else if (str == "read") {
    return io_method_t::IO_METHOD_READ;
  } else if (str == "userptr") {
    return io_method_t::IO_METHOD_USERPTR;
  } else {
    return io_method_t::IO_METHOD_UNKNOWN;
  }
}

/// @brief Bytes per pixel the *device* format occupies in a V4L2 buffer.
/// @details Not the ROS output bpp: YUYV/UYVY are 2 B/px packed, RGB332/GREY
/// are 1 B/px. MJPEG is compressed so there is no exact product; 2 B/px is the
/// worst-case uncompressed bound used for the sizeimage sanity limit.
inline uint32_t device_bytes_per_pixel(uint32_t fourcc)
{
  switch (fourcc) {
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_MJPEG:
    case V4L2_PIX_FMT_M420:
    case V4L2_PIX_FMT_Y16:
    case V4L2_PIX_FMT_Y10:
      return 2;
    case V4L2_PIX_FMT_GREY:
    case V4L2_PIX_FMT_RGB332:
      return 1;
    default:
      // Generous ceiling so an unrecognized fourcc still gets a finite bound.
      return 4;
  }
}

/// @brief Expected device-side frame size for a given fourcc and geometry.
inline size_t device_sizeimage_estimate(uint32_t fourcc, uint32_t width, uint32_t height)
{
  return static_cast<size_t>(width) * height * device_bytes_per_pixel(fourcc);
}

/// \brief List currently available valid V4L2 devices
///   Can be used to check if a device string is valid before
///   starting up.
///
///   Inspired by: http://stackoverflow.com/questions/4290834/how-to-get-a-list-of-video-capture-devices-web-cameras-on-linux-ubuntu-c
///
///   Every fd opened here is closed before return. A leaked probe fd is a
///   second handle on a live UVC device and can corrupt the kernel format
///   state (gigabyte sizeimage) when the real capture fd is already streaming.
inline std::map<std::string, v4l2_capability> available_devices(
  const std::string & v4l2_symlinks_dir = "/sys/class/video4linux/",
  const std::string & dev_prefix = "/dev/")
{
  // Initialize vector of device strings to fill in
  std::map<std::string, v4l2_capability> v4l2_devices;

  // Get a list of all v4l2 devices from /sys/class/video4linux
  std::error_code ec;
  if (!fs::is_directory(v4l2_symlinks_dir, ec)) {
    return v4l2_devices;
  }
  for (const auto & device_symlink : fs::directory_iterator(v4l2_symlinks_dir)) {
    if (fs::is_symlink(device_symlink)) {
      // device_str is the full path to the device in /sys/devices/*
      // A broken sysfs symlink must not throw out of this probe path.
      std::string device_name;
      try {
        std::string device_str = fs::canonical(
          v4l2_symlinks_dir + fs::read_symlink(
            device_symlink).generic_string());
        // get the proper device name (e.g. /dev/*)
        std::ifstream uevent_file(device_str + "/uevent");
        std::string line;
        while (std::getline(uevent_file, line)) {
          auto dev_name_index = line.find("DEVNAME=");
          if (dev_name_index != std::string::npos) {
            device_name = dev_prefix + line.substr(dev_name_index + 8, line.size());
            break;
          }
        }
      } catch (const fs::filesystem_error & e) {
        std::cerr << "usb_cam: skipping unreadable v4l2 sysfs entry: " << e.what() << std::endl;
        continue;
      }
      if (device_name.empty()) {
        continue;
      }

      int fd;
      // Try and open device to test access
      if ((fd = open(device_name.c_str(), O_RDONLY)) == -1) {
        std::cerr << "Cannot open device: `" << device_name << "`, ";
        std::cerr << "double-check read / write permissions for device" << std::endl;
      } else {
        struct v4l2_capability device_capabilities = {};
        if (xioctl(fd, VIDIOC_QUERYCAP, &device_capabilities) == -1) {
          std::cerr << "Could not retrieve device capabilities: `" << device_name;
          std::cerr << "`" << std::endl;
        } else {
          v4l2_devices[device_name] = device_capabilities;
        }
        close(fd);
      }
    } else {
      // device doesn't exist, continue
      continue;
    }
  }

  return v4l2_devices;
}

/// @brief ROS log directory (Humble default: $HOME/.ros/log).
inline std::string ros_log_dir()
{
  const char * home = std::getenv("HOME");
  std::string base = (home != nullptr && home[0] != '\0') ? home : "/tmp";
  return base + "/.ros/log";
}

/// @brief Snapshot /proc/self/maps so a later `(deleted)` mapping can be identified.
/// Uses open/read/write only — callable from a signal handler without iostream locks.
inline void copy_proc_maps(const std::string & dest_path)
{
  const int in = open("/proc/self/maps", O_RDONLY);
  if (in < 0) {
    return;
  }
  const int out = open(dest_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (out < 0) {
    close(in);
    return;
  }
  char buf[4096];
  ssize_t n;
  while ((n = read(in, buf, sizeof(buf))) > 0) {
    ssize_t off = 0;
    while (off < n) {
      const ssize_t w = write(out, buf + off, static_cast<size_t>(n - off));
      if (w <= 0) {
        break;
      }
      off += w;
    }
  }
  close(out);
  close(in);
}

/// @brief Write a one-line last-gasp record. Uses only async-signal-safe syscalls
/// when `dir` is a cached C string (see install_crash_handlers).
inline void write_last_gasp(const char * dir, int sig)
{
  if (dir == nullptr || dir[0] == '\0') {
    return;
  }
  char path[512];
  const int n = snprintf(path, sizeof(path), "%s/usb_cam_last_gasp.log", dir);
  if (n <= 0 || n >= static_cast<int>(sizeof(path))) {
    return;
  }
  const int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
  if (fd < 0) {
    return;
  }
  char line[256];
  const int m = snprintf(
    line, sizeof(line), "usb_cam last_gasp sig=%d pid=%d\n",
    sig, static_cast<int>(getpid()));
  if (m > 0) {
    (void)write(fd, line, static_cast<size_t>(m));
  }
  close(fd);

  char maps_path[512];
  const int p = snprintf(maps_path, sizeof(maps_path), "%s/usb_cam_maps_at_crash.txt", dir);
  if (p > 0 && p < static_cast<int>(sizeof(maps_path))) {
    // Best-effort; not strictly async-signal-safe but far more useful than nothing.
    copy_proc_maps(maps_path);
  }
}

/// @brief Cached log dir for the signal handler (path built before the crash).
inline char * crash_log_dir_storage()
{
  static char dir[512] = {0};
  return dir;
}

/// Length of crash_log_dir_storage()'s buffer. sizeof on the returned char*
/// would be pointer width, not the buffer — use this constant instead.
constexpr size_t kCrashLogDirLen = 512;

inline void crash_signal_handler(int sig)
{
  write_last_gasp(crash_log_dir_storage(), sig);
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}

/// @brief Install SIGSEGV/SIGABRT/... handlers that leave a last-gasp record
/// then re-raise with the default action (core dump / normal death).
inline void install_crash_handlers()
{
  char * dir = crash_log_dir_storage();
  const std::string d = ros_log_dir();
  std::strncpy(dir, d.c_str(), kCrashLogDirLen - 1);
  dir[kCrashLogDirLen - 1] = '\0';

  copy_proc_maps(d + "/usb_cam_maps_at_start.txt");

  struct sigaction sa;
  std::memset(&sa, 0, sizeof(sa));
  sa.sa_handler = crash_signal_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESETHAND;
  sigaction(SIGSEGV, &sa, nullptr);
  sigaction(SIGABRT, &sa, nullptr);
  sigaction(SIGBUS, &sa, nullptr);
  sigaction(SIGFPE, &sa, nullptr);
  sigaction(SIGILL, &sa, nullptr);
}

/// @brief Flush libc/RCLCPP buffers on normal exit and process teardown.
inline void flush_logs_on_exit()
{
  std::fflush(nullptr);
}

}  // namespace utils
}  // namespace usb_cam

#endif  // USB_CAM__UTILS_HPP_
