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

#ifndef USB_CAM__CRASHLOG_HPP_
#define USB_CAM__CRASHLOG_HPP_

extern "C" {
#include <fcntl.h>  // for open()
}

#include <unistd.h>  // for close/read/write/getpid

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace usb_cam
{
namespace crashlog
{

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
    const ssize_t w = write(fd, line, static_cast<size_t>(m));
    (void)w;
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

}  // namespace crashlog
}  // namespace usb_cam

#endif  // USB_CAM__CRASHLOG_HPP_
