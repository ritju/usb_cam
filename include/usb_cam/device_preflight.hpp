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

#ifndef USB_CAM__DEVICE_PREFLIGHT_HPP_
#define USB_CAM__DEVICE_PREFLIGHT_HPP_

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace usb_cam
{
namespace preflight
{

/// Same multiplier sizeimage-guard used against a device fourcc estimate.
constexpr size_t kMaxSizeimageFactor = 4;

/// Absolute ceiling for a single driver buffer. Healthy MJPEG/RAW frames are
/// a few MB; UVC garbage (dwMaxVideoFrameSize / sizeimage) lands near 4 GiB.
constexpr size_t kAbsSizeimageCap = 64u * 1024u * 1024u;

/// Device-side bytes/pixel for the capture input_format name (not ROS bpp).
inline size_t device_bpp_for_input_format(const std::string & input_format)
{
  if (input_format == "mjpeg") {
    return 3;  // JPEG worst case ~ uncompressed
  }
  if (input_format == "yuv420p") {
    return 2;
  }
  if (input_format == "gray") {
    return 1;
  }
  if (input_format == "gray16le" || input_format == "yuyv422" || input_format == "uyvy422") {
    return 2;
  }
  return 3;
}

/// @brief Plausible sizeimage bound for the requested geometry.
inline size_t sizeimage_bound(const std::string & input_format, int width, int height)
{
  if (width <= 0 || height <= 0) {
    return 0;
  }
  const size_t expected =
    static_cast<size_t>(width) * static_cast<size_t>(height) *
    device_bpp_for_input_format(input_format);
  const size_t bound = expected * kMaxSizeimageFactor;
  return bound < kAbsSizeimageCap ? bound : kAbsSizeimageCap;
}

/// @brief Reject a driver-reported sizeimage before any allocation.
/// @return empty string when acceptable, otherwise the reason.
inline std::string sizeimage_error(uint32_t sizeimage, size_t bound)
{
  if (bound == 0) {
    return "";
  }
  if (sizeimage == 0) {
    // Driver filled 0: legal, allocator will use geometry. Not a reject.
    return "";
  }
  // Classic UVC descriptor garbage: value near 2^32-epsilon.
  if (sizeimage > 0xF0000000u) {
    return "sizeimage " + std::to_string(sizeimage) + " looks like uint32 garbage";
  }
  if (static_cast<size_t>(sizeimage) > bound) {
    return "sizeimage " + std::to_string(sizeimage) +
           " exceeds accepted bound " + std::to_string(bound);
  }
  return "";
}

/// @brief VmPeak of this process in kB (diagnostic). Returns 0 on failure.
inline size_t read_vm_peak_kb()
{
  FILE * f = fopen("/proc/self/status", "r");
  if (!f) {
    return 0;
  }
  char line[256];
  size_t value = 0;
  while (fgets(line, sizeof(line), f)) {
    if (std::strncmp(line, "VmPeak:", 7) == 0) {
      unsigned long kb = 0;
      if (std::sscanf(line + 7, "%lu", &kb) == 1) {
        value = static_cast<size_t>(kb);
      }
      break;
    }
  }
  fclose(f);
  return value;
}

/// @brief True when path exists as a character device and keeps a stable
/// inode/size across `stable_ms`. Non-/dev paths (test fixtures) return true
/// as soon as the file exists so unit tests and file sources still work.
inline bool wait_device_ready(
  const std::string & path, int timeout_ms = 2000, int stable_ms = 100,
  std::string * detail = nullptr)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  struct stat last {};
  bool have_last = false;
  auto last_change = std::chrono::steady_clock::now();

  while (true) {
    struct stat st {};
    const bool exists = (::stat(path.c_str(), &st) == 0);

    if (!exists) {
      if (detail) {
        *detail = path + " does not exist yet";
      }
      have_last = false;
    } else {
      const bool is_chr = S_ISCHR(st.st_mode);
      // Non-character paths (regular files used as test sources) are ready
      // once present. Character nodes must be /dev nodes and stay stable.
      if (!is_chr) {
        return true;
      }
      if (!have_last || st.st_ino != last.st_ino || st.st_size != last.st_size) {
        last = st;
        have_last = true;
        last_change = std::chrono::steady_clock::now();
      } else if (std::chrono::steady_clock::now() - last_change >=
        std::chrono::milliseconds(stable_ms))
      {
        return true;
      }
    }

    if (std::chrono::steady_clock::now() >= deadline) {
      if (detail) {
        *detail = path + " not ready within " + std::to_string(timeout_ms) + " ms";
      }
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

/// @brief One-shot read-only VIDIOC_G_FMT probe. Opens and closes its own fd;
/// never streams. On success writes driver sizeimage and returns true.
inline bool probe_sizeimage(const std::string & path, uint32_t * sizeimage, std::string * err)
{
  const int fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    if (err) {
      *err = "probe open failed for " + path;
    }
    return false;
  }
  struct v4l2_format fmt {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  const int rc = ::ioctl(fd, VIDIOC_G_FMT, &fmt);
  const int saved = errno;
  ::close(fd);
  if (rc < 0) {
    if (err) {
      *err = "VIDIOC_G_FMT failed on " + path + ": " + std::strerror(saved);
    }
    return false;
  }
  if (sizeimage) {
    *sizeimage = fmt.fmt.pix.sizeimage;
  }
  return true;
}

/// @brief Map our FFmpeg input_format name to a V4L2 fourcc for S_FMT.
inline uint32_t v4l2_fourcc_for_input_format(const std::string & input_format)
{
  if (input_format == "mjpeg") {
    return V4L2_PIX_FMT_MJPEG;
  }
  if (input_format == "yuyv422") {
    return V4L2_PIX_FMT_YUYV;
  }
  if (input_format == "uyvy422") {
    return V4L2_PIX_FMT_UYVY;
  }
  if (input_format == "gray") {
    return V4L2_PIX_FMT_GREY;
  }
  if (input_format == "gray16le") {
    return V4L2_PIX_FMT_Y16;
  }
  if (input_format == "rgb24") {
    return V4L2_PIX_FMT_RGB24;
  }
  if (input_format == "yuv420p") {
    return V4L2_PIX_FMT_YUV420;
  }
  return 0;
}

/// @brief `ffplay`-equivalent kick: VIDIOC_S_FMT with the requested geometry
/// forces the UVC driver to recompute sizeimage, then G_FMT reads it back.
/// This is what clears a stuck/garbage dwMaxVideoFrameSize that a plain
/// G_FMT would keep reporting. Opens and closes its own fd; never streams.
/// @return true when the re-read sizeimage is present (caller still bounds it).
inline bool kick_s_fmt_and_probe(
  const std::string & path, int width, int height, const std::string & input_format,
  uint32_t * sizeimage, std::string * err)
{
  const int fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    if (err) {
      *err = "kick open failed for " + path;
    }
    return false;
  }
  struct v4l2_format fmt {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = static_cast<__u32>(width);
  fmt.fmt.pix.height = static_cast<__u32>(height);
  fmt.fmt.pix.pixelformat = v4l2_fourcc_for_input_format(input_format);
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  // Zero sizeimage before S_FMT, same hygiene sizeimage-guard used, so the
  // driver recomputes rather than inheriting a smeared value.
  fmt.fmt.pix.sizeimage = 0;
  if (fmt.fmt.pix.pixelformat == 0) {
    ::close(fd);
    if (err) {
      *err = "no V4L2 fourcc for input_format " + input_format;
    }
    return false;
  }

  int rc = ::ioctl(fd, VIDIOC_S_FMT, &fmt);
  if (rc < 0) {
    // TRY_FMT is a softer kick if the device refuses S_FMT while busy.
    rc = ::ioctl(fd, VIDIOC_TRY_FMT, &fmt);
  }
  const int saved = errno;
  if (rc < 0) {
    ::close(fd);
    if (err) {
      *err = "VIDIOC_S_FMT/TRY_FMT failed on " + path + ": " + std::strerror(saved);
    }
    return false;
  }

  struct v4l2_format out {};
  out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rc = ::ioctl(fd, VIDIOC_G_FMT, &out);
  const int saved2 = errno;
  ::close(fd);
  if (rc < 0) {
    if (err) {
      *err = "VIDIOC_G_FMT after kick failed on " + path + ": " + std::strerror(saved2);
    }
    return false;
  }
  if (sizeimage) {
    *sizeimage = out.fmt.pix.sizeimage;
  }
  return true;
}

}  // namespace preflight
}  // namespace usb_cam

#endif  // USB_CAM__DEVICE_PREFLIGHT_HPP_
