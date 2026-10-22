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

#ifndef USB_CAM__FORMAT_MAPPING_HPP_
#define USB_CAM__FORMAT_MAPPING_HPP_

extern "C" {
#include <libavutil/pixfmt.h>
}

#include <string>
#include <vector>

namespace usb_cam
{
namespace format_mapping
{

/// @brief How frames leave the capture pipeline
enum class decode_policy_t
{
  /// @brief decode (e.g. MJPEG) then swscale to out_pix_fmt
  DECODE_SWS,
  /// @brief raw video, copy/swscale to out_pix_fmt
  RAW_SWS,
  /// @brief copy compressed packet bytes as-is (raw_mjpeg)
  PACKET_PASSTHROUGH,
  /// @brief gray16le carrying 10-bit samples, pack >>2 to GRAY8
  Y10_PACK,
};

struct capture_format_t
{
  /// @brief legacy pixel_format parameter name
  std::string name;
  /// @brief FFmpeg v4l2 demuxer input_format / pixel_format option
  std::string input_format;
  decode_policy_t policy;
  /// @brief output pixel format for the caller buffer
  AVPixelFormat out_pix_fmt;
  /// @brief ROS sensor_msgs encoding string
  std::string ros_encoding;
  /// @brief packed bytes per pixel of the output buffer
  int bytes_per_pixel;
};

/// @brief Names accepted by the pixel_format parameter
std::vector<std::string> supported_names();

/// @brief Resolve a legacy pixel_format name to capture settings.
/// @throws std::invalid_argument if name is unknown
capture_format_t resolve_capture_format(const std::string & pixel_format_name);

/// @brief ROS encoding label for av_device_format (raw_mjpeg path).
/// Returns "unknown" when the string is unrecognized.
std::string ros_encoding_from_av_device_format(const std::string & av_device_format);

}  // namespace format_mapping
}  // namespace usb_cam

#endif  // USB_CAM__FORMAT_MAPPING_HPP_
