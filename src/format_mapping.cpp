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

#include "usb_cam/format_mapping.hpp"

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace usb_cam
{
namespace format_mapping
{

namespace
{

const std::unordered_map<std::string, capture_format_t> & table()
{
  static const std::unordered_map<std::string, capture_format_t> kTable = {
    {"mjpeg2rgb",
      {"mjpeg2rgb", "mjpeg", decode_policy_t::DECODE_SWS, AV_PIX_FMT_RGB24, "rgb8", 3}},
    {"raw_mjpeg",
      {"raw_mjpeg", "mjpeg", decode_policy_t::PACKET_PASSTHROUGH, AV_PIX_FMT_NONE, "unknown", 0}},
    {"yuyv2rgb",
      {"yuyv2rgb", "yuyv422", decode_policy_t::RAW_SWS, AV_PIX_FMT_RGB24, "rgb8", 3}},
    {"yuyv",
      {"yuyv", "yuyv422", decode_policy_t::RAW_SWS, AV_PIX_FMT_YUYV422, "yuv422_yuy2", 2}},
    {"uyvy2rgb",
      {"uyvy2rgb", "uyvy422", decode_policy_t::RAW_SWS, AV_PIX_FMT_RGB24, "rgb8", 3}},
    {"uyvy",
      {"uyvy", "uyvy422", decode_policy_t::RAW_SWS, AV_PIX_FMT_UYVY422, "yuv422", 2}},
    {"mono8",
      {"mono8", "gray", decode_policy_t::RAW_SWS, AV_PIX_FMT_GRAY8, "mono8", 1}},
    {"mono16",
      {"mono16", "gray16le", decode_policy_t::RAW_SWS, AV_PIX_FMT_GRAY16LE, "mono16", 2}},
    {"rgb8",
      {"rgb8", "rgb24", decode_policy_t::RAW_SWS, AV_PIX_FMT_RGB24, "rgb8", 3}},
    {"m4202rgb",
      {"m4202rgb", "yuv420p", decode_policy_t::RAW_SWS, AV_PIX_FMT_RGB24, "rgb8", 3}},
    {"y102mono8",
      {"y102mono8", "gray16le", decode_policy_t::RAW_SWS, AV_PIX_FMT_GRAY8, "mono8", 1}},
  };
  return kTable;
}

const std::unordered_map<std::string, std::string> & av_device_format_table()
{
  static const std::unordered_map<std::string, std::string> kTable = {
    {"YUV422P", "yuv422"},
    {"YUV420P", "nv21"},
    {"YUV444P", "nv24"},
    {"RGB24", "rgb8"},
    {"BGR24", "bgr8"},
    {"RGBA", "rgba8"},
    {"BGRA", "bgra8"},
    {"GRAY8", "mono8"},
    {"GRAY16BE", "mono16"},
    {"GRAY16LE", "mono16"},
  };
  return kTable;
}

}  // namespace

std::vector<std::string> supported_names()
{
  std::vector<std::string> names;
  names.reserve(table().size());
  for (const auto & entry : table()) {
    names.push_back(entry.first);
  }
  return names;
}

capture_format_t resolve_capture_format(const std::string & pixel_format_name)
{
  const auto it = table().find(pixel_format_name);
  if (it == table().end()) {
    std::string msg = "Specified format `" + pixel_format_name + "` is unsupported. Supported:";
    for (const auto & name : supported_names()) {
      msg += " " + name;
    }
    throw std::invalid_argument(msg);
  }
  return it->second;
}

std::string ros_encoding_from_av_device_format(const std::string & av_device_format)
{
  const auto it = av_device_format_table().find(av_device_format);
  if (it == av_device_format_table().end()) {
    return "unknown";
  }
  return it->second;
}

}  // namespace format_mapping
}  // namespace usb_cam
