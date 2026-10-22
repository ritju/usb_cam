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

#include "usb_cam/usb_cam.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace usb_cam
{

namespace
{

/// @brief Control names historically applied as V4L2 device controls.
/// FFmpeg's video4linux2 demuxer exposes no matching AVOptions, so these are
/// accepted for API compatibility and ignored with a one-time warning.
const std::unordered_set<std::string> & known_control_names()
{
  static const std::unordered_set<std::string> kNames = {
    "brightness",
    "contrast",
    "saturation",
    "sharpness",
    "gain",
    "white_balance_temperature_auto",
    "white_balance_temperature",
    "exposure_auto",
    "exposure_absolute",
    "focus_auto",
    "focus_absolute",
  };
  return kNames;
}

}  // namespace

UsbCam::UsbCam()
{
  m_last_stamp.tv_sec = 0;
  m_last_stamp.tv_nsec = 0;
}

UsbCam::~UsbCam()
{
  shutdown();
}

void UsbCam::log(log_level_t level, const std::string & msg) const
{
  if (m_logger) {
    m_logger(level, msg);
    return;
  }
  switch (level) {
    case log_level_t::DEBUG:
    case log_level_t::DEBUG_THROTTLE:
    case log_level_t::INFO:
    case log_level_t::INFO_THROTTLE:
      std::cout << "usb_cam: " << msg << std::endl;
      break;
    case log_level_t::WARN:
    case log_level_t::WARN_THROTTLE:
      std::cerr << "usb_cam: WARN: " << msg << std::endl;
      break;
    case log_level_t::ERROR:
      std::cerr << "usb_cam: ERROR: " << msg << std::endl;
      break;
  }
}

void UsbCam::configure(const parameters_t & parameters)
{
  capture_config_t config;
  config.device_path = parameters.device_name;
  config.width = parameters.image_width;
  config.height = parameters.image_height;
  config.framerate = parameters.framerate;

  // pixel_format name selects capture fourcc/codec; av_device_format is
  // advisory except for raw_mjpeg's ROS encoding label.
  try {
    config.format = format_mapping::resolve_capture_format(parameters.pixel_format_name);
  } catch (const std::invalid_argument & e) {
    log(log_level_t::ERROR, e.what());
    log(log_level_t::ERROR, "This driver supports the following formats:");
    for (const auto & name : format_mapping::supported_names()) {
      log(log_level_t::ERROR, "\t" + name);
    }
    throw;
  }
  if (config.format.policy == format_mapping::decode_policy_t::PACKET_PASSTHROUGH) {
    config.format.ros_encoding =
      format_mapping::ros_encoding_from_av_device_format(parameters.av_device_format);
    if (config.format.ros_encoding == "unknown") {
      log(
        log_level_t::WARN,
        "unrecognized av_device_format `" + parameters.av_device_format +
        "` for raw_mjpeg");
    }
  }

  m_capture.set_logger(m_logger);
  m_capture.open(config);
  m_internal_buffer.resize(m_capture.frame_size());
}

void UsbCam::start()
{
  start_capturing();
}

void UsbCam::shutdown()
{
  m_capture.close();
}

void UsbCam::start_capturing()
{
  m_capture.start_capturing();
}

void UsbCam::stop_capturing()
{
  m_capture.stop_capturing();
}

bool UsbCam::get_image(char * destination)
{
  if (!destination) {
    throw std::invalid_argument("get_image: null destination");
  }
  timespec stamp{};
  if (!m_capture.grab(destination, m_capture.frame_size(), stamp)) {
    return false;
  }
  m_last_stamp = stamp;
  return true;
}

char * UsbCam::get_image()
{
  if (m_internal_buffer.empty()) {
    m_internal_buffer.resize(m_capture.frame_size());
  }
  timespec stamp{};
  if (!m_capture.grab(m_internal_buffer.data(), m_internal_buffer.size(), stamp)) {
    return nullptr;
  }
  m_last_stamp = stamp;
  return m_internal_buffer.data();
}

bool UsbCam::set_camera_control(const std::string & name, int value)
{
  (void)value;
  static std::unordered_set<std::string> warned;
  if (known_control_names().count(name) != 0U) {
    if (warned.insert(name).second) {
      log(
        log_level_t::WARN,
        "camera control `" + name + "` is not supported by the FFmpeg capture path; ignored");
    }
    return false;
  }
  if (warned.insert(name).second) {
    log(log_level_t::WARN, "unknown camera control `" + name + "`; ignored");
  }
  return false;
}

}  // namespace usb_cam
