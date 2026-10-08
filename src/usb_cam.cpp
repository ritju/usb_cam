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

void UsbCam::configure(const parameters_t & parameters)
{
  capture_config_t config;
  config.device_path = parameters.device_name;
  config.width = parameters.image_width;
  config.height = parameters.image_height;
  config.framerate = parameters.framerate;

  // pixel_format name selects capture fourcc/codec; av_device_format is
  // advisory except for raw_mjpeg's ROS encoding label.
  config.format = format_mapping::resolve_capture_format(parameters.pixel_format_name);
  if (config.format.policy == format_mapping::decode_policy_t::PACKET_PASSTHROUGH) {
    config.format.ros_encoding =
      format_mapping::ros_encoding_from_av_device_format(parameters.av_device_format);
    if (config.format.ros_encoding == "unknown") {
      std::cerr << "usb_cam: unrecognized av_device_format `"
                << parameters.av_device_format << "` for raw_mjpeg" << std::endl;
    }
  }

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

void UsbCam::get_image(char * destination)
{
  if (!destination) {
    throw std::invalid_argument("get_image: null destination");
  }
  m_last_stamp = m_capture.grab(destination, m_capture.frame_size());
}

char * UsbCam::get_image()
{
  if (m_internal_buffer.empty()) {
    m_internal_buffer.resize(m_capture.frame_size());
  }
  m_last_stamp = m_capture.grab(m_internal_buffer.data(), m_internal_buffer.size());
  return m_internal_buffer.data();
}

bool UsbCam::set_camera_control(const std::string & name, int value)
{
  (void)value;
  static std::unordered_set<std::string> warned;
  if (known_control_names().count(name) != 0U) {
    if (warned.insert(name).second) {
      std::cerr << "usb_cam: camera control `" << name
                << "` is not supported by the FFmpeg capture path; ignored" << std::endl;
    }
    return false;
  }
  if (warned.insert(name).second) {
    std::cerr << "usb_cam: unknown camera control `" << name << "`; ignored" << std::endl;
  }
  return false;
}

}  // namespace usb_cam
