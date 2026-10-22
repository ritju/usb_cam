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


#ifndef USB_CAM__USB_CAM_HPP_
#define USB_CAM__USB_CAM_HPP_

#include <ctime>
#include <memory>
#include <string>
#include <vector>

#include "usb_cam/ffmpeg_capture.hpp"
#include "usb_cam/format_mapping.hpp"
#include "usb_cam/logging.hpp"

namespace usb_cam
{

typedef struct parameters_t
{
  std::string camera_name;
  std::string device_name;
  std::string frame_id;
  std::string io_method_name;
  std::string camera_info_url;
  std::string pixel_format_name;
  std::string av_device_format;
  int image_width;
  int image_height;
  double framerate;
  int brightness;
  int contrast;
  int saturation;
  int sharpness;
  int gain;
  int white_balance;
  int exposure;
  int focus;
  bool auto_white_balance;
  bool autoexposure;
  bool autofocus;
  bool undistort_image;
  bool undistort_image_gpu;
  bool pub_raw;
  bool pub_compressed;

  parameters_t()
// *INDENT-OFF*
    : camera_name("usb_cam"),
    device_name("/dev/video0"),
    frame_id("camera"),
    io_method_name("mmap"),
    camera_info_url("package://usb_cam/config/camera_info.yaml"),
    pixel_format_name("yuyv2rgb"),
    av_device_format("YUV422P"),
    image_width(640),
    image_height(480),
    framerate(30.0),
    brightness(-1),
    contrast(-1),
    saturation(-1),
    sharpness(-1),
    gain(-1),
    white_balance(-1),
    exposure(-1),
    focus(-1),
    auto_white_balance(true),
    autoexposure(true),
    autofocus(false),
    undistort_image(false),
    undistort_image_gpu(false),
    pub_raw(true),
    pub_compressed(true)
  {
  }
// *INDENT-ON*
} parameters_t;

/// @brief Slim facade over FfmpegCapture. No V4L2 / ioctl surface.
class UsbCam
{
public:
  UsbCam();
  ~UsbCam();

  /// @brief Open and configure the device. Throws on failure.
  void configure(const parameters_t & parameters);

  void start();
  void shutdown();

  void start_capturing();
  void stop_capturing();

  /// @brief Grab one frame into the caller buffer (tightly packed).
  /// Returns false on a dropped frame (soft failure); dest is untouched.
  bool get_image(char * destination);

  /// @brief Grab into an internal buffer and return it.
  /// Returns nullptr on a dropped frame (soft failure).
  char * get_image();

  /// @brief Map a control name to an FFmpeg option when one exists.
  /// UVC controls are unsupported on the FFmpeg v4l2 path: warn-once and
  /// return false. Never throws.
  bool set_camera_control(const std::string & name, int value);

  /// @brief Route UsbCam/FfmpegCapture messages to a sink (rclcpp-free).
  inline void set_logger(log_fn_t fn)
  {
    m_logger = std::move(fn);
    m_capture.set_logger(m_logger);
  }

  inline size_t get_image_width() const {return static_cast<size_t>(m_capture.width());}
  inline size_t get_image_height() const {return static_cast<size_t>(m_capture.height());}
  inline size_t get_image_size_in_bytes() const {return m_capture.frame_size();}
  inline unsigned int get_image_step() const {return static_cast<unsigned int>(m_capture.step());}
  inline std::string get_encoding() const {return m_capture.encoding();}
  inline timespec get_image_timestamp() const {return m_last_stamp;}
  inline bool is_capturing() const {return m_capture.is_capturing();}
  inline std::string get_device_name() const {return m_capture.device_path();}
  inline size_t get_last_payload_size() const {return m_capture.last_payload_size();}
  inline bool is_passthrough() const {return m_capture.is_passthrough();}

private:
  void log(log_level_t level, const std::string & msg) const;

  FfmpegCapture m_capture;
  std::vector<char> m_internal_buffer;
  timespec m_last_stamp{};
  log_fn_t m_logger;
};

}  // namespace usb_cam

#endif  // USB_CAM__USB_CAM_HPP_
