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

#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include "usb_cam/usb_cam_node.hpp"
#include "usb_cam/utils.hpp"

const char BASE_TOPIC_NAME[] = "image_raw";

namespace usb_cam
{

UsbCamNode::UsbCamNode(const rclcpp::NodeOptions& node_options)
  : Node("usb_cam", node_options)
  , m_camera(new usb_cam::UsbCam())
  , m_image_msg(new sensor_msgs::msg::Image())
  , m_compressed_img_msg(nullptr)
  , m_image_publisher(std::make_shared<image_transport::CameraPublisher>(
        image_transport::create_camera_publisher(this, BASE_TOPIC_NAME,
                                                 // rmw_qos_profile_default // reliable
                                                 rmw_qos_profile_sensor_data  // best_effort
                                                 )))
  , m_compressed_image_publisher(nullptr)
  , m_compressed_cam_info_publisher(nullptr)
  , m_parameters()
  , m_camera_info_msg(new sensor_msgs::msg::CameraInfo())
  , m_service_capture(this->create_service<std_srvs::srv::SetBool>(
        "set_capture", std::bind(&UsbCamNode::service_capture, this, std::placeholders::_1, std::placeholders::_2,
                                 std::placeholders::_3)))
{
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 10.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 360);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv2rgb");
  this->declare_parameter("av_device_format", "YUV444P");
  this->declare_parameter("video_device", "/dev/video1");
  this->declare_parameter("brightness", 50);  // 0-255, -1 "leave alone"
  this->declare_parameter("contrast", -1);    // 0-255, -1 "leave alone"
  this->declare_parameter("saturation", -1);  // 0-255, -1 "leave alone"
  this->declare_parameter("sharpness", -1);   // 0-255, -1 "leave alone"
  this->declare_parameter("gain", -1);        // 0-100?, -1 "leave alone"
  this->declare_parameter("auto_white_balance", true);
  this->declare_parameter("white_balance", 4000);
  this->declare_parameter("autoexposure", true);
  this->declare_parameter("exposure", 100);
  this->declare_parameter("autofocus", false);
  this->declare_parameter("focus", -1);  // 0-255, -1 "leave alone"
  this->declare_parameter("undistort_image", false);
  this->declare_parameter("undistort_image_gpu", false);
  this->declare_parameter("pub_raw", true);
  this->declare_parameter("pub_compressed", true);

  get_params();
  init();
  m_parameters_callback_handle =
      add_on_set_parameters_callback(std::bind(&UsbCamNode::parameters_callback, this, std::placeholders::_1));
}

UsbCamNode::~UsbCamNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  m_image_msg.reset();
  m_compressed_img_msg.reset();
  m_camera_info_msg.reset();
  m_camera_info.reset();
  m_timer.reset();
  m_service_capture.reset();
  m_parameters_callback_handle.reset();

  delete (m_camera);
}

void UsbCamNode::service_capture(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;
  if (request->data)
  {
    m_camera->start_capturing();
    response->message = "Start Capturing";
  }
  else
  {
    m_camera->stop_capturing();
    response->message = "Stop Capturing";
  }
}

std::string resolve_device_path(const std::string& path)
{
  if (std::filesystem::is_symlink(path))
  {
    std::filesystem::path target_path = std::filesystem::read_symlink(path);

    // if the target path is relative, resolve it
    if (target_path.is_relative())
    {
      target_path = std::filesystem::absolute(path).parent_path() / target_path;
      target_path = std::filesystem::canonical(target_path);
    }

    return target_path.string();
  }
  return path;
}

void UsbCamNode::init()
{
  while (m_parameters.frame_id == "")
  {
    RCLCPP_WARN_ONCE(this->get_logger(), "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // load the camera info
  m_camera_info.reset(
      new camera_info_manager::CameraInfoManager(this, m_parameters.camera_name, m_parameters.camera_info_url));
  // check for default camera info
  if (!m_camera_info->isCalibrated())
  {
    RCLCPP_INFO(get_logger(), "m_camera_info->isCalibrated(): false");
    m_camera_info->setCameraName(m_parameters.device_name);
    m_camera_info_msg->header.frame_id = m_parameters.frame_id;
    m_camera_info_msg->width = m_parameters.image_width;
    m_camera_info_msg->height = m_parameters.image_height;
    m_camera_info->setCameraInfo(*m_camera_info_msg);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "m_camera_info->isCalibrated(): true");
  }
  auto info_msg = m_camera_info->getCameraInfo();
  cameraMatrix = (cv::Mat_<double>(3, 3) << info_msg.k[0], 0, info_msg.k[2], 0, info_msg.k[1], info_msg.k[3], 0, 0, 1);
  distCoeffs = (cv::Mat_<double>(5, 1) << info_msg.d[0], info_msg.d[1], info_msg.d[2], info_msg.d[3], info_msg.d[4]);

  // Check if given device name is an available v4l2 device
  auto available_devices = usb_cam::utils::available_devices();
  if (available_devices.find(m_parameters.device_name) == available_devices.end())
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Device specified is not available or is not a vaild V4L2 device: `"
                                                << m_parameters.device_name << "`");
    RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
    for (const auto& device : available_devices)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
      RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
    }
    rclcpp::shutdown();
    return;
  }

  // if pixel format is equal to 'mjpeg', i.e. raw mjpeg stream, initialize compressed image message
  // and publisher
  if (m_parameters.pixel_format_name == "mjpeg2rgb")
  {
    m_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
    m_compressed_img_msg->header.frame_id = m_parameters.frame_id;
    m_compressed_image_publisher = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        std::string(BASE_TOPIC_NAME) + "/compressed", rclcpp::QoS(100));
    m_compressed_cam_info_publisher =
        this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::QoS(100));
  }

  m_image_msg->header.frame_id = m_parameters.frame_id;
  RCLCPP_INFO(this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", m_parameters.camera_name.c_str(),
              m_parameters.device_name.c_str(), m_parameters.image_width, m_parameters.image_height,
              m_parameters.io_method_name.c_str(), m_parameters.pixel_format_name.c_str(), m_parameters.framerate);
  // set the IO method
  io_method_t io_method = usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
  if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN)
  {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Unknown IO method '%s'", m_parameters.io_method_name.c_str());
    rclcpp::shutdown();
    return;
  }

  // configure the camera
  try
  {
    m_camera->configure(m_parameters, io_method);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure camera: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  set_v4l2_params();

  // Wait for camera to stabilize after configuration
  RCLCPP_INFO(this->get_logger(), "Waiting 3s for camera to stabilize...");
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // start the camera
  try
  {
    m_camera->start();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to start camera stream: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Check dmesg for USB/UVC errors");
    m_camera->shutdown();
    rclcpp::shutdown();
    return;
  }

  // TODO(lucasw) should this check a little faster than expected frame rate?
  // TODO(lucasw) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
  const int period_ms = 1000.0 / m_parameters.framerate;
  m_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
                                    std::bind(&UsbCamNode::update, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
}

void UsbCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters({ "camera_name",        "camera_info_url",     "frame_id",
                                                        "framerate",          "image_height",        "image_width",
                                                        "io_method",          "pixel_format",        "av_device_format",
                                                        "video_device",       "brightness",          "contrast",
                                                        "saturation",         "sharpness",           "gain",
                                                        "auto_white_balance", "white_balance",       "autoexposure",
                                                        "exposure",           "autofocus",           "focus",
                                                        "undistort_image",    "undistort_image_gpu", "pub_raw",
                                                        "pub_compressed" });

  assign_params(parameters);
}

void UsbCamNode::assign_params(const std::vector<rclcpp::Parameter>& parameters)
{
  for (auto& parameter : parameters)
  {
    if (parameter.get_name() == "camera_name")
    {
      RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
      m_parameters.camera_name = parameter.value_to_string();
    }
    else if (parameter.get_name() == "camera_info_url")
    {
      m_parameters.camera_info_url = parameter.value_to_string();
    }
    else if (parameter.get_name() == "frame_id")
    {
      m_parameters.frame_id = parameter.value_to_string();
    }
    else if (parameter.get_name() == "framerate")
    {
      RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
      m_parameters.framerate = parameter.as_double();
    }
    else if (parameter.get_name() == "image_height")
    {
      m_parameters.image_height = parameter.as_int();
    }
    else if (parameter.get_name() == "image_width")
    {
      m_parameters.image_width = parameter.as_int();
    }
    else if (parameter.get_name() == "io_method")
    {
      m_parameters.io_method_name = parameter.value_to_string();
    }
    else if (parameter.get_name() == "pixel_format")
    {
      m_parameters.pixel_format_name = parameter.value_to_string();
    }
    else if (parameter.get_name() == "av_device_format")
    {
      m_parameters.av_device_format = parameter.value_to_string();
    }
    else if (parameter.get_name() == "video_device")
    {
      m_parameters.device_name = resolve_device_path(parameter.value_to_string());
    }
    else if (parameter.get_name() == "brightness")
    {
      m_parameters.brightness = parameter.as_int();
    }
    else if (parameter.get_name() == "contrast")
    {
      m_parameters.contrast = parameter.as_int();
    }
    else if (parameter.get_name() == "saturation")
    {
      m_parameters.saturation = parameter.as_int();
    }
    else if (parameter.get_name() == "sharpness")
    {
      m_parameters.sharpness = parameter.as_int();
    }
    else if (parameter.get_name() == "gain")
    {
      m_parameters.gain = parameter.as_int();
    }
    else if (parameter.get_name() == "auto_white_balance")
    {
      m_parameters.auto_white_balance = parameter.as_bool();
    }
    else if (parameter.get_name() == "white_balance")
    {
      m_parameters.white_balance = parameter.as_int();
    }
    else if (parameter.get_name() == "autoexposure")
    {
      m_parameters.autoexposure = parameter.as_bool();
    }
    else if (parameter.get_name() == "exposure")
    {
      m_parameters.exposure = parameter.as_int();
    }
    else if (parameter.get_name() == "autofocus")
    {
      m_parameters.autofocus = parameter.as_bool();
    }
    else if (parameter.get_name() == "undistort_image")
    {
      m_parameters.undistort_image = parameter.as_bool();
    }
    else if (parameter.get_name() == "undistort_image_gpu")
    {
      m_parameters.undistort_image_gpu = parameter.as_bool();
    }
    else if (parameter.get_name() == "pub_raw")
    {
      m_parameters.pub_raw = parameter.as_bool();
    }
    else if (parameter.get_name() == "pub_compressed")
    {
      m_parameters.pub_compressed = parameter.as_bool();
    }
    else if (parameter.get_name() == "focus")
    {
      m_parameters.focus = parameter.as_int();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}

/// @brief Send current parameters to V4L2 device
/// TODO(flynneva): should this actuaully be part of UsbCam class?
void UsbCamNode::set_v4l2_params()
{
  // Helper lambda for small delay between V4L2 parameter changes
  auto v4l2_param_delay = [this]() { std::this_thread::sleep_for(std::chrono::milliseconds(50)); };

  // set camera parameters
  if (m_parameters.brightness >= 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting 'brightness' to %d", m_parameters.brightness);
    m_camera->set_v4l_parameter("brightness", m_parameters.brightness);
    v4l2_param_delay();
  }

  if (m_parameters.contrast >= 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting 'contrast' to %d", m_parameters.contrast);
    m_camera->set_v4l_parameter("contrast", m_parameters.contrast);
    v4l2_param_delay();
  }

  if (m_parameters.saturation >= 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting 'saturation' to %d", m_parameters.saturation);
    m_camera->set_v4l_parameter("saturation", m_parameters.saturation);
    v4l2_param_delay();
  }

  if (m_parameters.sharpness >= 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting 'sharpness' to %d", m_parameters.sharpness);
    m_camera->set_v4l_parameter("sharpness", m_parameters.sharpness);
    v4l2_param_delay();
  }

  if (m_parameters.gain >= 0)
  {
    RCLCPP_INFO(this->get_logger(), "Setting 'gain' to %d", m_parameters.gain);
    m_camera->set_v4l_parameter("gain", m_parameters.gain);
    v4l2_param_delay();
  }

  // check auto white balance
  if (m_parameters.auto_white_balance)
  {
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 1);
    v4l2_param_delay();
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance_temperature_auto' to %d", 1);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Setting 'white_balance' to %d", m_parameters.white_balance);
    m_camera->set_v4l_parameter("white_balance_temperature_auto", 0);
    v4l2_param_delay();
    m_camera->set_v4l_parameter("white_balance_temperature", m_parameters.white_balance);
    v4l2_param_delay();
  }

  // check auto exposure
  if (!m_parameters.autoexposure)
  {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 1);
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure' to %d", m_parameters.exposure);
    // turn down exposure control (from max of 3)
    m_camera->set_v4l_parameter("exposure_auto", 1);
    v4l2_param_delay();
    // change the exposure level
    m_camera->set_v4l_parameter("exposure_absolute", m_parameters.exposure);
    v4l2_param_delay();
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Setting 'exposure_auto' to %d", 3);
    m_camera->set_v4l_parameter("exposure_auto", 3);
    v4l2_param_delay();
  }

  // check auto focus
  if (m_parameters.autofocus)
  {
    m_camera->set_auto_focus(1);
    v4l2_param_delay();
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 1);
    m_camera->set_v4l_parameter("focus_auto", 1);
    v4l2_param_delay();
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Setting 'focus_auto' to %d", 0);
    m_camera->set_v4l_parameter("focus_auto", 0);
    v4l2_param_delay();
    if (m_parameters.focus >= 0)
    {
      RCLCPP_INFO(this->get_logger(), "Setting 'focus_absolute' to %d", m_parameters.focus);
      m_camera->set_v4l_parameter("focus_absolute", m_parameters.focus);
      v4l2_param_delay();
    }
  }
}

void UsbCamNode::undistortImage(std::unique_ptr<sensor_msgs::msg::Image>& src,
                                std::shared_ptr<camera_info_manager::CameraInfo> camera_info)
{
  // 1. 转换为OpenCV格式 (假设原始图像为BGR8编码)
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(*src, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("undistort"), "转换失败: %s", e.what());
    return;
  }
  cv::Mat mat = cv_ptr->image;

  // 2. 设置相机参数 (示例值，需替换为实际参数)
  cv::Mat cameraMatrix =
      (cv::Mat_<double>(3, 3) << camera_info->k[0], camera_info->k[1], camera_info->k[2], camera_info->k[3],
       camera_info->k[4], camera_info->k[5], camera_info->k[6], camera_info->k[7], camera_info->k[8]);
  cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << camera_info->d[0], camera_info->d[1], camera_info->d[2],
                        camera_info->d[3], camera_info->d[4]
                        // -0.365125, 0.110699, -0.001204, 0.002611, 0.000000
  );

  // 3. 创建映射矩阵 map1, map2
  // cv::Mat Knew = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, mat.size(), 0.0, mat.size());
  cv::Mat Knew = cameraMatrix;  // 保持一致，保证识别正确的位姿
  if (!map_generated)
  {
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), Knew, mat.size(), CV_32FC1, map1, map2);
    map_generated = true;
  }

  // 3. 执行畸变校正
  undistortImage2(mat);

  // 4. 转回ROS消息格式
  cv_bridge::CvImage out_msg;
  out_msg.header = src->header;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = mat;
  *src = *out_msg.toImageMsg();
}

void UsbCamNode::undistortImage2(cv::Mat& src)
{
  if (!m_parameters.undistort_image_gpu)
  {
    // using cpu
    RCLCPP_INFO_ONCE(get_logger(), "使用cpu矫正图像畸变");

    cv::Mat dst;

    // 方法一
    // cv::undistort(src, dst, K, D);
    // src = dst;

    // 方法二
    cv::remap(src, dst, map1, map2, cv::INTER_LINEAR);
    src = dst;
  }
  else
  {
    // using GPU
    RCLCPP_INFO_ONCE(get_logger(), "使用gpu校正图像畸变");
    src = gpuUndistort(src, map1, map2);
  }
}

cv::Mat UsbCamNode::gpuUndistort(cv::Mat& img, cv::Mat map1, cv::Mat map2)
{
  try
  {
    if (cv::cuda::getCudaEnabledDeviceCount() == 0)
    {
      RCLCPP_WARN(get_logger(), "No CUDA device available, falling back to CPU");
      cv::Mat dst;
      cv::remap(img, dst, map1, map2, cv::INTER_LINEAR);
      return dst;
    }

    if (!gpu_map_uploaded)
    {
      gpuMap1.upload(map1);
      gpuMap2.upload(map2);
      gpu_map_uploaded = true;
    }

    cv::cuda::GpuMat gpuImg, gpuDst;
    gpuImg.upload(img);

    cv::cuda::remap(gpuImg, gpuDst, gpuMap1, gpuMap2, cv::INTER_LINEAR, cv::BORDER_REFLECT);

    cv::Mat dst;
    gpuDst.download(dst);
    return dst;
  }
  catch (const cv::Exception& e)
  {
    RCLCPP_ERROR(get_logger(), "GPU undistort failed: %s", e.what());
    cv::Mat dst;
    cv::remap(img, dst, map1, map2, cv::INTER_LINEAR);
    return dst;
  }
}

bool UsbCamNode::take_and_send_image()
{
  // Only resize if required
  if (sizeof(m_image_msg->data) != m_camera->get_image_size_in_bytes())
  {
    m_image_msg->width = m_camera->get_image_width();
    m_image_msg->height = m_camera->get_image_height();
    m_image_msg->encoding = m_camera->get_pixel_format()->ros();
    m_image_msg->step = m_camera->get_image_step();
    if (m_image_msg->step == 0)
    {
      // Some formats don't have a linesize specified by v4l2
      // Fall back to manually calculating it step = size / height
      m_image_msg->step = m_camera->get_image_size_in_bytes() / m_image_msg->height;
    }
    m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  // grab the image, pass image msg buffer to fill
  m_camera->get_image(reinterpret_cast<char*>(&m_image_msg->data[0]));
  *m_camera_info_msg = m_camera_info->getCameraInfo();
  RCLCPP_INFO_ONCE(get_logger(), "undistort_image: %s", m_parameters.undistort_image ? "true" : "false");
  RCLCPP_INFO_ONCE(get_logger(), "undistort_image_gpu: %s", m_parameters.undistort_image_gpu ? "true" : "false");
  RCLCPP_INFO_ONCE(get_logger(), "pub_raw: %s", m_parameters.pub_raw ? "true" : "false");
  RCLCPP_INFO_ONCE(get_logger(), "pub_compressed: %s", m_parameters.pub_compressed ? "true" : "false");
  if (m_parameters.undistort_image)
  {
    RCLCPP_INFO_ONCE(get_logger(), "校正图像畸变");
    undistortImage(m_image_msg, m_camera_info_msg);
  }
  else
  {
    RCLCPP_INFO_ONCE(get_logger(), "未校正图像畸变");
  }

  auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;

  m_camera_info_msg->header = m_image_msg->header;

  if (m_parameters.pub_raw)
  {
    m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);
  }

  if (m_parameters.pub_compressed)
  {
    RCLCPP_INFO_ONCE(get_logger(), "publish compressed image topic.");
    take_and_send_image_mjpeg();
  }

  return true;
}

bool UsbCamNode::take_and_send_image_mjpeg()
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // 1. 转换为OpenCV格式 (假设原始图像为BGR8编码)
    cv_ptr = cv_bridge::toCvCopy(*m_image_msg, sensor_msgs::image_encodings::BGR8);
    m_compressed_img_msg->header = m_image_msg->header;
    m_compressed_img_msg->format = "jpeg";

    // 2. 使用OpenCV进行图像压缩
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);  // 压缩质量
    cv::imencode(".jpg", cv_ptr->image, m_compressed_img_msg->data, compression_params);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("undistort"), "转换失败: %s", e.what());
    return false;
  }
  m_compressed_image_publisher->publish(*m_compressed_img_msg);

  // 只有 pub_raw 为 false 时，才由压缩路径发布 camera_info
  if (!m_parameters.pub_raw)
  {
    m_compressed_cam_info_publisher->publish(*m_camera_info_msg);
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult
UsbCamNode::parameters_callback(const std::vector<rclcpp::Parameter>& parameters)
{
  RCLCPP_DEBUG(this->get_logger(), "Setting parameters for %s", m_parameters.camera_name.c_str());
  m_timer->reset();
  assign_params(parameters);
  set_v4l2_params();
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void UsbCamNode::update()
{
  if (m_camera->is_capturing())
  {
    // If the camera exposure longer higher than the framerate period
    // then that caps the framerate.
    // auto t0 = now();
    bool isSuccessful = take_and_send_image();
    if (!isSuccessful)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "USB camera did not respond in time.");
    }
  }
}

}  // namespace usb_cam

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamNode)
