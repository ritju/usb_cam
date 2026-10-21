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

#define CLEAR(x) memset(&(x), 0, sizeof(x))

extern "C" {
#include <linux/videodev2.h>  // Defines V4L2 format constants
#include <malloc.h>  // for memalign
#include <sys/ioctl.h>  // for ioctl
#include <sys/mman.h>  // for mmap
#include <sys/stat.h>  // for stat
#include <unistd.h>  // for getpagesize()
#include <fcntl.h>  // for O_* constants and open()
}

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "opencv2/imgproc.hpp"

#include "usb_cam/usb_cam.hpp"
#include "usb_cam/conversions.hpp"
#include "usb_cam/utils.hpp"


namespace usb_cam
{

using utils::io_method_t;

/// A driver may legitimately report more than the raw frame size (compressed
/// formats carry a worst-case bound, and rows get padded), but anything beyond
/// this multiple of the expected frame size means the format descriptor is
/// corrupt rather than that a real buffer is needed.
constexpr size_t kMaxSizeimageFactor = 4;


UsbCam::UsbCam()
: m_device_name(), m_io(io_method_t::IO_METHOD_MMAP), m_fd(-1),
  m_number_of_buffers(4), m_max_buffer_bytes(0),
  // Value-initialize so that a partially completed init_mmap()/init_userp()
  // leaves nullptr entries behind; uninit_device() must never munmap/free an
  // indeterminate pointer when unwinding an error path.
  m_buffers(new usb_cam::utils::buffer[m_number_of_buffers]()), m_image(),
  m_avframe(NULL), m_avcodec(NULL), m_avoptions(NULL),
  m_avcodec_context(NULL), m_is_capturing(false), m_framerate(0),
  m_epoch_time_shift_us(usb_cam::utils::get_epoch_time_shift_us()), m_supported_formats()
{}

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
  // Default sink keeps the library usable without a host logger.
  switch (level) {
    case log_level_t::DEBUG:
    case log_level_t::INFO:
      std::cout << msg << std::endl;
      break;
    case log_level_t::WARN:
    case log_level_t::ERROR:
      std::cerr << msg << std::endl;
      break;
  }
}


/// @brief Fill destination image with source image. If required, convert a given
/// V4L2 Image into another type. Look up possible V4L2 pixe formats in the
/// `linux/videodev2.h` header file.
/// @param src a pointer to a V4L2 source image
/// @param dest a pointer to where the source image should be copied (if required)
/// @param bytes_used number of bytes used by the src buffer
void UsbCam::process_image(const char * src, char * & dest, const int & bytes_used)
{
  // TODO(flynneva): could we skip the copy here somehow?
  // If no conversion required, just copy the image from V4L2 buffer
  if (m_image.pixel_format->requires_conversion() == false) {
    // Never copy past either the dequeued payload or the destination capacity.
    const size_t src_bytes = bytes_used > 0 ? static_cast<size_t>(bytes_used) : 0;
    const size_t n = std::min(src_bytes, m_image.size_in_bytes);
    memcpy(dest, src, n);
  } else {
    m_image.pixel_format->convert(src, dest, bytes_used);
  }
}

void UsbCam::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      len = read(m_fd, m_buffers[0].start, m_buffers[0].length);
      if (len == -1) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Unable to read frame");
        }
      }
      {
        const std::string err = frame_payload_error(
          /* index */ 0, /* n_buffers */ 1,
          static_cast<uint32_t>(len), m_buffers[0].length,
          /* flags */ 0, m_max_buffer_bytes);
        if (!err.empty()) {
          log(log_level_t::WARN, "usb_cam: dropping read frame: " + err);
          return;
        }
      }
      return process_image(m_buffers[0].start, m_image.data, len);
    case io_method_t::IO_METHOD_MMAP:
      CLEAR(buf);
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      /// Dequeue buffer with the new image
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Unable to retrieve frame with mmap");
        }
      }

      {
        const std::string err = frame_payload_error(
          buf.index, m_number_of_buffers,
          buf.bytesused, buf.length, buf.flags, m_max_buffer_bytes);
        if (!err.empty()) {
          log(log_level_t::WARN, "usb_cam: dropping mmap frame: " + err);
          // Requeue so the stream keeps draining; one bad buffer must not stall.
          if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
            log(log_level_t::ERROR, "usb_cam: QBUF failed while dropping a bad frame");
          }
          return;
        }
      }

      // Get timestamp from V4L2 image buffer
      m_image.stamp = usb_cam::utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);

      process_image(m_buffers[buf.index].start, m_image.data, buf.bytesused);

      /// Requeue buffer so it can be reused
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
        throw std::runtime_error("Unable to exchange buffer with the driver");
      }
      return;
    case io_method_t::IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_DQBUF), &buf)) {
        switch (errno) {
          case EAGAIN:
            return;
          default:
            throw std::runtime_error("Unable to exchange buffer with driver");
        }
      }

      {
        const std::string err = frame_payload_error(
          buf.index, m_number_of_buffers,
          buf.bytesused, buf.length, buf.flags, m_max_buffer_bytes);
        if (!err.empty()) {
          log(log_level_t::WARN, "usb_cam: dropping userptr frame: " + err);
          if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
            log(log_level_t::ERROR, "usb_cam: QBUF failed while dropping a bad frame");
          }
          return;
        }
      }

      // Get timestamp from V4L2 image buffer
      m_image.stamp = usb_cam::utils::calc_img_timestamp(buf.timestamp, m_epoch_time_shift_us);

      for (i = 0; i < m_number_of_buffers; ++i) {
        if (buf.m.userptr == reinterpret_cast<uint64_t>(m_buffers[i].start) && \
          buf.length == m_buffers[i].length)
        {
          return;
        }
      }

      assert(i < m_number_of_buffers);
      process_image(reinterpret_cast<const char *>(buf.m.userptr), m_image.data, buf.bytesused);
      if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
        throw std::runtime_error("Unable to exchange buffer with driver");
      }
      return;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
}

void UsbCam::stop_capturing()
{
  if (!m_is_capturing) {return;}

  m_is_capturing = false;
  enum v4l2_buf_type type;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      return;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMOFF, &type)) {
        // Keep capturing=false: the fd is about to be closed or already dead.
        // Never throw from a cleanup path — shutdown()/~UsbCam call this.
        log(
          log_level_t::ERROR,
          std::string("usb_cam: VIDIOC_STREAMOFF failed: ") + strerror(errno));
      }
      return;
    case io_method_t::IO_METHOD_UNKNOWN:
      return;
  }
}

void UsbCam::start_capturing()
{
  if (m_is_capturing) {return;}

  unsigned int i;
  enum v4l2_buf_type type;

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      /* Nothing to do. */
      break;
    case io_method_t::IO_METHOD_MMAP:
      // Queue the buffers
      for (i = 0; i < m_number_of_buffers; ++i) {
        struct v4l2_buffer buf;
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
          throw std::runtime_error("Unable to queue image buffer");
        }
      }

      // Start the stream
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMON, &type)) {
        throw std::runtime_error("Unable to start stream");
      }
      break;
    case io_method_t::IO_METHOD_USERPTR:
      for (i = 0; i < m_number_of_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = reinterpret_cast<uint64_t>(m_buffers[i].start);
        buf.length = m_buffers[i].length;

        if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QBUF), &buf)) {
          throw std::runtime_error("Unable to configure stream");
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_STREAMON, &type)) {
        throw std::runtime_error("Unable to start stream");
      }
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("IO method unknown");
  }
  m_is_capturing = true;
}

void UsbCam::uninit_device()
{
  if (!m_buffers) {
    return;
  }

  bool released_any = false;

  switch (m_io) {
    case io_method_t::IO_METHOD_MMAP:
      for (uint32_t i = 0; i < m_number_of_buffers; ++i) {
        if (m_buffers[i].start == nullptr) {continue;}
        munmap(m_buffers[i].start, m_buffers[i].length);
        m_buffers[i].start = nullptr;
        released_any = true;
      }
      break;

    case io_method_t::IO_METHOD_USERPTR:
      for (uint32_t i = 0; i < m_number_of_buffers; ++i) {
        if (m_buffers[i].start == nullptr) {continue;}
        free(m_buffers[i].start);
        m_buffers[i].start = nullptr;
        released_any = true;
      }
      break;

    case io_method_t::IO_METHOD_READ:
      if (m_buffers[0].start != nullptr) {
        free(m_buffers[0].start);
        m_buffers[0].start = nullptr;
      }
      break;

    case io_method_t::IO_METHOD_UNKNOWN:
      break;
  }

  // Drop the vb2 queue instead of leaving it to close(m_fd). A node that is
  // killed or hangs before close_device() runs would otherwise leave the
  // buffers resident in kernel memory, which is charged to VmallocUsed rather
  // than to any process and so is invisible in top.
  if (released_any) {
    free_buffers();
  }

  m_buffers.reset();
}

void UsbCam::free_buffers()
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = 0;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = (m_io == io_method_t::IO_METHOD_MMAP) ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR;

  // Not fatal: close_device() releases the queue as a backstop.
  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_REQBUFS), &req)) {
    log(
      log_level_t::ERROR,
      std::string("usb_cam: VIDIOC_REQBUFS(0) failed: ") + strerror(errno));
  }
}

void UsbCam::init_read()
{
  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  m_buffers[0].length = m_image.size_in_bytes;
  m_buffers[0].start = static_cast<char *>(calloc(1, m_image.size_in_bytes));

  if (!m_buffers[0].start) {
    throw std::overflow_error("Out of memory");
  }
}

void UsbCam::init_mmap()
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = m_number_of_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_REQBUFS), &req)) {
    if (EINVAL == errno) {
      throw std::runtime_error("Device does not support memory mapping");
    } else {
      throw std::runtime_error("Unable to initialize memory mapping");
    }
  }

  if (req.count < m_number_of_buffers) {
    throw std::overflow_error("Insufficient buffer memory on device");
  }

  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  for (uint32_t current_buffer = 0; current_buffer < req.count; ++current_buffer) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = current_buffer;

    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYBUF), &buf)) {
      throw std::runtime_error("Unable to query status of buffer");
    }

    // validate_sizeimage() already ran before REQBUFS, so this only catches a
    // driver reporting a per-buffer length inconsistent with the format it just
    // accepted. Never mmap an unbounded length from the driver.
    if (buf.length == 0 || buf.length > m_max_buffer_bytes) {
      throw std::runtime_error(
              "Driver reported an implausible buffer length of " + std::to_string(buf.length) +
              " bytes (limit " + std::to_string(m_max_buffer_bytes) + "), refusing to mmap");
    }

    m_buffers[current_buffer].length = buf.length;
    m_buffers[current_buffer].start =
      reinterpret_cast<char *>(mmap(
        NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */,
        MAP_SHARED /* recommended */, m_fd, buf.m.offset));

    if (MAP_FAILED == m_buffers[current_buffer].start) {
      throw std::runtime_error("Unable to allocate memory for image buffers");
    }
  }
}

void UsbCam::init_userp()
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  auto buffer_size = (m_image.size_in_bytes + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = m_number_of_buffers;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      throw std::invalid_argument("Device does not support user pointer i/o");
    } else {
      throw std::invalid_argument("Unable to initialize memory mapping");
    }
  }

  if (!m_buffers) {
    throw std::overflow_error("Out of memory");
  }

  for (uint32_t current_buffer = 0; current_buffer < req.count; ++current_buffer) {
    m_buffers[current_buffer].length = buffer_size;
    m_buffers[current_buffer].start =
      reinterpret_cast<char *>(memalign(/* boundary */ page_size, buffer_size));

    if (!m_buffers[current_buffer].start) {
      throw std::overflow_error("Out of memory");
    }
  }
}

/// @brief Reject an implausible driver-reported sizeimage before any buffer is
/// allocated, and record the accepted upper bound in m_max_buffer_bytes.
/// @details VIDIOC_REQBUFS makes the kernel allocate m_number_of_buffers copies
/// of sizeimage. That memory is charged to VmallocUsed, not to this process, so
/// an oversized value is invisible in top and can OOM the whole machine. A
/// corrupt UVC frame descriptor -- observed after device re-enumeration and when
/// a second fd issues S_FMT on an already-streaming device -- can report
/// gigabytes here.
void UsbCam::validate_sizeimage()
{
  const struct v4l2_pix_format & pix = m_image.v4l2_fmt.fmt.pix;

  // Derive the expectation from the dimensions the driver actually accepted
  // (VIDIOC_S_FMT is free to round width/height) and from the *device* fourcc,
  // not the ROS output bpp: YUYV is 2 B/px even when converting to RGB8.
  const size_t expected = usb_cam::utils::device_sizeimage_estimate(
    pix.pixelformat, pix.width, pix.height);
  const size_t limit = expected * kMaxSizeimageFactor;

  if (pix.sizeimage > limit) {
    throw std::runtime_error(
            "Driver reported an implausible sizeimage of " + std::to_string(pix.sizeimage) +
            " bytes for " + std::to_string(pix.width) + "x" + std::to_string(pix.height) + " " +
            m_image.pixel_format->v4l2_str() + " (expected at most " + std::to_string(limit) +
            " bytes); refusing to allocate " + std::to_string(m_number_of_buffers) +
            " such buffers. Check `v4l2-ctl --get-fmt-video` and re-plug the device.");
  }

  if (pix.sizeimage == 0) {
    // Some drivers leave this unset and still work, so keep the old behaviour
    // and only bound what init_mmap() will accept.
    log(
      log_level_t::WARN,
      "usb_cam: driver reported sizeimage 0 for " + std::to_string(pix.width) + "x" +
      std::to_string(pix.height) + " " + m_image.pixel_format->v4l2_str());
    m_max_buffer_bytes = limit;
    return;
  }

  m_max_buffer_bytes = pix.sizeimage;
}

void UsbCam::init_device()
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCAP), &cap)) {
    if (EINVAL == errno) {
      throw std::invalid_argument("Device is not a V4L2 device");
    } else {
      throw std::invalid_argument("Unable to query device capabilities");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    throw std::invalid_argument("Device is not a video capture device");
  }

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        throw std::invalid_argument("Device does not support read i/o");
      }
      break;
    case io_method_t::IO_METHOD_MMAP:
    case io_method_t::IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        throw std::invalid_argument("Device does not support streaming i/o");
      }
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      throw std::invalid_argument("Unsupported IO method specified");
  }

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_CROPCAP), &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == usb_cam::utils::xioctl(m_fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
  }

  CLEAR(m_image.v4l2_fmt);
  m_image.v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  m_image.v4l2_fmt.fmt.pix.width = m_image.width;
  m_image.v4l2_fmt.fmt.pix.height = m_image.height;
  m_image.v4l2_fmt.fmt.pix.pixelformat = m_image.pixel_format->v4l2();
  m_image.v4l2_fmt.fmt.pix.field = V4L2_FIELD_ANY;

  // Set v4l2 capture format
  // Note VIDIOC_S_FMT may change width and height
  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_FMT), &m_image.v4l2_fmt)) {
    throw std::runtime_error(std::string("VIDIOC_S_FMT failed: ") + strerror(errno));
  }

  // Deliberately do NOT resync m_image.width/height/size_in_bytes from the
  // accepted format. Converter objects (YUYV2RGB, M4202RGB, ...) freeze their
  // pixel count at construction from the *requested* geometry; shrinking the
  // ROS destination buffer to the accepted size would let them write past it.
  // validate_sizeimage() uses pix.width/height directly. The memcpy path is
  // bounded by min(bytes_used, size_in_bytes) instead.

  // Must run before VIDIOC_REQBUFS below: that ioctl is what actually commits
  // the kernel allocation of m_number_of_buffers * sizeimage bytes.
  validate_sizeimage();

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_G_PARM), &stream_params) < 0) {
    throw std::runtime_error(std::string("VIDIOC_G_PARM failed: ") + strerror(errno));
  }

  if (!(stream_params.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
    throw std::runtime_error("V4L2_CAP_TIMEPERFRAME not supported");
  }

  // TODO(lucasw) need to get list of valid numerator/denominator pairs
  // and match closest to what user put in.
  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = m_framerate;
  if (usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_PARM), &stream_params) < 0) {
    throw std::invalid_argument("Couldn't set camera framerate");
  }

  switch (m_io) {
    case io_method_t::IO_METHOD_READ:
      init_read();
      break;
    case io_method_t::IO_METHOD_MMAP:
      init_mmap();
      break;
    case io_method_t::IO_METHOD_USERPTR:
      init_userp();
      break;
    case io_method_t::IO_METHOD_UNKNOWN:
      // TODO(flynneva): log something
      break;
  }
}

void UsbCam::close_device()
{
  // Device is already closed
  if (m_fd == -1) {return;}

  // Teardown must never throw: a destructor or error-unwind path calling
  // shutdown() would std::terminate on a char* / runtime_error escape.
  if (-1 == close(m_fd)) {
    log(
      log_level_t::ERROR,
      "usb_cam: close(" + m_device_name + ") failed: " + strerror(errno));
  }

  m_fd = -1;
}

void UsbCam::open_device()
{
  struct stat st;

  if (-1 == stat(m_device_name.c_str(), &st)) {
    throw std::runtime_error(
            std::string("Cannot stat '") + m_device_name + "': " + strerror(errno));
  }

  if (!S_ISCHR(st.st_mode)) {
    throw std::runtime_error("'" + m_device_name + "' is not a character device");
  }

  m_fd = open(m_device_name.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == m_fd) {
    throw std::runtime_error(
            std::string("Cannot open '") + m_device_name + "': " + strerror(errno));
  }
}

void UsbCam::configure(
  parameters_t & parameters, const io_method_t & io_method)
{
  m_device_name = parameters.device_name;
  m_io = io_method;

  // Open device file descriptor before anything else
  open_device();

  m_image.width = static_cast<int>(parameters.image_width);
  m_image.height = static_cast<int>(parameters.image_height);
  m_image.set_number_of_pixels();

  // Do this before calling set_bytes_per_line and set_size_in_bytes
  m_image.pixel_format = set_pixel_format(parameters);
  m_image.set_bytes_per_line();
  m_image.set_size_in_bytes();
  m_framerate = parameters.framerate;

  init_device();
}

void UsbCam::start()
{
  start_capturing();
}

void UsbCam::shutdown()
{
  // Each step is isolated: a failure in one must not skip the others, and
  // nothing here may throw (destructor and error-unwind call this).
  try {
    stop_capturing();
  } catch (const std::exception & e) {
    log(
      log_level_t::WARN,
      std::string("usb_cam: stop_capturing during shutdown: ") + e.what());
  } catch (...) {
    log(log_level_t::WARN, "usb_cam: stop_capturing during shutdown: unknown error");
  }
  try {
    uninit_device();
  } catch (const std::exception & e) {
    log(
      log_level_t::WARN,
      std::string("usb_cam: uninit_device during shutdown: ") + e.what());
  } catch (...) {
    log(log_level_t::WARN, "usb_cam: uninit_device during shutdown: unknown error");
  }
  try {
    close_device();
  } catch (const std::exception & e) {
    log(
      log_level_t::WARN,
      std::string("usb_cam: close_device during shutdown: ") + e.what());
  } catch (...) {
    log(log_level_t::WARN, "usb_cam: close_device during shutdown: unknown error");
  }
}

/// @brief Grab new image from V4L2 device, return pointer to image
/// @return pointer to image data
char * UsbCam::get_image()
{
  if ((m_image.width == 0) || (m_image.height == 0)) {
    return nullptr;
  }
  // grab the image
  grab_image();
  return m_image.data;
}

/// @brief Overload get_image so users can pass in an image pointer to fill
/// @param destination destination to fill in with image
void UsbCam::get_image(char * destination)
{
  if ((m_image.width == 0) || (m_image.height == 0)) {
    return;
  }
  // Set the destination pointer to be filled
  m_image.data = destination;
  // grab the image
  grab_image();
}

std::vector<capture_format_t> UsbCam::get_supported_formats()
{
  m_supported_formats.clear();
  struct v4l2_fmtdesc * current_format = new v4l2_fmtdesc();
  struct v4l2_frmsizeenum * current_size = new v4l2_frmsizeenum();
  struct v4l2_frmivalenum * current_interval = new v4l2_frmivalenum();

  current_format->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  current_format->index = 0;
  for (current_format->index = 0;
    usb_cam::utils::xioctl(
      m_fd, VIDIOC_ENUM_FMT, current_format) == 0;
    ++current_format->index)
  {
    current_size->index = 0;
    current_size->pixel_format = current_format->pixelformat;

    for (current_size->index = 0;
      usb_cam::utils::xioctl(
        m_fd, VIDIOC_ENUM_FRAMESIZES, current_size) == 0;
      ++current_size->index)
    {
      current_interval->index = 0;
      current_interval->pixel_format = current_size->pixel_format;
      current_interval->width = current_size->discrete.width;
      current_interval->height = current_size->discrete.height;
      for (current_interval->index = 0;
        usb_cam::utils::xioctl(
          m_fd, VIDIOC_ENUM_FRAMEINTERVALS, current_interval) == 0;
        ++current_interval->index)
      {
        if (current_interval->type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          capture_format_t capture_format;
          capture_format.format = *current_format;
          capture_format.v4l2_fmt = *current_interval;
          m_supported_formats.push_back(capture_format);
        }
      }  // interval loop
    }  // size loop
  }  // fmt loop

  delete (current_format);
  delete (current_size);
  delete (current_interval);

  return m_supported_formats;
}

void UsbCam::grab_image()
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO(&fds);
  FD_SET(m_fd, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(m_fd + 1, &fds, NULL, NULL, &tv);

  if (-1 == r) {
    if (EINTR == errno) {
      // interruped (e.g. maybe Ctrl + c) so don't throw anything
      return;
    }

    log(
      log_level_t::ERROR,
      std::string("Something went wrong, exiting...") + std::to_string(errno));
    throw std::runtime_error(std::string("select() failed: ") + strerror(errno));
  }

  if (0 == r) {
    log(log_level_t::ERROR, "Select timeout, exiting...");
    throw std::runtime_error("select timeout");
  }

  read_frame();
}

// enables/disables auto focus
bool UsbCam::set_auto_focus(int value)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_QUERYCTRL), &queryctrl)) {
    if (errno != EINVAL) {
      log(log_level_t::ERROR, "VIDIOC_QUERYCTRL");
      return false;
    } else {
      log(log_level_t::WARN, "V4L2_CID_FOCUS_AUTO is not supported");
      return false;
    }
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    log(log_level_t::WARN, "V4L2_CID_FOCUS_AUTO is not supported");
    return false;
  } else {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == usb_cam::utils::xioctl(m_fd, static_cast<int>(VIDIOC_S_CTRL), &control)) {
      log(log_level_t::ERROR, "VIDIOC_S_CTRL");
      return false;
    }
  }
  return true;
}

/**
* Set video device parameter via ioctl V4L2 controls.
*
* @param param The name of the parameter to set
* @param value The value to assign
* @return true on success, false on failure
*/
bool UsbCam::set_v4l_parameter(const std::string & param, int value)
{
  // Parameter name to V4L2 control ID mapping
  static const std::unordered_map<std::string, int> param_map = {
    {"brightness", V4L2_CID_BRIGHTNESS},
    {"contrast", V4L2_CID_CONTRAST},
    {"saturation", V4L2_CID_SATURATION},
    {"sharpness", V4L2_CID_SHARPNESS},
    {"gain", V4L2_CID_GAIN},
    {"white_balance_temperature_auto", V4L2_CID_DO_WHITE_BALANCE},
    {"white_balance_temperature", V4L2_CID_WHITE_BALANCE_TEMPERATURE},
    {"exposure_auto", V4L2_CID_EXPOSURE_AUTO},
    {"exposure_absolute", V4L2_CID_EXPOSURE_ABSOLUTE},
    {"focus_auto", V4L2_CID_FOCUS_AUTO},
    {"focus_absolute", V4L2_CID_FOCUS_ABSOLUTE},
  };

  // Look up control ID
  auto it = param_map.find(param);
  if (it == param_map.end()) {
    log(log_level_t::WARN, "usb_cam: unknown parameter '" + param + "'");
    return false;
  }
  int control_id = it->second;

  // Query if the control exists on this device
  struct v4l2_queryctrl query;
  CLEAR(query);
  query.id = control_id;
  if (ioctl(m_fd, VIDIOC_QUERYCTRL, &query) < 0) {
    log(log_level_t::WARN, "usb_cam: parameter '" + param + "' not supported by device");
    return false;
  }

  // Clamp value to device range
  int clamped = value;
  if (clamped < query.minimum) {clamped = query.minimum;}
  if (clamped > query.maximum) {clamped = query.maximum;}

  // Set the control value
  struct v4l2_control ctrl;
  CLEAR(ctrl);
  ctrl.id = control_id;
  ctrl.value = clamped;
  if (ioctl(m_fd, VIDIOC_S_CTRL, &ctrl) < 0) {
    log(
      log_level_t::WARN,
      "usb_cam: failed to set '" + param + "' to " + std::to_string(clamped));
    return false;
  }

  return true;
}

/**
* Set video device parameter via string value (converted to int).
*
* @param param The name of the parameter to set
* @param value The value to assign as string
* @return true on success, false on failure
*/
bool UsbCam::set_v4l_parameter(const std::string & param, const std::string & value)
{
  try {
    int int_value = std::stoi(value);
    return set_v4l_parameter(param, int_value);
  } catch (const std::exception & e) {
    log(
      log_level_t::WARN,
      "usb_cam: invalid value '" + value + "' for parameter '" + param + "'");
    return false;
  }
}

}  // namespace usb_cam
