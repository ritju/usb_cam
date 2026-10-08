---
feature: ffmpeg-capture
status: designed
updated: 2026-03-04
branch: feature/ffmpeg-capture
commits: 
---

# FFmpeg Capture Replace V4L2

## Report

## [S1] Problem

`usb_cam` 采集与相机配置直接建立在 V4L2 ioctl/mmap/userptr/read 之上，维护成本高，并带来多开句柄、格式协商结果不回读、手写 YUV 转换等问题。需要以 FFmpeg（libavdevice/libavformat/libavcodec/swscale）完全替换相机配置与数据采集；OpenCV 仅保留 GPU/CPU 去畸变与 JPEG 压缩发布。

## [S2] Design

### 架构

```
/dev/video*  --avformat_open_input("video4linux2")-->  FfmpegCapture
  opts: video_size, input_format/pixel_format, framerate, timestamps=mono2abs
       --av_read_frame --> decode(mjpeg|rawvideo) --> swscale --> packed dest
       --pts --> timespec (fallback wallclock)
UsbCam (facade) --> UsbCamNode (undistort + publish)
```

### 新文件

| File | Role |
|------|------|
| `include/usb_cam/ffmpeg_capture.hpp` + `src/ffmpeg_capture.cpp` | `usb_cam::FfmpegCapture`：open/configure/grab/stop/close |
| `include/usb_cam/format_mapping.hpp` + `src/format_mapping.cpp` | 旧 `pixel_format` 名 → capture input_format / AVPixelFormat / ROS encoding |
| `include/usb_cam/timestamps.hpp` | pts + time_base → timespec；NOPTS/墙钟回退 |

### UsbCam 公开 API（瘦身后）

- `configure(const parameters_t &)` — 不再接收 io_method
- `start()` / `shutdown()` / `start_capturing()` / `stop_capturing()`
- `get_image(char * dest)` / `char * get_image()`
- `set_camera_control(name, value)` — 今日恒 false + warn（FFmpeg 无 UVC 控件 option）
- `get_image_width/height/size_in_bytes/step()` / `get_encoding()` / `get_image_timestamp()` / `is_capturing()` / `get_device_name()`
- `parameters_t.framerate` 改为 `double`（修截断）

删除：`get_fd/get_buffers/number_of_buffers/get_avcodec*/get_avframe/get_io_method/get_supported_formats/set_pixel_format/set_v4l_parameter/set_auto_focus/get_epoch_time_shift_us/capture_format_t/driver_supported_formats`。

### 格式映射（`pixel_format` 优先于 `av_device_format`）

部署 YAML 同时含 `pixel_format: mjpeg2rgb` 与 `av_device_format: YUV422P`；capture 用前者派生的 fourcc/codec，后者仅 `raw_mjpeg` 的 ROS encoding 标签或 raw 类型辅助。

| 旧名 | input_format | 解码 | 输出 AVPixelFormat | ROS encoding |
|------|--------------|------|-------------------|--------------|
| `mjpeg2rgb` | `mjpeg` | MJPEG+swscale | RGB24 | `rgb8` |
| `raw_mjpeg` | `mjpeg` | JPEG 透传 | — | 由 av_device_format 子表 |
| `yuyv2rgb` | `yuyv422` | raw+swscale | RGB24 | `rgb8` |
| `yuyv` | `yuyv422` | passthrough | YUYV422 | `yuv422_yuy2` |
| `uyvy2rgb` | `uyvy422` | raw+swscale | RGB24 | `rgb8` |
| `uyvy` | `uyvy422` | passthrough | UYVY422 | `yuv422` |
| `mono8` | `gray` | passthrough | GRAY8 | `mono8` |
| `mono16` | `gray16le` | passthrough | GRAY16LE | `mono16` |
| `rgb8` | `rgb24` | passthrough | RGB24 | `rgb8` |
| `m4202rgb` | `m420`→`yuv420p` | raw+swscale | RGB24 | `rgb8` |
| `y102mono8` | `y10`→`gray16le` | `>>2` | GRAY8 | `mono8` |
| 其他 | throw `invalid_argument` 列出支持列表 | | | |

swscale 源格式以实际 `frame->format` 为准，首帧懒创建。

### 控件（能力降级）

FFmpeg v4l2 demuxer 无 UVC 控件 AVOption。11 个控件参数（brightness/contrast/saturation/sharpness/gain/auto_white_balance/white_balance/autoexposure/exposure/autofocus/focus）→ `set_camera_control` 返回 false + warn-once，不抛异常。`apply_camera_controls()` 视 false 为非致命。

### 时间戳

- open 时 `timestamps=mono2abs`
- pts 优先级：`best_effort_timestamp` → `pts` → `pkt_dts`
- `timespec = pts * av_q2d(stream->time_base)`
- 回退：NOPTS / 非法 time_base / 结果 <2000 年或 >now+60s → `CLOCK_REALTIME`
- 删除 `get_epoch_time_shift_us` / `calc_img_timestamp`

### 参数兼容

全部 ROS 参数名保留。`io_method` 忽略 + warn-once（未知值不再 shutdown）。`/map/config/usb_cam/*.yaml` 零改动。

### 构建

- pkg-config：`libavformat libavdevice libavcodec libavutil libswscale`（SYSTEM includes）
- `usb_cam` lib 仅链 libav*；OpenCV/cv_bridge 移至 `usb_cam_node`
- `package.xml` 删除 `v4l-utils`，保留 `ffmpeg`

### 顺带修复

| Bug | Fix |
|-----|-----|
| compressed publisher 仅 mjpeg2rgb 创建 | 门控改为 `pub_compressed` |
| `parameters_t.framerate` int 截断 | 改 double |
| 未知 `io_method` shutdown | warn + 继续 |
| 旧 `rgb8`→RGB332 | 改 `rgb24` |
| S_FMT 结果未回读 | 从 codecpar 回读宽高 |

## [S3] Out of Scope

- 不改 CAMERAS env / launch respawn 约定
- 不实现 UVC 控件兼容层（VIDIOC_S_CTRL / v4l2-ctl）
- 不改 undistort 算法与 GPU map 缓存逻辑
- 不改 `/map/config` 部署 YAML 内容
- 不做多进程/多设备共享 FFmpeg 状态

## Tasks

- [ ] T1: format_mapping 映射表 + 单测 — acceptance: 11 个旧名+未知 throw+优先级 `mjpeg2rgb`+`YUV422P`→`mjpeg` 用例绿 (covers: S2)
- [ ] T2: timestamps + set_camera_control + 单测 — acceptance: pts 换算/回退用例绿；11 控件 false 不抛 (covers: S2)
- [ ] T3: FfmpegCapture open/configure/grab/stop/close + 文件源测试 — acceptance: RGB24 缓冲尺寸正确；`-Werror` 编译过 (covers: S2)
- [ ] T4: UsbCam facade 重写 — acceptance: 库链接；header 无 VIDIOC_/v4l2 符号 (covers: S2; depends: T3)
- [ ] T5: Node 最小适配 — acceptance: `take_and_send_image` diff ≤ 数行；compressed null-deref 修复 (covers: S2; depends: T4)
- [ ] T6: CMake/package.xml — acceptance: `grep -R videodev2` 为空且构建过 (covers: S2; depends: T4)
- [ ] T7: 删除 formats/*、utils.hpp 等死代码 — acceptance: grep 无残留，colcon test 绿 (covers: S2; depends: T5, T6)
- [ ] T8: lint + 全量本地门 — acceptance: colcon test 0 failures (covers: S2; depends: T7)
- [ ] T9: 真机冒烟 — acceptance: 启动/参数兼容/hz/stamp/set_capture/双相机/respawn 通过 (covers: S2; depends: T8)
