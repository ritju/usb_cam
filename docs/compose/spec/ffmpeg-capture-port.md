---
feature: ffmpeg-capture-port
status: in-progress
updated: 2026-10-22
branch: feature/ffmpeg-capture-port
commits: # leave empty while in progress; fill at delivery
---

# FFmpeg Capture Port onto develop

## Report

## [S1] Problem

`feature/ffmpeg-capture` 已用 FFmpeg（libavdevice `video4linux2` demuxer + libavformat/libavcodec/swscale）整体替换 V4L2 ioctl/mmap 采集，但其分叉点之后 `develop` 合入了 crashlog-persist、frame-validate-logger、log-throttle、sizeimage-guard、zombie-reaper 等硬化，双方重写了同一批文件，无法直接 merge。需要从 `develop` 切出新分支，移植 FFmpeg 采集核心（不含 GPU/GpuMat 池提交），使相机初始化与取流全部经 FFmpeg 封装、不直接操作设备 fd，同时保留仍然适用的硬化能力。

## [S2] Design

### 架构

```
/dev/video*  --avformat_open_input("video4linux2")-->  FfmpegCapture
  opts: video_size, input_format/pixel_format, framerate, timestamps=mono2abs
       --av_read_frame --> decode(mjpeg|rawvideo) --> swscale | Y10_PACK | packet passthrough
       --pts --> timespec (wallclock fallback)
UsbCam (facade) --> UsbCamNode (crashlog + log bridge + undistort + publish)
```

库层保持 rclcpp-free 且 OpenCV-free；OpenCV/cv_bridge 仅在 node。

### 移植来源与范围

- 源：`feature/ffmpeg-capture` @ `e6d4b50`（核心）+ `13ea620`（重写）。**不含** `878c574`/`9a7d648`/`73615a4`（GpuMat 池与去畸变计时）。
- 策略：整体检出 `e6d4b50` 的采集文件，手工回植 develop 硬化；不做 git merge。

### 文件面

| 动作 | 路径 |
|------|------|
| 整体取自 e6d4b50 | `ffmpeg_capture.hpp/cpp`, `format_mapping.hpp/cpp`, `timestamps.hpp`, `test_{format_mapping,timestamps,controls,ffmpeg_capture_file}.cpp` |
| 新建 | `include/usb_cam/crashlog.hpp`（自 `utils.hpp` 迁出）、`include/usb_cam/logging.hpp`（自 `usb_cam.hpp` 迁出） |
| 手工合并 | `usb_cam.hpp/cpp`、`usb_cam_node.hpp/cpp`、`CMakeLists.txt`、`package.xml` |
| 删除 | `conversions.hpp`、`utils.hpp`、`formats/**`、`test_{pixel_formats,usb_cam_lib,usb_cam_utils,gtest_main}.cpp` |

### UsbCam 公开 API（facade）

- `configure(const parameters_t &)` / `start()` / `shutdown()` / `start_capturing()` / `stop_capturing()`
- `bool get_image(char * dest)` / `char * get_image()` — **软失败返回 false**（见下）
- `set_camera_control(name, value)` — 恒 false + warn-once（无 demuxer option）
- `set_logger(log_fn_t)` — 转发至 `FfmpegCapture`，库内默认 iostream sink
- 几何/编码/时间戳 getters；`get_last_payload_size()` / `is_passthrough()` 供 raw_mjpeg 透传

### 格式映射

11 个旧 `pixel_format` 名（mjpeg2rgb, raw_mjpeg, yuyv2rgb, yuyv, uyvy2rgb, uyvy, mono8, mono16, rgb8, m4202rgb, y102mono8）→ input_format / decode_policy / AVPixelFormat / ROS encoding；未知名 throw `invalid_argument`。`pixel_format` 优先于 `av_device_format`。

### 软失败取流（相对 e6d4b50 的刻意变更）

逐帧瞬时故障 **不抛异常**，`note_dropped_frame`（首帧 + 每 30 帧 ERROR，否则 WARN_THROTTLE）并返回无帧；node 跳过本拍发布（绝不发脏缓冲）：

- `av_read_frame` 失败 / 64 次尝试耗尽
- `avcodec_send_packet`/`receive_frame` 硬错误（非 EAGAIN）
- 空包（`size<=0` / null data）
- 透传超长包（`size > dest_size`）— 禁止 `std::min` 截断 JPEG

前置条件失败仍抛：未 open、空 dest、configure 期 `sws_getContext` 失败。

`frame_payload_error` 的 V4L2 签名不复用，改为 FFmpeg 语义检查（可测试的 `packet_payload_error(size, bound)`）。

### 硬化保留 / 丢弃

| 项 | 处理 |
|----|------|
| crashlog-persist | `crashlog.hpp` + node install/atexit/UNBUFFERED_LOG/dtor flush |
| frame-validate-logger + log-throttle | `logging.hpp` + node `RCLCPP_*_THROTTLE` 桥（7 级，`kLogThrottleMs=5000`） |
| sizeimage-guard | 随 V4L2 路径删除（几何从 `codecpar` 回读；拷贝有界） |
| available_devices / io_method 解析 | 删除（`io_method` 参数仍接受 + warn-once-ignore） |
| zombie-reaper launch | 不动 |
| UVC 控件真实控制 | 不实现（stub） |

### 构建

pkg-config `IMPORTED_TARGET`：libavformat/libavdevice/libavcodec/libavutil/libswscale。`usb_cam` 库仅链 libav*；OpenCV/cv_bridge 在 `_node`。`package.xml` 去掉 `v4l-utils`。

## [S3] Out of Scope

- GPU/GpuMat 池与去畸变计时提交（`878c574`..`73615a4`）
- UVC 控件兼容层（VIDIOC_S_CTRL / v4l2-ctl）
- undistort 算法与 GPU map 缓存逻辑变更
- `/map/config` 部署 YAML
- 真机冒烟（本机无 `/dev/video*`）

## Tasks

- [ ] T1: 分支 + 抽出 `crashlog.hpp`/`logging.hpp` + 拆分测试 — acceptance: 纯搬迁后现有 V4L2 树构建与测试仍绿 (covers: S2)
- [ ] T2: 整体检出 e6d4b50 采集文件，删除 V4L2 表面 — acceptance: 文件面符合 S2；库可编译 (covers: S2; depends: T1)
- [ ] T3: facade + logger 转发 + 软失败 grab + 包检查 — acceptance: `get_image` 软失败返回 false 不抛；透传不截断；丢帧有日志 (covers: S2; depends: T2)
- [ ] T4: node 硬化回植 + CMake/package — acceptance: 节点构建；7 级日志桥；crashlog 完好；grab 失败不发布 (covers: S2; depends: T3)
- [ ] T5: 6 目标测试套件 — acceptance: `colcon test` 全绿 (covers: S2; depends: T4)
- [ ] T6: 验证门 + 本文档定稿 — acceptance: 构建/测试/grep 门通过；spec delivered (covers: S2; depends: T5)
- [ ] T7: 真机冒烟 — acceptance: 启动/参数兼容/hz/stamp/set_capture/双相机/respawn 通过 (covers: S2; depends: T6) **本机无相机设备**
