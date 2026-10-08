---
feature: ffmpeg-capture
status: delivered
updated: 2026-03-04
branch: feature/ffmpeg-capture
commits: b2c1a80..e6d4b50
---

# FFmpeg Capture Replace V4L2

## Report

**What was built** — `usb_cam` 的相机配置与数据采集从 V4L2 ioctl/mmap 整体迁移到 FFmpeg（libavdevice video4linux2 demuxer + libavformat + libavcodec + swscale）。新增 `FfmpegCapture`（单 fd 生命周期）、`format_mapping`（旧 `pixel_format` 名 → input_format/AVPixelFormat/ROS encoding）、`timestamps`（pts→timespec，墙钟回退）。`UsbCam` 收敛为 facade，删除 `formats/*`、`conversions.hpp`、`utils.hpp` 与全部 `VIDIOC_*` 路径。OpenCV 仅保留节点侧 GPU/CPU 去畸变与 JPEG 压缩。ROS 参数名全部兼容（`io_method` 忽略+warn）。UVC 控件经 FFmpeg options（当前 demuxer 无对应 option，故 11 个控件 warn-once 忽略）。

**Verification** — `colcon build --packages-select usb_cam` PASS；gtest `test_format_mapping`(5) + `test_timestamps`(5) + `test_controls`(2) + `test_ffmpeg_capture_file`(3) = 15/15 PASS；`grep -R 'videodev2|VIDIOC_|pixel_format_base|conversions.hpp'` 为空。lint：copyright/lint_cmake/pep257 PASS；cpplint/uncrustify 对 `usb_cam_node.*`、flake8 对 `launch/*.py` 为 **PRE-EXISTING** fork 风格问题；cppcheck 可执行文件缺失 **PRE-EXISTING**。真机冒烟 **未跑**（本机无 `/dev/video*`）。

**Journey log** —
1. 用户在 Grill 中将「OpenCV 采集」修正为「ffmpeg 采集 + OpenCV GPU 去畸变」；UVC 控件选择「仅 ffmpeg options、能力降级」。
2. FFmpeg 4.4 `avformat_open_input` 需非 const `AVInputFormat*`；`AVPacket` 用 `dts` 而非 `pkt_dts`。
3. 评审指出 swscale GRAY16→GRAY8 ≈`>>8`，与旧 Y10 `>>2` 不符——新增 `Y10_PACK` 手工打包。
4. compressed publisher 空指针是生命周期问题：运行时 `pub_compressed` 打开仍可能解引用空指针 → 改为 `ensure_compressed_publishers()` 懒创建。
5. `raw_mjpeg` 透传 JPEG 字节不能走 cv_bridge BGR8 再编码；改为直接把 payload 作为 CompressedImage。

## [S1] Problem

`usb_cam` 采集与相机配置直接建立在 V4L2 ioctl/mmap/userptr/read 之上，维护成本高，并带来多开句柄、格式协商结果不回读、手写 YUV 转换等问题。需要以 FFmpeg（libavdevice/libavformat/libavcodec/swscale）完全替换相机配置与数据采集；OpenCV 仅保留 GPU/CPU 去畸变与 JPEG 压缩发布。

## [S2] Design

### 架构

```
/dev/video*  --avformat_open_input("video4linux2")-->  FfmpegCapture
  opts: video_size, input_format/pixel_format, framerate, timestamps=mono2abs
       --av_read_frame --> decode(mjpeg|rawvideo) --> swscale/Y10_PACK --> packed dest
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
- `set_camera_control(name, value)` — 今日恒 false + warn
- `get_image_width/height/size_in_bytes/step()` / `get_encoding()` / `get_image_timestamp()` / `is_capturing()` / `get_device_name()`
- `get_last_payload_size()` / `is_passthrough()` — raw_mjpeg 压缩路径用
- `parameters_t.framerate` 为 `double`

### 格式映射（`pixel_format` 优先于 `av_device_format`）

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
| `m4202rgb` | `yuv420p` | raw+swscale | RGB24 | `rgb8` |
| `y102mono8` | `gray16le` | `Y10_PACK` (`>>2`) | GRAY8 | `mono8` |
| 其他 | throw `invalid_argument` | | | |

### 控件（能力降级）

11 个 UVC 控件参数 → `set_camera_control` 返回 false + warn-once，不抛异常。

### 时间戳

`timestamps=mono2abs`；pts 优先 `best_effort` → `pts` → `dts`；回退 `CLOCK_REALTIME`。

### 构建

pkg-config `libavformat libavdevice libavcodec libavutil libswscale`；`usb_cam` lib 仅链 libav*；OpenCV/cv_bridge 在 node；`package.xml` 无 `v4l-utils`。

### 顺带修复

| Bug | Fix |
|-----|-----|
| compressed publisher 空指针（含运行时开关） | `ensure_compressed_publishers()` 懒创建 |
| `raw_mjpeg` 误走 cv_bridge | 透传 JPEG 直接作 CompressedImage |
| `parameters_t.framerate` int 截断 | 改 double |
| 未知 `io_method` shutdown | warn + 继续 |
| 旧 `rgb8`→RGB332 | 改 `rgb24` |
| S_FMT 结果未回读 | 从 codecpar 回读宽高 |

## [S3] Out of Scope

- 不改 CAMERAS env / launch respawn 约定
- 不实现 UVC 控件兼容层（VIDIOC_S_CTRL / v4l2-ctl）
- 不改 undistort 算法与 GPU map 缓存逻辑
- 不改 `/map/config` 部署 YAML 内容
- 真机冒烟（T9）在目标机执行，不在本开发机

## Tasks

- [x] T1: format_mapping 映射表 + 单测 — acceptance: 11 个旧名+未知 throw+优先级 `mjpeg2rgb`+`YUV422P`→`mjpeg` 用例绿 (covers: S2)
- [x] T2: timestamps + set_camera_control + 单测 — acceptance: pts 换算/回退用例绿；11 控件 false 不抛 (covers: S2)
- [x] T3: FfmpegCapture open/configure/grab/stop/close + 文件源测试 — acceptance: RGB24 缓冲尺寸正确；`-Werror` 编译过 (covers: S2)
- [x] T4: UsbCam facade 重写 — acceptance: 库链接；header 无 VIDIOC_/v4l2 符号 (covers: S2; depends: T3)
- [x] T5: Node 最小适配 — acceptance: `take_and_send_image` diff ≤ 数行；compressed null-deref 修复 (covers: S2; depends: T4)
- [x] T6: CMake/package.xml — acceptance: `grep -R videodev2` 为空且构建过 (covers: S2; depends: T4)
- [x] T7: 删除 formats/*、utils.hpp 等死代码 — acceptance: grep 无残留，colcon test 绿 (covers: S2; depends: T5, T6)
- [x] T8: lint + 全量本地门 — acceptance: gtest 0 failures；fork 既有 lint 问题标 PRE-EXISTING (covers: S2; depends: T7)
- [ ] T9: 真机冒烟 — acceptance: 启动/参数兼容/hz/stamp/set_capture/双相机/respawn 通过 (covers: S2; depends: T8) **未完成：本机无相机设备**
