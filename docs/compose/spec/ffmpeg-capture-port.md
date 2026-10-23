---
feature: ffmpeg-capture-port
status: delivered
updated: 2026-10-22
branch: feature/ffmpeg-capture-port
commits: dfb0bff..563dfc6
---

# FFmpeg Capture Port onto develop

## Report

**What was built** — 从 `develop` 切出 `feature/ffmpeg-capture-port`，移植 `feature/ffmpeg-capture` @ `e6d4b50` 的 FFmpeg 采集核心：`FfmpegCapture`（libavdevice `video4linux2` 单 fd 生命周期）、`format_mapping`（11 个旧 `pixel_format` 名）、`timestamps`（pts→timespec），`UsbCam` 收敛为 facade，删除全部 `VIDIOC_*`/`formats/*`/`utils.hpp`。相机初始化与取流完全经 FFmpeg 封装，不再直接操作设备 fd。回植 develop 硬化：`crashlog.hpp`（自 `utils.hpp` 迁出，含 `kCrashLogDirLen` 路径缓存）、`logging.hpp` + `set_logger` 转发 + 节点 `RCLCPP_*_THROTTLE` 桥、`note_dropped_frame` 丢帧统计。刻意变更：`grab` 软失败（瞬时故障丢帧返回 false，node 跳过本拍发布，绝不发脏缓冲）；透传超长包拒绝而非 `std::min` 截断；`avcodec_send_packet` EAGAIN 先 drain 再 resend，不再静默丢包。不含 GPU/GpuMat 池提交（`878c574`..`73615a4`）。

**Verification** — `colcon build --merge-install --packages-select usb_cam` PASS；gtest 24/24 PASS（format_mapping 5 + timestamps 5 + controls 3 + ffmpeg_capture_file 6 + crashlog 3 + logging 2）；`grep -rnE 'videodev2|VIDIOC_|V4L2_PIX_FMT|xioctl|v4l2_'` 于 include/src/test 为空。lint：uncrustify 对 `usb_cam_node.*` Allman 风格、flake8 对 `launch/*.py`、cppcheck 可执行缺失均为 **PRE-EXISTING**。真机冒烟 **未跑**（本机无 `/dev/video*`，T7 遗留）。

**Journey log** —
1. 分叉后 develop 合入 5 个硬化特性、ffmpeg 分支重写同一批文件，`git merge` 冲突不可维护 → 按文件整体检出 `e6d4b50` + 手工回植硬化。
2. `e6d4b50` 的 `grab` 对瞬时故障 **抛异常**，而 node timer 无 try/catch，USB 抖动会杀掉整流 → 改为软失败 + 跳过发布（相对源分支的刻意行为变更，已在 S2 记录）。
3. 评审指出 `avcodec_send_packet` EAGAIN 时 packet 已 unref 却未 resend，静默丢包 → 保持 packet 存活，drain 一帧后 resend。
4. 评审指出 Y10_PACK 缺 `& 0x3FF` 掩码、`ensure_compressed_publishers` 文档串错误、grab 路径无测试 → 同轮修复。
5. 预存问题（不在本 port 范围）：`undistort_image=true` 时 resize 守卫不刷新 encoding，第 2 帧起 RGB 被当 BGR8 解读——develop `dfb0bff` 即存在，值得单独开 issue。
6. **漏网真 bug**：FFmpeg 4.x 下 `av_find_input_format("video4linux2")` 在 `avdevice_register_all()` 前返回 NULL，`open()` 必抛 “demuxer not available”。y4m 文件测试走 libavformat 内建 demuxer，掩盖了这条路径 → 同 `av_log_set_level` 做一次性 bring-up，加 demuxer 注册回归测试；FFmpeg 5+ 用版本宏跳过。
7. **频繁切换 OOM（uvcvideo URB -1 + dwMaxPayload 垃圾值）** 与 sizeimage-guard H1–H3 同族；FFmpeg port 拆掉了 `validate_sizeimage` 后 REQBUFS 会照单全收驱动垃圾。补 `device_preflight`：设备就绪门、只读 `VIDIOC_G_FMT` sizeimage 拒绝、`VmPeak` 前后追踪、close→open 500ms 去抖；未就绪/尺寸不合理直接拒绝进 FFmpeg。
8. **僵死 UVC 流靠手动 ffplay 恢复** — 不操作相机时持续吐异常数据，`ffplay` 一次 S_FMT+STREAM 往返即好。补自动恢复看门狗：坏帧（纯色填充 / raw_mjpeg 无 SOI）与连续软失败累积后按 L1 flush → L2 流重启 → L3 整机 reopen 升级（L3 即 ffplay 等价），带 1s 限速；`recover_now()` 可手动触发。
9. **open 期 sizeimage 垃圾只挡不救** — 纯 `G_FMT` 预检会 throw，L3 重开撞同一堵墙，失败后还留下无 fd 的 `grab` 硬抛。补 `kick_s_fmt_and_probe`（S_FMT 踢一脚让驱动重算 sizeimage，即 ffplay 在 open 做的事）+ 3 次重试；reopen 失败保持可恢复（orphaned 状态直接升 L3，`grab` 不再 throw）。
10. **G_FMT 干净仍 OOM（对照 ffplay.c）** — ffplay **无** sizeimage/REQBUFS 防护；缺口是只对“当前格式”G_FMT，FFmpeg 在 `avformat_open_input` 内再 S_FMT 后 REQBUFS 新 sizeimage。改为 `negotiate_and_verify`：**无条件**用目标几何 S_FMT→G_FMT 通过才进 FFmpeg；`av_max_alloc` 封顶用户态 `av_new_packet`；`avformat_alloc_context`+interrupt；open 失败指数退避，streak≥5 停火。
11. **相对 s1_gpu 的 B/R 互换** — s1_gpu 用 `sizeof(vector)` 恒真，每帧把 encoding 重置为 `rgb8`，掩盖 undistort 后 BGR8 不刷新问题。resize 条件修为 `data.size()` 后真 bug 暴露。每帧 grab 前 `encoding = get_encoding()`。

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
| sizeimage-guard | V4L2 `validate_sizeimage` 随路径删除；等价防护回到 `device_preflight.hpp`：open 前只读 `VIDIOC_G_FMT` 拒绝垃圾 sizeimage + 几何合理性检查 + 就绪门 + open 去抖 |
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

- [x] T1: 分支 + 抽出 `crashlog.hpp`/`logging.hpp` + 拆分测试 — acceptance: 纯搬迁后现有 V4L2 树构建与测试仍绿 (covers: S2)
- [x] T2: 整体检出 e6d4b50 采集文件，删除 V4L2 表面 — acceptance: 文件面符合 S2；库可编译 (covers: S2; depends: T1)
- [x] T3: facade + logger 转发 + 软失败 grab + 包检查 — acceptance: `get_image` 软失败返回 false 不抛；透传不截断；丢帧有日志 (covers: S2; depends: T2)
- [x] T4: node 硬化回植 + CMake/package — acceptance: 节点构建；7 级日志桥；crashlog 完好；grab 失败不发布 (covers: S2; depends: T3)
- [x] T5: 6 目标测试套件 — acceptance: `colcon test` 全绿 (covers: S2; depends: T4)
- [x] T6: 验证门 + 本文档定稿 — acceptance: 构建/测试/grep 门通过；spec delivered (covers: S2; depends: T5)
- [ ] T7: 真机冒烟 — acceptance: 启动/参数兼容/hz/stamp/set_capture/双相机/respawn 通过 (covers: S2; depends: T6) **本机无相机设备**
