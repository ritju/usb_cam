---
feature: crashlog-persist
status: delivered
updated: 2026-10-19
branch: feature/crashlog-persist
commits: ddca4e5..4c5af47
---

# crashlog-persist

## Report

**What was built** — Crash/exit logs now reach disk, and the MJPEG path no longer NULL-derefs under memory pressure. `install_crash_handlers()` snapshots `/proc/self/maps` at start and writes `usb_cam_last_gasp.log` + `usb_cam_maps_at_crash.txt` into `~/.ros/log` on SIGSEGV/ABRT/BUS/FPE/ILL (so a `(deleted)` mapping can be identified later). `atexit` and the node destructor flush stdio. Diagnostic switch `USB_CAM_UNBUFFERED_LOG=1` (launch `SetEnvironmentVariable` + node `setenv`) sets `RCUTILS_LOGGING_BUFFERED_STREAM=0` so every line hits the file before an OOM-kill — default **off**. MJPEG `convert()`/ctor check every FFmpeg alloc and never `memcpy`/`sws_scale`/`av_image_copy_to_buffer` through NULL or past a tightly packed ROS buffer. Also: timer-first destructor order, null-safe compressed publishers and `parameters_callback`, and a periodic dropped-frame ERROR summary.

**Verification** —
- `colcon build --packages-select usb_cam` — **PASS**
- `ctest -R 'test_usb_cam_utils|test_pixel_formats'` — **PASS** (incl. `test_last_gasp_and_maps_snapshot`, `test_ros_log_dir_default`, `test_install_crash_handlers_caches_full_path`)
- `ament_uncrustify` / `ament_cpplint` on touched files — **PASS** (`usb_cam_node.cpp` Allman style is PRE-EXISTING)
- Reviewer #1: D1/D3/D4 accept; 2 CRITICAL (sizeof(char*) path truncation, padded memset) → fixed; re-review: C1 resolved, C2 residual at `av_image_copy_to_buffer` → fixed with `align=1` + packed `dst_size`. No further criticals.

**Journey log** —
1. `sizeof(fn())` where `fn` returns `char*` is pointer width (8), not the buffer. That silently truncated the cached crash dir to `"/home/s"` — the unit test called `write_last_gasp(dir)` directly and never exercised `install_crash_handlers`. Always name the buffer length constant.
2. FFmpeg `av_image_get_buffer_size(..., align=N)` and `av_image_copy_to_buffer(..., align=N)` are row-padded. A ROS `sensor_msgs::Image` is tightly packed `w*3*h`. Use `align=1` for the copy, never `memset` with the padded size.
3. Signal handlers cannot use iostream (locks/malloc). `copy_proc_maps` now uses `open`/`read`/`write`.
4. Unbuffered ROS logs (`RCUTILS_LOGGING_BUFFERED_STREAM=0`) are the only thing that survives OOM-kill; signal handlers do not run on SIGKILL. Keep the switch diagnostic-only (IO amplification).
5. Residual: `fflush(nullptr)` does not flush rcutils' internal buffers; unbuffered mode is the reliable path. `m_avframe_rgb_size` (align=32) is still used for the internal FFmpeg frame, which is correct.

## [S1] Problem

Field: before OOM, kernel shows `usb_cam_node_ex[pid]: segfault at 0 ... error 4 in (deleted)[...+a1000]` (NULL read inside a deleted/replaced .so). Critical ROS logs at process exit never reach the file, so the crash is hard to locate.

**Why exit logs are lost**

- rcutils file logs are buffered; SIGKILL/OOM-kill or a raw SIGSEGV dies without flushing.
- The node is a **child process in a container** (`rgb_camera_manager` Popen → `ros2 launch`); `output='screen'` text dies with the parent console.
- No `atexit`/signal flush, no maps/core breadcrumbs.

**Ranked crash causes**

| Rank | ID | Mechanism | Evidence |
|---|---|---|---|
| 1 | H1 | MJPEG: `av_packet_alloc()` unchecked → NULL on ENOMEM; `av_new_packet` failure then `memcpy(avpacket->data,…)` — NULL read **inside libavcodec** | `formats/mjpeg.hpp:190-192`; field configs all `mjpeg2rgb` |
| 2 | H2 | `sws_getContext` / `av_frame_get_buffer` failures only logged → `sws_scale` on NULL frame data **inside libswscale** | `mjpeg.hpp:114,148-155,216-219` |
| 3 | H4 | Init failure leaves `m_timer` null but `parameters_callback` still registered → `m_timer->reset()` | `usb_cam_node.cpp:89-90` + early returns |
| — | H3 | `pub_compressed` default true; `m_compressed_*` only created for `mjpeg2rgb` | `usb_cam_node.cpp:85,223-231,717-721` |
| — | (deleted) | .so replaced under a live process (`colcon build` / apt) or install on tmpfs | deploy issue |

## [S2] Design

**D1 Diagnostics switch — unbuffered ROS logs.** Env `USB_CAM_UNBUFFERED_LOG=1` (set in launch before node start) sets `RCUTILS_LOGGING_BUFFERED_STREAM=0` so every line `write()`s immediately and survives OOM-kill. **Default off** — only for diagnosis; constant-on amplifies IO.

**D2 Exit/crash force-flush + breadcrumbs.**
- `atexit` / node teardown: flush rcutils output buffers (`fflush(NULL)` fallback).
- Signal handler for SIGSEGV/SIGABRT/SIGBUS/SIGFPE/SIGILL (not SIGKILL): async-signal-safe `write(2)` of a last-gasp line to `~/.ros/log/usb_cam_last_gasp.log`, copy `/proc/self/maps` to `usb_cam_maps_at_crash.txt`, then restore default handler and re-raise.
- At init: snapshot `/proc/self/maps` + linked-lib mtime/size to `~/.ros/log/usb_cam_maps_<pid>.log` so a later `(deleted)` mapping can be identified.

**D3 NULL guards.**
- H1/H2: check `av_packet_alloc`, `av_new_packet`, `av_frame_alloc`, `avcodec_alloc_context3`, `sws_getContext`, `av_frame_get_buffer`. On failure log + return/throw; never `memcpy`/`sws_scale` through NULL.
- H4: `parameters_callback` no-ops when `m_timer` is null.
- H3: `take_and_send_image_mjpeg` returns if compressed msg/publisher missing.

**D4 Breadcrumbs.** Frame-drop counter (in addition to 5s THROTTLE) emits a periodic ERROR summary so burst patterns before a crash are visible.

## [S3] Out of Scope

- Deploy-side versioned install / avoid live rebuild (documented only).
- QoS depth / copy-count memory optimization.
- `rgb_camera_manager` stdout redirect (other package).
- Always-on unbuffered logging.

## Tasks

- [x] T1: Spec — acceptance: file with H1–H4 + D1–D4 (covers: S1, S2, S3)
- [x] T2: H1/H2 FFmpeg alloc checks in `mjpeg.hpp` — acceptance: every alloc failure path returns/throws; no memcpy/sws_scale on NULL (covers: D3)
- [x] T3: H3/H4 null guards in `usb_cam_node.cpp` — acceptance: callback safe without timer; compressed path safe without pubs (covers: D3)
- [x] T4: atexit + signal last-gasp + maps snapshot — acceptance: after a forced abort, `~/.ros/log` has last_gasp + maps; helper unit-testable (covers: D2, D4)
- [x] T5: `USB_CAM_UNBUFFERED_LOG` switch in launch — acceptance: default off; on → `RCUTILS_LOGGING_BUFFERED_STREAM=0` set before node start (covers: D1)
- [x] T6: Verify + Review + Finalize — acceptance: gtests green/PRE-EXISTING; review pass; `delivered` (covers: all; depends: T2–T5)
