---
feature: sizeimage-guard
status: in-progress
updated: 2026-10-19
branch: feature/sizeimage-guard
commits: 
---

# sizeimage-guard

## Report

## [S1] Problem

`v4l2-ctl -d /dev/rgb_camera_back --get-fmt-video` sometimes reports `sizeImage` above 3 GB when system memory is low. For the deployed config (`config/params_back.yaml`: 1280x1024, `mjpeg2rgb`, `mmap`) a plausible `sizeimage` is a few hundred KB to ~2.6 MB worst-case; `kMaxSizeimageFactor = 4` allows ≈ 10.5 MB. A 3 GB value is ~1000× that bound — garbage, not a large frame.

`sizeImage` is `fmt.pix.sizeimage`, a `__u32` filled by the **kernel** UVC driver. Userspace `VIDIOC_S_FMT` zeroes the field and the driver recomputes it (`src/usb_cam.cpp:555-566`), so this package cannot legitimately write a 3 GB value. The corrupt value lives in UVC driver state; `usb_cam` is the trigger/amplifier.

### Feedback loop

```
garbage sizeimage (H1/H2)
  → REQBUFS(4 × sizeimage) kernel vmalloc  (H3 amplifier)
  → real 低内存 / OOM
  → node crash / respawn=True overlap
  → second fd + USB re-enum  (H2 trigger)
  → more garbage sizeimage (H1)
```

### Ranked root causes

| Rank | ID | Hypothesis | Evidence | Role |
|---|---|---|---|---|
| 1 | H1 | Corrupt UVC `dwMaxVideoFrameSize` / frame descriptor under memory pressure (partial `kmalloc`/`usb_control_msg`, or USB re-enumeration) | Authors' comment `src/usb_cam.cpp:456-463`; matches "sometimes / >3GB / low memory"; value persists via `v4l2-ctl` after the node dies | Kernel root — userspace can only refuse it |
| 2 | H2 | Second fd on an already-streaming UVC device | `available_devices()` leaks one O_RDONLY fd per V4L2 device every init (`include/usb_cam/utils.hpp:175-188` never calls `close(fd)`; invoked `src/usb_cam_node.cpp:173`); historical `popen(v4l2-ctl)` (removed `b2c1a80`); `respawn=True` overlap; manual `v4l2-ctl --set-fmt-video` | Userspace trigger |
| 3 | H3 | Unbounded `REQBUFS`/`mmap` of `4×sizeimage` creates an OOM feedback loop | Pre-7aa7d17 (and deployed `s1_gpu`) have no bound; develop now has `validate_sizeimage()` + `m_max_buffer_bytes` | Amplifier — guard present on develop |
| 4 | H4 | `throw strerror(errno)` / `throw errno` / `throw "..."` bypass `catch (const std::exception&)` | 9 sites in `src/usb_cam.cpp` (565, 576, 580, 613, 624, 628, 634, 767, 772); node catch at `src/usb_cam_node.cpp:218` catches only `std::exception` → no `shutdown()`, fd left open, raw `UsbCam*` leaked if the ctor throws | Error-path gap |
| 5 | H5 | Per-frame `VIDIOC_G_FMT` into the S_FMT struct smears the corrupt value | `read_frame()` `src/usb_cam.cpp:128` writes into `m_image.v4l2_fmt` every frame; that same struct is the S_FMT payload on reconfigure (pre-7aa7d17 had no `CLEAR`) | Observer / smear |

`validate_sizeimage()` also used ROS-format bpp (`yuyv2rgb` → 3 B/px) instead of device bpp (YUYV → 2 B/px); the 4× factor masked a 1.5× error in `expected`.

## [S2] Design

No new classes. Six contracts:

**C1 Exception contract.** Every throw out of `UsbCam` is a `std::exception` (`std::runtime_error` or `std::system_error`). Observable: `catch (const std::exception&)` always catches, including `configure()` failure on a bad device path.

**C2 Cleanup never throws.** `close_device()`, `stop_capturing()`, `uninit_device()`, `shutdown()`, and `~UsbCam` log and continue. Teardown must not `std::terminate`. The `start()` failure path uses the same guarded shutdown as `configure()`.

**C3 fd lifecycle.** `available_devices()` closes every fd it opens, success or failure. Optional `sysfs_dir` / `dev_prefix` arguments (defaulted to current constants) make the function testable without real `/dev/video*` nodes.

**C4 sizeimage gate.** `expected` is derived from the **device** fourcc bytes/pixel via `usb_cam::utils::device_sizeimage_estimate(fourcc, w, h)`, times `kMaxSizeimageFactor = 4`. Reject implausible `sizeimage` before `VIDIOC_REQBUFS`. `m_max_buffer_bytes` semantics unchanged (accepted sizeimage, or `limit` when the driver reports 0).

**C5 format-state hygiene.** `read_frame()` never writes `m_image.v4l2_fmt`. `process_image` memcpy is bounded by `min(bytes_used, size_in_bytes)`. `m_image.width/height/size_in_bytes` stay at construction (requested) values after `S_FMT` — converter objects freeze their pixel count at construction, so shrinking the ROS destination to the accepted geometry would let `YUYV2RGB`/`M4202RGB`/… write past it. `validate_sizeimage()` uses `pix.width/height` directly. A future geometry adopt must first re-sync converter-internal pixel counts.

**C6 msg sizing.** The ROS image buffer resize check uses `m_image_msg->data.size()`, not `sizeof(m_image_msg->data)`.

## [S3] Out of Scope

- H1 kernel/UVC driver fix — out of package control. Operator mitigations: re-plug the device, inspect `dmesg` for `uvcvideo`/`vb2`/OOM lines, avoid concurrent `v4l2-ctl --set-fmt-video` while streaming.
- FFmpeg capture rewrite (`feature/ffmpeg-capture`) — separate feature. Lesson for that design: never open a second fd on a live capture device; always refuse implausible driver buffer sizes before allocating.
- MJPEG `av_frame_get_buffer` / `av_new_packet` NULL checks (`include/usb_cam/formats/mjpeg.hpp`) — amplifier only; deferred with the FFmpeg rewrite.
- GPU undistort / GpuMat pooling, `get_image(char*)` contract, ROS parameter names, the 3s stabilize sleep.
- Deploying `7aa7d17` onto `s1_gpu` (ops decision).

## Tasks

- [ ] T1: Spec skeleton + [S1] root-cause report — acceptance: this file exists at `docs/compose/spec/sizeimage-guard.md` with H1–H5 ranking and file:line evidence; every contract covered (covers: S1, S2, S3)
- [ ] T2: `usb_cam::utils::device_sizeimage_estimate(fourcc, w, h)` helper + gtest cases — acceptance: tests fail before the helper exists, pass after; no `/dev/video*` needed (covers: C4)
- [ ] T3: Rewire `validate_sizeimage()` to use the device-bpp helper — acceptance: unit tests green; yuyv2rgb 640x480 limit = 4×640×480×2 (covers: C4; depends: T2)
- [ ] T4: Fix `available_devices()` fd leak + path-injection args + fd-count regression test — acceptance: fd-count stable across calls; production defaults unchanged (covers: C3)
- [ ] T5: Replace 9 bare throws with `std::runtime_error`/`std::system_error`; regression test on bad device path — acceptance: `EXPECT_THROW(..., std::exception)` passes; grep for bare throws is empty (covers: C1)
- [ ] T6: Non-throwing cleanup (`shutdown`/`close_device`/`~UsbCam`); guard `start()` catch — acceptance: no throw escapes a destructor path; error-path test leaves no open fd (covers: C2; depends: T5)
- [ ] T7: Format hygiene — drop per-frame G_FMT smear; clamp memcpy by `bytes_used`; keep construction geometry (converters freeze pixel count); `.size()` fix — acceptance: no `VIDIOC_G_FMT` in `read_frame`; `sizeof(vector)` gone; memcpy bounded; `-Werror` clean (covers: C5, C6; depends: T3)
- [ ] T8: Verify + Review + Finalize — acceptance: `colcon test` green (or `PRE-EXISTING`); reviewer conclusions pass; `status: delivered` with commits range (covers: S1, S2; depends: T3, T4, T6, T7)
