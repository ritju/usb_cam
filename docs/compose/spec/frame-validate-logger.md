---
feature: frame-validate-logger
status: designed
updated: 2026-10-19
branch: feature/frame-validate-logger
commits: 
---

# frame-validate-logger

## Report

## [S1] Problem

`UsbCam::read_frame()` trusts driver-provided `v4l2_buffer` fields and feeds `buf.bytesused` straight into `process_image`. A corrupt or short DQBUF payload (`bytesused == 0`, `bytesused > buf.length`, `V4L2_BUF_FLAG_ERROR`, out-of-range `buf.index`) currently reaches the converters — MJPEG does `av_new_packet(bytesused)` + `memcpy`, the raw path copies into the ROS message — with no gate. That is a crash / heap-corruption path and is exactly the class of garbage that follows the corrupt-UVC / low-memory episodes documented in `sizeimage-guard`.

Separately, `UsbCam` logs with `std::cerr` / `std::cout` (20 sites in `src/usb_cam.cpp`, 7 in `UsbCam::set_pixel_format` in `include/usb_cam/usb_cam.hpp`). Those messages never reach the ROS log stream, so field debugging via `ros2 topic` / log aggregation misses camera-driver errors entirely.

## [S2] Design

### D1 Frame payload gate

After every successful DQBUF / read, validate before `process_image`:

| Check | Failure behavior |
|---|---|
| `buf.index < m_number_of_buffers` (MMAP/USERPTR) | log WARN, requeue if applicable, skip frame |
| `buf.bytesused == 0` | log WARN, requeue, skip frame |
| `buf.bytesused > buf.length` | log WARN, requeue, skip frame |
| `buf.bytesused > m_max_buffer_bytes` | log WARN, requeue, skip frame |
| `buf.flags & V4L2_BUF_FLAG_ERROR` | log WARN, requeue, skip frame |

Skip = return without `process_image`. Never throw from a bad frame (one corrupt buffer must not kill the stream). Always requeue the buffer so the queue keeps draining.

`IO_METHOD_READ`: validate the `read()` return `len` the same way (`len <= 0` or `len > m_buffers[0].length` → skip). No requeue.

### D2 Abstract log sink (lib stays rclcpp-free)

`CMakeLists.txt:37` keeps `usb_cam` free of rclcpp. Use a callback, not `rclcpp::Logger`:

```cpp
enum class log_level_t { DEBUG, INFO, WARN, ERROR };
using log_fn_t = std::function<void(log_level_t, const std::string &)>;
void UsbCam::set_logger(log_fn_t fn);   // stored; empty = default iostream sink
```

Internal helper `log(level, msg)` routes every message. Default sink keeps today's iostream behavior so the lib remains usable standalone and in gtests; `UsbCamNode` injects a callback that maps levels to `RCLCPP_DEBUG/INFO/WARN/ERROR` on the node logger.

### D3 Scope of the iostream switch

Only `UsbCam` **class** methods:

- All 20 `std::cerr` sites in `src/usb_cam.cpp` member functions
- `UsbCam::set_pixel_format` listings in `include/usb_cam/usb_cam.hpp` (`std::cout` / `std::cerr`)

Out of this switch: `usb_cam::utils::available_devices` (`utils.hpp`) and `formats/*` converters.

## [S3] Out of Scope

- `utils.hpp` / `formats/*` iostream prints (free functions / converters).
- Fixing the USERPTR `read_frame` early-`return`-on-match bug (pre-existing; not touched).
- Making converters honor `bytes_used` (residual noted in `sizeimage-guard`; separate).
- rclcpp dependency for `usb_cam` lib (explicitly rejected).
- `set_pixel_format` output shape / verbosity redesign — only the sink changes.

## Tasks

- [ ] T1: Spec with D1–D3 — acceptance: this file with gate table, callback contract, scope (covers: S1, S2, S3)
- [ ] T2: `log_level_t` / `log_fn_t` / `set_logger` / `log()` in `UsbCam`; default iostream sink — acceptance: unit test captures callback output and asserts level+message; without a logger, no crash (covers: D2)
- [ ] T3: Frame gate in `read_frame` (all three IO methods) per D1 table — acceptance: unit-level test of the pure predicate (`is_valid_frame_payload`) covering each failure row + happy path; gated path requeues and returns (covers: D1)
- [ ] T4: Switch UsbCam-class iostream to `log()`; node injects RCLCPP mapping — acceptance: `rg 'std::c(err|out)' src/usb_cam.cpp` empty; `rg 'std::c(err|out)' include/usb_cam/usb_cam.hpp` only outside UsbCam methods; node constructs/sets logger before `configure` (covers: D2, D3; depends: T2)
- [ ] T5: Verify + Review + Finalize — acceptance: gtests pass (or `PRE-EXISTING`); reviewer conclusions pass; `status: delivered` + commits range (covers: S1, S2; depends: T3, T4)
