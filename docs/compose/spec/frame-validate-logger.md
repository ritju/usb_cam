---
feature: frame-validate-logger
status: delivered
updated: 2026-10-19
branch: feature/frame-validate-logger
commits: e6ca261..48ec1f4
---

# frame-validate-logger

## Report

**What was built** — `UsbCam::read_frame` now gates every DQBUF/read payload through `frame_payload_error()` (index range, `V4L2_BUF_FLAG_ERROR`, `bytesused==0`, `bytesused>length`, `bytesused>max_buffer_bytes`) and drops bad frames with a requeue instead of feeding them to `convert`/`memcpy`. Logging moved to an abstract `set_logger(log_fn_t)` sink so the `usb_cam` library stays rclcpp-free; `UsbCamNode` injects a callback that maps DEBUG/INFO/WARN/ERROR onto `RCLCPP_*` on the node logger. All UsbCam-class `std::cerr`/`std::cout` prints route through `log()`; the default sink remains iostream for standalone/gtests.

**Verification** —
- `colcon build --packages-select usb_cam` — **PASS**
- `ctest -R 'test_usb_cam_utils|test_pixel_formats'` — **PASS** (8 + 5 cases; new: `test_frame_payload_error`, `test_set_logger_routes_messages`)
- `ament_uncrustify` / `ament_cpplint` on `src/usb_cam.cpp`, `include/usb_cam/usb_cam.hpp`, `test/test_usb_cam_utils.cpp` — **PASS**
- `cppcheck` / `flake8` / `usb_cam_node` style — **PRE-EXISTING**
- Reviewer (fresh subagent): D1–D3 PASS, verdict **PASS**, no criticals.

**Journey log** —
1. `CMakeLists.txt:37` forbids linking rclcpp into `usb_cam`. User chose an abstract callback over `rclcpp::Logger` injection — the lib stays ROS-free and the node owns the RCLCPP mapping.
2. Spec T4's original "rg empty" acceptance contradicted D2's required default iostream sink; acceptance wording amended at finalize.
3. Predicate is `frame_payload_error` (returns a reason string) rather than a bool `is_valid_frame_payload` — same semantics, better log messages.
4. `max_buffer_bytes == 0` disables that bound (unconfigured object); documented in the D1 table.
5. Residual (out of scope, pre-existing): USERPTR lookup loop early-returns on match and never validates `buf.m.userptr`; converters still ignore `bytes_used` (noted in `sizeimage-guard`).

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
| `buf.bytesused > m_max_buffer_bytes` (skipped when `m_max_buffer_bytes == 0`) | log WARN, requeue, skip frame |
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

- [x] T1: Spec with D1–D3 — acceptance: this file with gate table, callback contract, scope (covers: S1, S2, S3)
- [x] T2: `log_level_t` / `log_fn_t` / `set_logger` / `log()` in `UsbCam`; default iostream sink — acceptance: unit test captures callback output and asserts level+message; without a logger, no crash (covers: D2)
- [x] T3: Frame gate in `read_frame` (all three IO methods) per D1 table — acceptance: unit-level test of the pure predicate (`frame_payload_error`) covering each failure row + happy path; gated path requeues and returns (covers: D1)
- [x] T4: Switch UsbCam-class iostream to `log()`; node injects RCLCPP mapping — acceptance: `rg 'std::c(err|out)' src/usb_cam.cpp` empty except the `UsbCam::log` default sink; `rg 'std::c(err|out)' include/usb_cam/usb_cam.hpp` only outside UsbCam methods (or in a comment); node sets logger before `configure` (covers: D2, D3; depends: T2)
- [x] T5: Verify + Review + Finalize — acceptance: gtests pass (or `PRE-EXISTING`); reviewer conclusions pass; `status: delivered` + commits range (covers: S1, S2; depends: T3, T4)
