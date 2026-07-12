# Cam_Test — CLAUDE.md

Project memory for Claude Code sessions on this Raspberry Pi + ArduPilot autonomous drone project.

---

## Project Goal

A 7-inch FPV drone should detect an AprilTag with an onboard camera, estimate the tag's
position relative to the drone, and autonomously **keep the tag centered in frame and at a
fixed distance** — moving with the tag as it moves. This is a "follow me" / visual-servoing
behavior, built up in conservative stages. It is intended for **indoor use with no GPS**.

The Raspberry Pi is a **companion computer**, not the flight controller. It observes the
world (camera) and, in later stages, sends attitude targets to ArduPilot over MAVLink.
ArduPilot (running on the SpeedyBee F405 V3) remains solely responsible for stabilization,
motor mixing, arming logic, and failsafes. **The Pi never talks to motors directly and never
arms the vehicle.**

Current phase: vision + telemetry monitoring, integrated but not yet closing the control
loop. See "Project Status" below for exactly what exists today.

---

## Hardware

| Component | Detail |
|-----------|--------|
| Airframe | 7-inch FPV drone |
| Flight controller | SpeedyBee F405 V3 (running ArduPilot / ArduCopter) |
| Companion computer | Raspberry Pi 4 Model B |
| Camera | Camera Module 3 (Sony IMX708), mounted upside-down (frames are flipped 180° in software) |
| Radio | RadioMaster Pocket (ELRS) + ELRS receiver on the FC |
| Configuration tool | Mission Planner (laptop, used only to configure/monitor ArduPilot — never in the runtime data path) |
| GPS / optical flow / rangefinder | **None assumed.** Indoor flight, no position source. |
| Companion link | Pi GPIO UART ↔ FC UART, wired directly (no telemetry radio) |

### Camera Module 3 — Sensor Modes (IMX708)

| Mode | Resolution | Max FPS | Use case |
|------|-----------|---------|----------|
| Full | 4608 × 2592 | 14 fps | High-res stills |
| Binned 2×2 | 2304 × 1296 | 56 fps | General video |
| High-speed | 1536 × 864 | 120 fps | Fast motion |
| HDR | 2304 × 1296 | 30 fps | HDR video |

Operating point in use: **1280 × 720 @ 30 fps.** Lower resolution than the original
stream-only demo, traded for detection/pose-estimation headroom and lower MJPEG bandwidth —
revisit if tag detection range needs to improve.

**Note on rolling shutter:** the IMX708 is a rolling-shutter sensor. At higher drone
angular velocity this will skew tag corners and degrade pose estimates. Not a problem at
today's slow/bench-test speeds; worth revisiting if the airframe moves fast during tracking.

---

## Software Stack

| Layer | Choice | Why |
|-------|--------|-----|
| OS | Raspberry Pi OS (Bookworm 64-bit) | Official, libcamera support |
| Camera driver | `libcamera` / `rpicam-apps` | Replaces deprecated raspistill/raspivid |
| Python camera API | `picamera2` | Captures frames as numpy arrays, directly usable by OpenCV |
| Tag detection | `cv2.aruco` with `DICT_APRILTAG_36H11` | True AprilTag family, no separate `apriltag` library dependency — OpenCV's ArUco module detects it directly |
| Pose estimation | `cv2.aruco.estimatePoseSingleMarkers` | Needs calibrated camera intrinsics — see "Camera Calibration" |
| MAVLink | `pymavlink` | **Not DroneKit** — DroneKit is unmaintained and has known compatibility gaps with current ArduPilot/MAVLink2; pymavlink is what ArduPilot's own tooling and most companion-computer projects use today |
| Video delivery | MJPEG over a small stdlib HTTP server | No X11 dependency — the drone isn't tethered to a monitor, and `cv2.imshow` over SSH/X11 doesn't make sense once this is flying. X11 (`ssh -X` + `cv2.imshow`) is still fine for one-off bench tools like camera calibration. |
| Numpy | `numpy` | Frame buffer + pose math |

---

## Development Environment

- Editing happens via **VS Code Remote-SSH** into the Pi; the laptop is only the editor/terminal.
- All code **runs on the Pi** — it needs the camera, the GPIO UART, and (eventually) MAVLink.
- Mission Planner runs on the laptop purely to configure/monitor ArduPilot parameters and
  watch telemetry during bench tests. **It is not part of the runtime software** — the Pi
  talks to the FC directly over its own MAVLink connection, independent of whatever Mission
  Planner is doing.
- View the live annotated stream at `http://<pi-ip>:8080/stream` from any browser on the LAN.
- `ssh -X` + `cv2.imshow()` is only used for one-off interactive tools (e.g. calibration
  capture) run on the bench, never for the main drone-facing application.

```bash
# Verify camera hardware
rpicam-hello --list-cameras

# Check the UART is present
ls -l /dev/serial0

# Vision-only validation — no MAVLink involved at all (current recommended
# way to test the Pi/camera side before any flight-controller integration)
python3 vision_test.py
python3 vision_test.py --log calibration/run1.csv   # also log every detection to CSV

# Run the main app (vision + optional MAVLink telemetry monitoring)
python3 main.py

# Run vision-only, without attempting a flight-controller link
python3 main.py --no-mavlink
```

---

## Testing the Vision Pipeline (current phase)

`vision_test.py` is a standalone harness with **zero MAVLink code path** — it exists
specifically so the camera/detection/pose/velocity side can be validated on its own before
any flight-controller integration is trusted. It:

- Runs detection + pose estimation and prints position, distance, and orientation at 10 Hz
- Estimates per-tag velocity (`vision/velocity_estimator.py`) — finite difference on FRD
  position, smoothed, with gap-based reset so tag re-acquisition doesn't spike velocity
- Overlays a center crosshair and an offset line/readout in pixels — this is the same signal
  a future follow-controller would act on, so it's worth eyeballing that it points the right
  way before anything acts on it
- Optionally logs every detection to CSV (`--log path.csv`) for offline accuracy analysis

Suggested validation pass:
1. Place the tag at known distances (0.5m, 1m, 2m...) and compare against `distance_m`.
2. Move the tag off-center and confirm the offset direction matches which way it moved.
3. Wave the tag by hand and watch `v_right`/`v_fwd`/`v_down` track the motion, then settle
   back toward zero when it stops.
4. If numbers look off, suspect uncalibrated intrinsics first — see "Camera Calibration".

`main.py` is the integrated entrypoint (vision + optional read-only MAVLink telemetry) and
will be where control logic eventually lands; `vision_test.py` stays MAVLink-free permanently
as a fast way to sanity-check the vision stack in isolation.

---

## Project Status (read this before assuming what exists)

**Implemented:**
- AprilTag detection + pose estimation (`vision/`), producing tag position in the ArduPilot
  FRD body frame (forward/right/down, yaw/pitch/roll).
- Per-tag velocity estimation (`vision/velocity_estimator.py`), smoothed finite-difference on
  the FRD position.
- MJPEG live stream (`streaming/mjpeg_server.py`, shared by `main.py` and `vision_test.py`).
- `vision_test.py` — MAVLink-free validation harness with offset/velocity overlay + CSV logging.
- Read-only MAVLink link (`mavlink/connection.py`): connects, waits for heartbeat, reports
  armed state, flight mode, and link health. Sends its own heartbeat (presence-only).
- A freshness watchdog (`safety/watchdog.py`) tracking camera/detection/MAVLink staleness —
  currently observation/logging only.
- Camera calibration tooling (`calibration/`) — chessboard capture + `cv2.calibrateCamera`.

**Not implemented — deliberately:**
- Anything that arms, disarms, changes flight mode, or sends an attitude/position/velocity
  target. No `SET_ATTITUDE_TARGET`, no `COMMAND_LONG` mode changes, nothing that can move the
  vehicle. This is a hard line — see "Control Architecture" and "Safety Rules" below.

If you (Claude) are asked to "wire up control" or "make it follow the tag," that crosses this
line — stop and confirm with the user first, even if it seems like the obvious next step.

---

## Control Architecture (target design — not yet built)

**Why not plain `GUIDED` mode:** ArduCopter's `GUIDED` mode accepts position/velocity targets,
which require the EKF to have a position source (GPS, optical flow, or vision-based aiding).
None of those exist here indoors, so plain `GUIDED` is not usable as-is.

**The mode to use is `GUIDED_NOGPS`.** It accepts MAVLink `SET_ATTITUDE_TARGET` messages
(roll/pitch/yaw rate + thrust) and nothing else — it never touches the EKF's position
estimate, so it doesn't need GPS, optical flow, or a rangefinder. This is the standard
approach for indoor companion-computer visual servoing on ArduPilot.

Intended control mapping, once built:
- Horizontal tag offset in frame → commanded roll angle (move the drone sideways to recenter)
- Vertical tag offset / apparent tag size → commanded pitch angle and/or throttle (close or
  open distance, hold altitude)
- Everything else (yaw, actual stabilization, motor mixing) stays with ArduPilot

**On tag velocity (`vision/velocity_estimator.py`):** it reports the tag's velocity relative
to the camera, not the tag's velocity in the room and not the drone's own ground velocity —
a single monocular camera can't distinguish those. That's fine for a follow-controller, which
should null relative drift regardless of whether the drone or the tag caused it. It only
becomes a real design question if something later needs the tag's absolute motion independent
of the drone's own maneuvering — that would require subtracting ArduPilot's own IMU/attitude-
based ego-motion estimate from the vision-relative measurement. Not needed yet.

This will be built as a small, bounded correction loop (clamped angles, rate-limited, with
the watchdog gating every command) — not a general-purpose autopilot. It needs explicit
user approval before implementation, and props-off bench validation before any prop-on test.

---

## Safety Rules (hard constraints — do not relax these without the user explicitly asking)

1. **The Pi never arms or disarms the vehicle.** Arming is a deliberate pilot action via the
   transmitter. The Pi may eventually *read* armed state, never set it.
2. **The Pi never sends a command that directly actuates motors.** All motion goes through
   ArduPilot's own control loops via MAVLink — never bypassed.
3. **A transmitter mode switch must be able to pull the FC out of `GUIDED_NOGPS` back to a
   manual mode (e.g. Stabilize) at any time.** This is the actual emergency stop for
   autonomous behavior and is completely independent of GPS — it's a normal FC flight-mode-
   channel mapping, configured in Mission Planner's Flight Modes tab. **Confirm this is
   configured and tested before any control code is written or run**, not after.
4. **Loss of health (camera stale, detection stale, or MAVLink link stale) must stop
   autonomous movement**, not attempt to compensate or guess. ArduPilot's own failsafes
   (RC loss, battery, EKF) are the primary safety net; the Pi-side watchdog is a secondary
   check, never the only one.
5. **Assume propellers are removed during all early development and bench testing.** Only
   move to props-on testing with explicit user confirmation, in a cleared area.
6. **Always ask before implementing anything that could physically move the drone** — this
   includes anything sending `SET_ATTITUDE_TARGET`, velocity/position targets, RC overrides,
   or mode-change commands. Vision-only and telemetry-only changes don't need to ask first;
   anything with command authority over the vehicle does.

---

## Camera Calibration

`vision/apriltag_detector.py` loads `config/camera_intrinsics.npz` if present; otherwise it
falls back to intrinsics estimated from the Camera Module 3 datasheet (not measured) and
prints a warning. The fallback is fine for a first bring-up but **not accurate enough for
distance/pose-based control** — run the calibration flow before that matters:

```bash
python3 calibration/capture_calibration_images.py   # needs a monitor/X11 for cv2.imshow
python3 calibration/calibrate_camera.py
```

This produces `config/camera_intrinsics.npz`, picked up automatically on the next run.

---

## Key Constraints and Rules

1. **libcamera is the only supported camera stack** — never the old `picamera` (V1 API),
   `raspistill`, or `raspivid`. Not compatible with Camera Module 3.
2. **Root is not needed** for camera or UART access — run as the normal user. If permissions
   fail: `groups $USER | grep video` and `groups $USER | grep dialout` (serial port group
   varies by OS image).
3. **Don't use `time.sleep()` in the capture loop** — `picamera2` manages frame timing
   internally; a blocking sleep just adds latency.
4. **Always call `picam2.stop()`** (and close the MAVLink connection) on exit — uncleaned
   camera handles can block future runs and require a reboot.
5. **Autofocus** — Camera Module 3 has PDAF. Continuous AF adds latency/hunting that will
   show up as pose jitter; lock or trigger AF explicitly once that becomes a problem.
6. **MAVLink serial config:** Pi side is `/dev/serial0` @ 921600 baud. The matching FC side
   (`SERIALx_PROTOCOL=2` for MAVLink2, `SERIALx_BAUD` matching) must be set in Mission
   Planner on whichever UART is physically wired to the Pi.

---

## Project Structure

```
Cam_Test/
├── main.py                 # Integrated entrypoint: vision + optional MAVLink telemetry
├── vision_test.py          # MAVLink-free validation harness: pose + velocity + offset overlay
├── vision/
│   ├── apriltag_detector.py    # Detection + pose estimation, calibration-aware
│   ├── velocity_estimator.py   # Per-tag smoothed velocity from consecutive detections
│   └── frame_transform.py      # Camera frame -> ArduPilot FRD body frame conversion
├── streaming/
│   └── mjpeg_server.py         # Shared MJPEG-over-HTTP server (used by main.py and vision_test.py)
├── mavlink/
│   └── connection.py           # Read-only heartbeat/telemetry link (no command authority)
├── safety/
│   └── watchdog.py             # Freshness monitor for camera/detection/mavlink
├── calibration/
│   ├── capture_calibration_images.py
│   ├── calibrate_camera.py
│   └── images/                 # Captured chessboard frames (gitignore candidate)
├── config/
│   └── camera_intrinsics.npz   # Generated by calibration/calibrate_camera.py
├── tools/
│   └── generate_tag.py         # Prints the AprilTag used for testing
├── assets/
│   ├── apriltag_36h11_id0.png
│   └── apriltag_print.pdf
└── requirements.txt
```

Tag in use: **family 36h11, ID 0, 16.8 cm side length** (`assets/apriltag_print.pdf`).

---

## Python Code Style for This Project

- Python 3.11+, type hints on function signatures welcome but not required
- Small, focused modules (`vision/`, `mavlink/`, `safety/`) over one large script — this
  project's whole point is that vision, telemetry, and (later) control stay separable and
  independently testable
- Use `argparse` for CLI flags (resolution, port, MAVLink device/baud, `--no-mavlink`)
- No logging frameworks yet — `print()` is fine for debug output at this stage
- Format with `black` if available; otherwise PEP 8 spacing

---

## Future Work / Roadmap

Roughly in order, each stage gated on the previous one working and on explicit user sign-off
before anything with command authority is added:

- [x] AprilTag detection + pose estimation in the ArduPilot body frame
- [x] Per-tag velocity estimation
- [x] MJPEG live view with overlay
- [x] MAVLink-free vision validation harness (`vision_test.py`)
- [x] Read-only MAVLink telemetry (heartbeat, armed state, mode, link health)
- [x] Camera calibration tooling
- [ ] Vision pipeline validated against known distances/offsets using `vision_test.py`
- [ ] Camera calibration actually run and `config/camera_intrinsics.npz` committed to use
- [ ] Verify transmitter mode-switch override out of `GUIDED_NOGPS` in Mission Planner
- [ ] Bounded `GUIDED_NOGPS` + `SET_ATTITUDE_TARGET` control loop (props off first)
- [ ] Tune correction gains, rate limits, and watchdog-triggered cutoff, props off
- [ ] First props-on hover test, tethered/caged, tag stationary
- [ ] Tag motion following
- [ ] Longer-term: optical flow or a rangefinder if drift/altitude hold become limiting

---

## References

- [ArduCopter GUIDED_NOGPS mode](https://ardupilot.org/copter/docs/ac2_guidedmode.html)
- [pymavlink](https://github.com/ArduPilot/pymavlink)
- [picamera2 Manual (PDF)](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf)
- [Camera Module 3 Product Brief](https://datasheets.raspberrypi.com/camera/camera-module-3-product-brief.pdf)
- [MAVLink common message set](https://mavlink.io/en/messages/common.html)
