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

Current phase: **first control code, bench-validation stage.** Vision and the MAVLink link
are validated end-to-end (heartbeats both ways; Pi -> FC command authority proven via
`MAV_CMD_DO_MOTOR_TEST` bench scripts). With explicit user sign-off, the first bounded
GUIDED_NOGPS hover controller (`hover_on_tag.py`, streaming `SET_ATTITUDE_TARGET`) has been
written — it is **not flight-approved**: it still needs the TX mode-switch override
configured/tested and the full props-off bench sequence before any armed run. See "Project
Status" below for exactly what exists today.

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
| Video delivery | MJPEG over a small stdlib HTTP server | No X11 dependency — the drone isn't tethered to a monitor, and `cv2.imshow` over SSH/X11 doesn't make sense once this is flying. **Every tool, including calibration capture, is headless via the MJPEG stream** — nothing needs a display. |
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
- Nothing uses `cv2.imshow` or needs a display; every tool (including calibration
  capture) streams to the browser over MJPEG, so plain SSH is enough.

```bash
# Verify camera hardware
rpicam-hello --list-cameras

# Check the UART is present
ls -l /dev/serial0

# Vision-only validation — no MAVLink involved at all
python3 vision_test.py
python3 vision_test.py --log calibration/run1.csv   # also log every detection to CSV

# MAVLink-only connection check — no camera/vision involved at all
python3 mavlink_test.py

# Hover controller, dry-run (no FC connection, no commands — vision + legend only)
python3 hover_on_tag.py --dry-run
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

The integrated entrypoint (vision + MAVLink telemetry, previously `main.py`) has been removed
for this camera/CV-focused phase — it will be reintroduced once flight-controller integration
resumes. `vision_test.py` stays MAVLink-free permanently as a fast way to sanity-check the
vision stack in isolation.

---

## Testing the MAVLink Link

`mavlink_test.py` is the MAVLink counterpart to `vision_test.py` — **zero camera/vision code
path**, just the serial link to the flight controller. It:

- Connects on `/dev/serial0` @ 921600 baud (override with `--device`/`--baud`)
- Waits for the first heartbeat (fails fast with a wiring/baud/`SERIALx_PROTOCOL` hint if none
  arrives within `--timeout` seconds, default 10s)
- Prints armed state, flight mode, and link health at 1 Hz
- Sends this Pi's own heartbeat back once a second — presence-only, no command authority

This only proves the wire is alive and MAVLink2 framing is correct end-to-end; it does not
arm, change mode, or send any position/attitude target. Use it before trusting any MAVLink
work in `main.py` once that's reintroduced.

---

## Project Status (read this before assuming what exists)

**Implemented:**
- AprilTag detection + pose estimation (`vision/`), producing tag position in the ArduPilot
  FRD body frame (forward/right/down, yaw/pitch/roll).
- Per-tag velocity estimation (`vision/velocity_estimator.py`), smoothed finite-difference on
  the FRD position.
- MJPEG live stream (`streaming/mjpeg_server.py`, used by `vision_test.py`).
- `vision_test.py` — MAVLink-free validation harness with offset/velocity overlay + CSV logging.
- Camera calibration tooling (`calibration/`) — chessboard capture + `cv2.calibrateCamera`.
- Read-only MAVLink link (`mavlink/connection.py`): connects, binds to the autopilot's
  heartbeat (filtering out GCS heartbeats), reports armed state, flight mode, and link
  health. Sends its own heartbeat (presence-only). Exposes `raw_connection` for the few
  scripts that hold command authority.
- `mavlink_test.py` — MAVLink-only connection validation harness, no camera/vision code path.
- `motor_test_on_tag.py` — first command-authority bench script: one `MAV_CMD_DO_MOTOR_TEST`
  spin on tag (re)acquisition. NOTE: motor numbers are `MOTOR_TEST_ORDER_DEFAULT`
  test-sequence positions (Mission Planner's Test A/B/C/D = 1..4, clockwise from
  front-right), NOT ESC output-channel labels — this was confirmed empirically.
- `vision_to_motor_indicator.py` — maps tag-position conditions (far/left/right/close) to
  per-motor spins as a visible indicator that the FC acts on vision directives; includes the
  origin->tag vector + pose-legend verification overlay and a `--dry-run` mode.
- `hover_on_tag.py` — **the first real control loop (bench-validation stage, NOT
  flight-approved).** Streams bounded `SET_ATTITUDE_TARGET` in GUIDED_NOGPS: yaw rate from
  tag bearing, roll from tag skew (square-up), pitch from distance error (default 1.0 m),
  thrust as climb-rate demand around 0.5 from vertical offset. Sends nothing unless it
  observes armed + GUIDED_NOGPS (pilot engages via TX switch); tag lost -> streams neutral
  hover + warning, auto-resumes on re-acquisition. Camera->FC mounting offsets are
  placeholder constants awaiting measured values. Gains/signs are placeholders pending
  props-off bench verification.

**Removed for now (was implemented, deferred):**
- `safety/watchdog.py` — freshness watchdog module (superseded for now by inline staleness
  checks in `hover_on_tag.py`; the standalone module returns when integration matures).
- `main.py` — integrated entrypoint that combined vision with MAVLink telemetry.

Both are recoverable from git history when that stage resumes; see the roadmap below.

**Not implemented — deliberately:**
- Anything that arms, disarms, or changes flight mode. The Pi observes armed state and mode;
  the pilot controls both via the transmitter. This line has NOT moved.
- Position/velocity targets, RC overrides, EKF feeding. Attitude targets
  (`SET_ATTITUDE_TARGET`) are now in scope, but only via `hover_on_tag.py`'s bounded,
  gated loop — see "Control Architecture" and "Safety Rules" below.

If you (Claude) are asked to extend command authority beyond this (arming, mode changes,
new scripts that move the vehicle, relaxing clamps/gates), stop and confirm with the user
first, even if it seems like the obvious next step.

---

## Control Architecture (implemented in `hover_on_tag.py` — bench-validation stage)

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

## SITL — the ONLY way to validate control code (read before bench-debugging)

**You cannot validate the GUIDED_NOGPS control loop on the ground. Do not try.**
ArduCopter's `ModeGuided::angle_control_run()` early-returns into
`make_safe_ground_handling()` whenever `!auto_armed || land_complete` — so while the vehicle is
landed it **discards every attitude target**. `ATTITUDE_TARGET` echoes zero and the motors stay
at ground idle no matter what you send. This is by design, not a bug, and it burned a lot of
bench time before we understood it. Motor RPM and the echo are both dead ends on the ground.

ArduPilot SITL is built at `~/ardupilot` (aarch64, `./waf configure --board sitl && ./waf copter`,
~20 min on the Pi). Run it and validate:

```bash
# terminal 1 — simulated copter, listens on tcp:127.0.0.1:5760
cd /tmp/sitl_run && ~/ardupilot/build/sitl/bin/arducopter --model quad \
    --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm

# terminal 2 — arms, takes off, hands over to GUIDED_NOGPS, tests every axis
python3 sitl_validate.py
```

`sitl_validate.py` imports `hover_on_tag`'s *real* send path (not a copy) and asserts: the FC
ingests `SET_ATTITUDE_TARGET` (echo tracks), +roll drifts right, −pitch drives forward, +yaw
target turns right, thrust >0.5 climbs. It **refuses to run against a serial device** — it arms
and flies, so it is simulator-only.

**Two real bugs it caught, both of which silently broke everything (don't reintroduce):**
1. **`type_mask` must be 0.** ArduCopter accepts only *all three* body-rate-ignore bits set, or
   *all three* clear. The natural-looking choice (ignore roll+pitch rate, supply yaw rate) is an
   illegal mix and the FC **discards the whole message** — `"The body rates are ill-defined" ->
   hold_position(); return`. Supply all three rates as zeros and carry attitude in the quaternion.
2. **The quaternion's yaw is an ABSOLUTE earth-frame heading, not an offset.** Hardcoding `yaw=0`
   commands the drone to *turn and face north*. Yaw must be sent as
   `(FC's current ATTITUDE yaw) + correction`, which means a fresh `ATTITUDE` message is a hard
   prerequisite for sending at all (`NO_HEADING` state gates this).
   A yaw *rate* in `body_yaw_rate` does nothing — the quaternion's yaw always wins.

Bonus gotcha: when polling MAVLink in a loop, **drain the queue** (`while recv_match(blocking=False)`),
don't read one message per iteration — the backlog grows and every reading goes stale, which
produced convincing but completely bogus "the FC isn't responding" results.

---

## Closed-loop tuning in SITL (`sitl_tag_sim.py`) — and what it found

`sitl_tag_sim.py` puts a VIRTUAL AprilTag in the SITL world, synthesises the detection the
real camera would produce from the simulated drone's true pose, and feeds it through
`hover_on_tag.compute_commands()` — the real control law, not a copy. That closes the whole
loop (vision -> control -> FC -> motion -> vision) and is how the gains were tuned.

```bash
python3 sitl_tag_sim.py                                   # default scenario
python3 sitl_tag_sim.py --tag-range 6 --tag-skew -35      # harder start
python3 sitl_tag_sim.py --kd-roll 2.5                     # sweep one gain
```

**Three real design bugs it caught — all of which would have crashed the real drone:**

1. **Pure-P control cannot work here.** Tilt commands ACCELERATION while we control POSITION —
   a double integrator, which a P-only controller can never stabilise. The drone pinned max
   forward tilt for the whole approach and flew straight *through* the tag. Fixed by adding
   velocity damping (`KD_*`) fed from `vision/velocity_estimator.py`. **Never remove the D terms.**
2. **`KD_ROLL` had the wrong sign — it was ANTI-damping.** `v_right` goes *negative* when the
   drone strafes right, so braking needs a POSITIVE coefficient. With it negative, adding
   "damping" pumped energy in and the drone orbited the tag forever. Sign is counterintuitive
   because the velocity is the *tag's* relative to the camera, not the drone's.
3. **"Yaw to centre the tag + roll to null the skew" is structurally unstable.** The two loops
   chase each other (strafe -> changes bearing -> yaws -> changes skew -> strafe) and the drone
   ORBITS. Replaced with a decoupled goal-point controller: compute where the drone should be
   (the point `--distance` out along the tag's normal), drive pitch/roll straight at it, and let
   yaw independently point the nose at the tag.

Also: **the approach must be speed-capped** (`MAX_APPROACH_ERR_M`). The goal point's lateral
offset is only `distance * sin(skew)` — tiny — so without a cap the drone rushes the tag far
faster than it squares up, arrives at a huge viewing angle, and the tag becomes undetectable
(AprilTags cannot be decoded past ~60 deg edge-on).

Converges from both a 4 m/+20 deg and a 6 m/-35 deg start: distance err <0.08 m, lateral
<0.01 m, vertical <0.05 m, skew <3 deg, zero frames lost.

**Hover setpoint is 1.0 m** (`--distance`, and `--focus-m` must match it — the camera's focus
is locked, so a mismatch puts the tag outside the depth of field). The setpoint is not a free
parameter: the goal point's lateral offset is `distance * sin(skew)`, so a LARGER distance
gives the squaring-up loop MORE authority. Moving 0.5 m -> 1.0 m improved steady-state skew
from ~5.8 deg to ~2.8 deg for free. Re-run `sitl_tag_sim.py` if you change it again.

**Gains transfer to the real drone reasonably well** because we command angles + a climb rate,
not motor outputs: `a = g*tan(roll)` is airframe-independent, and `WPNAV_SPEED_UP` is 250 on
both. The real 816 g / ~7:1-thrust airframe is punchier than SITL's default quad, so these
gains land on the CONSERVATIVE side — the safe direction. Drag/wind/camera-latency are not
modelled; treat them as a sane starting point, not final numbers.

---

## Camera Calibration

`vision/apriltag_detector.py` loads `config/camera_intrinsics.npz` if present; otherwise it
falls back to intrinsics estimated from the Camera Module 3 datasheet (not measured) and
prints a warning. The fallback is fine for a first bring-up but **not accurate enough for
distance/pose-based control** — run the calibration flow before that matters:

```bash
python3 calibration/capture_calibration_images.py   # headless: watch http://<pi-ip>:8080/stream, auto-captures
#   IMPORTANT: focus is locked at 1 m to match flight; measure a square and set
#   SQUARE_SIZE_M in calibrate_camera.py before running calibrate
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
4. **Always call `picam2.stop()`** (and, once MAVLink returns, close that connection too) on
   exit — uncleaned camera handles can block future runs and require a reboot.
5. **Autofocus** — Camera Module 3 has PDAF. Continuous AF adds latency/hunting that will
   show up as pose jitter; lock or trigger AF explicitly once that becomes a problem.
6. **MAVLink serial config:** Pi side is `/dev/serial0` @ 921600 baud. The matching FC side
   (`SERIALx_PROTOCOL=2` for MAVLink2, `SERIALx_BAUD` matching) must be set in Mission
   Planner on whichever UART is physically wired to the Pi.

---

## Project Structure

```
Cam_Test/
├── vision_test.py          # MAVLink-free validation harness: pose + velocity + offset overlay
├── mavlink_test.py          # Camera-free validation harness: heartbeat + link health check
├── motor_test_on_tag.py     # Bench: one DO_MOTOR_TEST spin on tag (re)acquisition
├── vision_to_motor_indicator.py  # Bench: tag far/left/right/close -> per-motor spin indicator
├── hover_on_tag.py          # GUIDED_NOGPS hover controller (bench-validation stage)
├── vision/
│   ├── apriltag_detector.py    # Detection + pose estimation, calibration-aware
│   ├── velocity_estimator.py   # Per-tag smoothed velocity from consecutive detections
│   └── frame_transform.py      # Camera frame -> ArduPilot FRD body frame conversion
├── mavlink/
│   └── connection.py           # Read-only heartbeat/telemetry link (no command authority)
├── streaming/
│   └── mjpeg_server.py         # Shared MJPEG-over-HTTP server (used by vision_test.py)
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

`main.py` and `safety/` are still removed for this camera/CV-focused phase — see "Project
Status" above. They'll return (from git history) when flight-controller integration resumes.

Tag in use: **family 36h11, ID 0, 16.8 cm side length** (`assets/apriltag_print.pdf`).

---

## Python Code Style for This Project

- Python 3.11+, type hints on function signatures welcome but not required
- Small, focused modules (`vision/`, and later `mavlink/`, `safety/`) over one large script —
  this project's whole point is that vision, telemetry, and (later) control stay separable and
  independently testable
- Use `argparse` for CLI flags (resolution, port, etc.)
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
- [x] Pi -> FC command authority proven (`MAV_CMD_DO_MOTOR_TEST` bench scripts, props off)
- [ ] Vision pipeline validated against known distances/offsets using `vision_test.py`
- [ ] Camera calibration actually run and `config/camera_intrinsics.npz` committed to use
- [ ] **Verify transmitter mode-switch override out of `GUIDED_NOGPS` in Mission Planner —
  REQUIRED before any armed run of `hover_on_tag.py` (it is also the engage switch)**
- [x] Bounded `GUIDED_NOGPS` + `SET_ATTITUDE_TARGET` control loop written (`hover_on_tag.py`)
- [ ] Verify `GUID_OPTIONS=0` (thrust = climb-rate demand, 0.5 = hold alt) and `GUID_TIMEOUT`
  on the FC before first armed run
- [ ] Measure and fill in the camera->FC mounting offsets in `hover_on_tag.py`
- [x] ArduPilot SITL built and `sitl_validate.py` written (see "SITL" section)
- [x] **Control path validated in SITL** — FC ingests `SET_ATTITUDE_TARGET`; +roll drifts right,
  -pitch drives forward, +yaw target turns right, thrust >0.5 climbs. All sign conventions
  confirmed correct. Two message-format bugs found and fixed this way.
- [ ] ~~Bench-validate props-off~~ — **NOT POSSIBLE**: ArduCopter discards attitude targets while
  landed. Ground testing of the control loop is a dead end; use SITL.
- [x] **Outer-loop gains tuned in SITL closed-loop** (`sitl_tag_sim.py`) — converges from 4 m
  and 6 m starts, both skew directions. Three design bugs found and fixed (see "Closed-loop
  tuning" above): missing D terms, inverted `KD_ROLL`, and the orbit-prone skew/yaw coupling.
- [ ] **AUTOTUNE the inner loop on the real airframe.** Mission Planner's Initial Parameter
  Setup was run (7" props / 6S: `INS_GYRO_FILTER=57`, `MOT_THST_EXPO=0.54`, accel limits,
  `MOT_THST_HOVER=0.20`), and the 6S battery failsafe is configured (`BATT_LOW/CRT_VOLT`
  21.0/19.8, action=Land — RTL is wrong with no GPS). But `ATC_RAT_*` are still STOCK defaults
  aimed at a ~10" copter. Our outer loop rides on this — **no vision tuning fixes a bad inner
  loop.**
- [ ] Manual hover in Stabilize, then AltHold (lets `MOT_HOVER_LEARN` converge the real hover
  throttle), then AUTOTUNE — **in that order, before any GUIDED_NOGPS flight**
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
