# Autonomous Drone

A 7-inch quadcopter that finds an AprilTag with an onboard camera and holds
station relative to it — a metre back, centred, square to its face. There's no
GPS involved; position comes entirely from the camera.

<p align="center">
  <img src="assets/drone-top.jpg" width="46%" alt="Top view of the quad, showing the Raspberry Pi and wiring">
  <img src="assets/drone-side.jpg" width="46%" alt="Side view, showing the camera and flight controller stack">
</p>

## How it works

A Raspberry Pi handles the vision and the outer control loop. It detects the
tag, converts the tag's pose into the drone's body frame, and works out where
the drone *should* be — the point one metre out along the tag's normal. A PD
controller drives roll and pitch toward that point, yaw independently keeps the
nose on the tag, and thrust holds the tag vertically centred.

The result is streamed to ArduPilot as attitude targets over MAVLink at 20 Hz,
in `GUIDED_NOGPS`. ArduPilot keeps everything that matters for safety:
stabilisation, motor mixing, arming, and its own failsafes. The Pi never arms
the vehicle and never talks to the motors. The pilot engages and disengages the
controller with a switch on the transmitter, which is also the emergency exit.

Losing the tag doesn't cause a guess or a search — the controller falls back to
a level, altitude-holding hover and says so.

## Hardware

|  |  |
|---|---|
| Airframe | 7" quad, 816 g, 6S |
| Flight controller | SpeedyBee F405 V3, ArduCopter 4.6 |
| Companion computer | Raspberry Pi 4 |
| Camera | Camera Module 3 (IMX708) — 2 ms shutter, focus fixed at 1 m |
| Radio | ELRS |
| Link | Pi ↔ FC over UART, MAVLink2 at 921600 |

`GUIDED_NOGPS` isn't in the stock firmware for this board — it's compiled out to
save flash — so the FC runs a custom build from ArduPilot's build server.

## Status

Vision, the MAVLink link, and the control path all work and are validated. The
controller converges in ArduPilot SITL from 4–6 m out, in both skew directions,
settling within a few centimetres without dropping a frame.

It hasn't flown autonomously yet. The airframe is still on stock rate PIDs aimed
at a larger copter, and that inner loop needs AUTOTUNE before the outer loop can
be trusted in the air.

## Things that cost a lot of time

- **ArduCopter silently drops `SET_ATTITUDE_TARGET` if the type_mask mixes
  ignored and supplied body rates.** No error, no NAK — the message just
  disappears. Every command was being thrown away.
- **The attitude quaternion's yaw is an absolute heading, not an offset.**
  Sending yaw = 0 politely asks the drone to turn and face north.
- **None of this can be tested on a bench.** ArduCopter ignores attitude targets
  while it believes it's landed, and clamping the drone down is worse — the
  controllers wind up against the restraint and drive the motors to full. SITL
  is the only honest way to check it before flying.
- **Tilt commands acceleration while you're controlling position** — a double
  integrator, which no amount of proportional gain will stabilise. It flew
  straight through the tag until velocity damping went in.
- **Detection was failing in flight because the shutter was at 20 ms.** The
  props vibrate around 65–100 Hz, so every frame smeared across one or two full
  cycles. A 2 ms shutter and more light fixed it; the sensor has plenty of gain
  to spare, and tag detection tolerates noise far better than blur.

## Running it

Nothing needs a display — every tool streams an annotated view to
`http://<pi-ip>:8080/stream`.

```bash
python3 vision_test.py             # camera and tag detection, no MAVLink
python3 mavlink_test.py            # link check, no camera
python3 camera_tune.py             # sweep exposure, measure detection and jitter
python3 hover_on_tag.py --dry-run  # controller: computes and displays, sends nothing
```

Against the simulator:

```bash
python3 sitl_validate.py   # confirms the FC ingests targets, and every sign convention
python3 sitl_tag_sim.py    # closed loop against a virtual tag
```

Camera calibration, if you're rebuilding this: print `assets/calibration_chessboard.png`,
measure a square, set `SQUARE_SIZE_M`, then run the two scripts in `calibration/`.

## Layout

```
vision/       detection, pose, velocity, camera setup, spike filtering
mavlink/      link to the flight controller
streaming/    MJPEG server
calibration/  camera calibration
```

## License

MIT — see [LICENSE](LICENSE).
