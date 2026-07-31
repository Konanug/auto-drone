# Hardware and wiring

## Components

| Component            | Part                                                        |
| -------------------- | ----------------------------------------------------------- |
| Airframe             | Custom 7-inch quadcopter, ~816 g, 6S                        |
| Flight controller    | SpeedyBee F405 V3, running ArduCopter 4.6                   |
| Companion computer   | Raspberry Pi 4                                              |
| Camera               | Raspberry Pi Camera Module 3 (Sony IMX708), mounted inverted |
| Radio                | ELRS receiver, ELRS transmitter                             |
| GPS and compass      | HGLRC M100 Pro GPS with QMC5883L compass                    |
| Companion link       | Pi GPIO UART ↔ flight-controller UART, wired directly       |
| Custom parts         | Raspberry Pi mount, camera mount, GPS mount (SolidWorks)    |

The GPS is fitted but is **not** part of the tag-relative control path. All positioning
relative to the tag comes from the camera. The target environment is indoor and GPS-denied.

<!-- Add full system wiring diagram here: assets/diagrams/system-wiring.svg -->
<!-- Add labelled component photograph here: assets/drone/components-labelled.jpg -->
<!-- Add power distribution diagram here: assets/diagrams/power-distribution.svg -->

## Companion link

| Setting        | Value                                          |
| -------------- | ---------------------------------------------- |
| Pi device      | `/dev/serial0`                                 |
| Baud           | 921600                                         |
| Protocol       | MAVLink 2                                      |
| FC side        | `SERIALx_PROTOCOL = 2`, `SERIALx_BAUD` matched |

The UART is wired directly between the Pi and the flight controller — there is no telemetry
radio in the loop. Mission Planner is used on a laptop to configure and monitor ArduPilot,
but it is **not in the runtime data path**; the Pi holds its own independent MAVLink
connection.

One connection detail worth knowing: ArduPilot relays heartbeats between all connected
MAVLink channels, so a ground station plugged in over USB will put its own heartbeats on
the Pi's link. `pymavlink`'s `wait_heartbeat()` would bind to whichever arrives first.
[`mavlink/connection.py`](../mavlink/connection.py) therefore skips heartbeats whose
autopilot type is `MAV_AUTOPILOT_INVALID` and binds only to an actual autopilot.

Neither the camera nor the UART needs root. If access fails, check that the user is in the
`video` and `dialout` groups.

## Custom firmware

`GUIDED_NOGPS` is **compiled out of the stock ArduPilot firmware for this board** to save
flash. Without it the mode is simply unavailable and the whole approach is impossible, so
the flight controller runs a custom build produced through ArduPilot's build server with
that mode included.

The mode is the right one for this application because it accepts `SET_ATTITUDE_TARGET` and
nothing else. It never touches the EKF's position estimate, so it needs no GPS, optical
flow, or rangefinder. Plain `GUIDED` accepts position and velocity targets, which require
the EKF to have a position source — none of which exist indoors here.

## Flight-controller parameters

Parameters the outer loop depends on, or that were set during initial configuration:

| Parameter                        | Value       | Why it matters                                              |
| -------------------------------- | ----------- | ----------------------------------------------------------- |
| `GUID_OPTIONS`                   | 0           | Thrust means climb rate; 0.5 holds altitude. The controller assumes this |
| `WPNAV_SPEED_UP` / `_DN`         | 250 / 150   | Sets the thrust-to-climb-rate scale the thrust gain couples to |
| `GUID_TIMEOUT`                   | default     | Levels the vehicle if the command stream simply stops       |
| `ANGLE_MAX`                      | 3000 (30°)  | The controller commands at most 5°, well inside this        |
| `ATC_SLEW_YAW`                   | 6000 (60°/s) | How fast yaw converges on a new heading target             |
| `INS_GYRO_FILTER`                | 57          | From Mission Planner's initial setup for 7" props on 6S     |
| `MOT_THST_EXPO`                  | 0.54        | Same                                                        |
| `MOT_THST_HOVER`                 | 0.20        | Reflects the roughly 7:1 thrust-to-weight ratio             |
| `BATT_LOW_VOLT` / `BATT_CRT_VOLT` | 21.0 / 19.8 | 6S failsafe thresholds                                     |
| Battery failsafe action          | Land        | **Not RTL** — return-to-launch is meaningless with no GPS   |

`ATC_RAT_*` are still at stock defaults aimed at a larger copter. AUTOTUNE on this airframe
is outstanding, and the outer loop rides directly on the inner loop — no vision tuning
compensates for a badly tuned inner loop. `hover_on_tag.py` measures the gap between
commanded and achieved attitude in flight and warns when it exceeds 3° sustained.

## Bring-up checks

```bash
rpicam-hello --list-cameras     # camera detected
ls -l /dev/serial0              # UART present
python3 vision_test.py          # vision only
python3 mavlink_test.py         # link only
```

Order matters for first flights: manual hover in Stabilize, then AltHold so
`MOT_HOVER_LEARN` converges the real hover throttle, then AUTOTUNE — all before any
`GUIDED_NOGPS` flight.
