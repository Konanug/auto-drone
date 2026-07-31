# System architecture

How a camera frame becomes a motor command, and which processor is responsible for what.

## Responsibility split

The design rule is that the Raspberry Pi is an *advisor* and the flight controller is the
*authority*. The Pi decides where the drone should go; ArduPilot decides whether and how
the vehicle gets there, and retains every safety function.

| Raspberry Pi 4                          | SpeedyBee F405 V3 (ArduPilot)          |
| --------------------------------------- | -------------------------------------- |
| Camera configuration and capture        | Attitude stabilisation (`ATC_RAT_*`)   |
| AprilTag detection                      | Motor mixing                           |
| Monocular pose estimation               | Arming and disarming                   |
| Camera → body-frame transform           | RC, battery, and EKF failsafes         |
| Goal-point geometry and PD control      | `GUID_TIMEOUT` levelling               |
| MAVLink command generation              | Final authority over the motors        |

The Pi never arms the vehicle, never changes flight mode, and never addresses the motors.
The one exception is deliberate and bench-only: `motor_test_on_tag.py` and
`vision_to_motor_indicator.py` issue `MAV_CMD_DO_MOTOR_TEST`, which is how the Pi's command
authority over the flight controller was first proven with propellers removed.

## Pipeline

```
Camera Module 3  ──►  1280×720 BGR frame, 2 ms shutter, hardware 180° rotation
      │
      ▼
AprilTagDetector  ──►  detect at 0.5 scale, refine corners at full resolution
      │
      ▼
estimatePoseSingleMarkers  ──►  rvec, tvec in the OpenCV camera frame
      │
      ▼
pose_to_ardupilot  ──►  (fwd, right, down, yaw, pitch, roll) in FRD body frame
      │
      ▼
PoseGate  ──►  drop single-frame spikes implying > 4 m/s of tag motion
      │
      ▼
VelocityEstimator  ──►  smoothed relative velocity, 48 ms time constant
      │
      ▼
compute_commands  ──►  goal point, then PD → roll, pitch, yaw correction, thrust
      │
      ▼
slew limiting  ──►  0.5°/tick and 0.005 thrust/tick at 20 Hz
      │
      ▼
SET_ATTITUDE_TARGET  ──►  MAVLink 2 over UART at 921600 baud, 20 Hz
      │
      ▼
ArduPilot GUIDED_NOGPS  ──►  attitude controller → motor mixer → ESCs
```

The loop closes physically: the vehicle moves, and the camera observes the result on the
next frame.

## Rates and timing

| Stage                      | Rate      | Note                                                       |
| -------------------------- | --------- | ---------------------------------------------------------- |
| Sensor                     | 60 fps    | `queue=False`, so each capture returns a freshly exposed frame |
| Vision loop                | ~30 fps   | CPU-bound on the Pi; the 60 fps sensor buys freshness, not loop rate |
| Detection cost             | ~17 ms    | At `--detect-scale 0.5`; full resolution costs ~71 ms and starves the loop to ~8 fps |
| Command stream             | 20 Hz     | `SET_ATTITUDE_TARGET` |
| Flight-controller telemetry | 10 Hz    | `ATTITUDE` and `ATTITUDE_TARGET`, requested via `MAV_CMD_SET_MESSAGE_INTERVAL` |
| Debug MJPEG stream         | ~12 fps   | JPEG encoding costs ~13 ms of a 33 ms budget, so it is throttled below the detection rate |

## State machine

`hover_on_tag.py` sends nothing unless every precondition holds.

| State        | Condition                                                       | Behaviour                          |
| ------------ | --------------------------------------------------------------- | ---------------------------------- |
| `WAITING`    | Not armed, or not in `GUIDED_NOGPS`, or the link is stale        | Sends nothing                      |
| `NO_HEADING` | No `ATTITUDE` message within 0.5 s                              | Sends nothing                      |
| `ACTIVE`     | Engaged, heading fresh, tag seen within 0.5 s                   | Streams computed attitude targets  |
| `TAG_LOST`   | Engaged and heading fresh, but the tag is stale                 | Streams a level, altitude-holding hover |

`NO_HEADING` matters more than it looks. The yaw carried in the attitude quaternion is an
**absolute earth-frame heading**, not an offset, so the command is built as
`current FC heading + correction`. Without a fresh heading there is nothing valid to add
the correction to, and sending a stale or zero yaw would command the vehicle to turn and
face north. So the loop sends nothing and lets ArduPilot's own `GUID_TIMEOUT` level the
vehicle. It never guesses a heading.

## Monitoring

Two health signals run alongside the controller:

- **Inner-loop tracking.** The flight controller reports both what it is targeting
  (`ATTITUDE_TARGET`) and what its IMU achieved (`ATTITUDE`). The rolling mean gap between
  them is the inner loop's tracking error. Above 3° sustained, the display and terminal
  warn that ArduPilot is not holding the attitude it is being told to hold — which no
  amount of outer-loop tuning can fix.
- **Pose gating.** Rejected detections are counted and drawn in red on the stream, so the
  rate of vibration-induced corruption is visible during a flight rather than inferred
  afterwards.
