# Simulation and testing

## Why the control loop cannot be tested on the ground

This is the single most useful thing learned in this project, and it cost a lot of bench
time before it was understood.

ArduCopter's `ModeGuided::angle_control_run()` early-returns into
`make_safe_ground_handling()` whenever `!auto_armed || land_complete`. While the vehicle is
on the ground it **discards every attitude target it receives**. `ATTITUDE_TARGET` echoes
zero, motors stay at ground idle, and nothing you send has any effect. This is by design,
not a fault.

So motor RPM and the target echo are both dead ends on the ground. There is no props-off
bench test that validates the outer loop. **SITL is the only way.**

## Setup

ArduPilot SITL is built from source (`./waf configure --board sitl && ./waf copter`).

```bash
# Terminal 1 — simulated copter, listening on tcp:127.0.0.1:5760
cd /tmp/sitl_run && ~/ardupilot/build/sitl/bin/arducopter --model quad \
    --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm

# Terminal 2 — pick a harness
python3 sitl_validate.py
python3 sitl_tag_sim.py
```

Both harnesses **refuse to run against a serial device**. They arm and fly, so they check
that the endpoint is `tcp:` or `udp:` and exit otherwise.

## `sitl_validate.py` — is the control path correct?

Open-loop. It arms, takes off, hands over to `GUIDED_NOGPS`, and commands one axis at a
time while measuring what the vehicle actually does. Critically, it imports and calls
`hover_on_tag.send_attitude_target()` — the **real** send path, type mask, and quaternion
construction, not a reimplementation that could drift out of sync with the shipping code.

| Check                                   | Expectation                        |
| --------------------------------------- | ---------------------------------- |
| FC ingests `SET_ATTITUDE_TARGET`        | The echoed target tracks what we sent |
| `+roll`                                 | Drifts right                       |
| `−pitch`                                | Drives forward                     |
| `+yaw` target                           | Turns right                        |
| `thrust > 0.5`                          | Climbs                             |

A failed sign here means a gain in `hover_on_tag.py` has the wrong sign and the drone would
fly the wrong way — which on a real vehicle means straight into whatever it was supposed to
approach.

### Two bugs it caught

1. **`type_mask` must be 0.** ArduCopter accepts all three body-rate-ignore bits set or all
   three clear. The obvious-looking mix — ignore roll and pitch rate, supply yaw rate — is
   illegal and the flight controller silently discards the whole message. Every command was
   being thrown away.
2. **The quaternion's yaw is absolute.** Hardcoding `yaw = 0` commands the vehicle to turn
   and face north, not to hold heading. Yaw must be sent as
   `current ATTITUDE yaw + correction`.

Both were invisible on the bench and obvious within one SITL run.

## `sitl_tag_sim.py` — does the loop converge?

Closed-loop. It places a virtual AprilTag in the simulated world, synthesises the detection
the real camera would produce from the drone's true pose, and feeds it through
`hover_on_tag.compute_commands()` — again the real control law, not a copy. That closes the
entire loop: vision → control → flight controller → motion → vision.

The synthetic camera is deliberately pessimistic. It returns no detection when the tag is
behind the drone, outside the 66° horizontal field of view, or past 60° of viewing angle,
because a real AprilTag cannot be decoded edge-on. Without that last constraint the
simulator "sees" tags at 100° of skew and flatters the controller with information the
camera could never supply. It also exercises the `TAG_LOST` path for free.

SITL is synced to the real flight controller's control parameters before each run —
`GUID_OPTIONS`, `WPNAV_SPEED_UP`, `ANGLE_MAX`, `ATC_SLEW_YAW`, the acceleration limits,
`INS_GYRO_FILTER`, `MOT_THST_EXPO`, and `MOT_THST_HOVER` — so the layer underneath the
gains behaves like the real one.

```bash
python3 sitl_tag_sim.py                                # default scenario
python3 sitl_tag_sim.py --tag-range 6 --tag-skew -35   # harder start
python3 sitl_tag_sim.py --kd-roll 2.5                  # sweep one gain
```

It prints a convergence report: mean and peak error per axis over the last 40% of the run,
against tolerances, plus a sign-flip count on the distance error to detect oscillation.

### Result

Converges from a 4 m start at +20° of skew and a 6 m start at −35°, in both skew
directions. Distance error under 0.08 m, lateral under 0.01 m, vertical under 0.05 m,
residual skew under 3°, with no frames lost.

### Three design faults it caught

1. **Pure-P control cannot work here.** Tilt commands acceleration while the loop controls
   position — a double integrator. The drone pinned maximum forward tilt for the entire
   approach and flew straight through the tag. Fixed by adding velocity damping.
2. **`KD_ROLL` had the wrong sign.** It was anti-damping. Because the velocity signal is
   the tag's motion relative to the camera, `v_right` goes negative as the drone strafes
   right, so braking needs a *positive* coefficient. With it negative, adding "damping"
   pumped energy in and the drone orbited the tag forever.
3. **Yaw-to-centre plus roll-to-null-skew is structurally unstable.** The two loops chase
   each other and the drone orbits. Replaced with the decoupled goal-point controller
   described in [Control system](control-system.md).

Every one of these would have crashed the real drone.

## Hardware harnesses

Each isolates one subsystem so a failure has one obvious cause.

| Script                          | Tests                                                                 |
| ------------------------------- | --------------------------------------------------------------------- |
| `vision_test.py`                | Camera, detection, pose, velocity. **No MAVLink code path at all.** Optional CSV logging |
| `mavlink_test.py`               | Serial link, heartbeat, armed state, mode, link health. **No camera code path** |
| `camera_tune.py`                | Exposure sweep with detection rate and pose jitter; `--live --log` records during a real flight |
| `guided_echo_test.py`           | Streams a gentle 3° roll oscillation and compares the flight controller's echo against what was sent |
| `motor_test_on_tag.py`          | Bench, props off: one motor spin on tag acquisition — how Pi → FC command authority was first proven |
| `vision_to_motor_indicator.py`  | Bench, props off: tag far/left/right/close mapped to individual motor spins |
| `hover_on_tag.py --dry-run`     | Full controller with no flight-controller connection — computes and displays commands only |

## A MAVLink gotcha worth recording

When polling MAVLink in a loop, **drain the queue** with
`while conn.recv_match(blocking=False)`, rather than reading one message per iteration. The
flight controller streams faster than a single read per loop consumes, so the backlog grows
and every reading goes progressively stale. This produced convincing but entirely bogus
"the flight controller is not responding" results before it was found.

## Suggested vision validation pass

1. Place the tag at known distances (0.5 m, 1 m, 2 m) and compare against `distance_m`.
2. Move the tag off-centre and confirm the reported offset direction matches.
3. Move the tag by hand and watch the velocity estimates track, then settle back to zero.
4. If the numbers look wrong, suspect uncalibrated intrinsics first.

<!-- Add SITL convergence plot here: assets/results/sitl-convergence.png -->
<!-- Add in-flight detection-rate plot here: assets/results/detection-rate.png -->
