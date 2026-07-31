# Control system

The outer loop lives in `compute_commands()` in
[`hover_on_tag.py`](../hover_on_tag.py). It is a pure function: detection plus relative
velocity in, four commands out. Clamping happens inside it; slew limiting happens at the
send site.

## The goal point

The obvious controller — yaw to centre the tag, roll to null its skew — does not work.
The two loops chase each other: strafing changes the bearing, which yaws the drone, which
changes the observed skew, which strafes again. In simulation the drone orbited the tag
indefinitely and never converged.

The working design decouples them. From the tag's pose, the controller computes a single
**goal point**: the position `--distance` metres out along the tag's outward normal, which
is where the drone is supposed to end up. Roll and pitch drive straight at that point.
Yaw is handled independently and only keeps the nose pointed at the tag.

```
horiz  = hypot(fwd, right)
u      = (-fwd, -right) / horiz          # unit vector from tag to drone
n      = rotate(u, -skew)                # the tag's outward normal, in our body frame
e_fwd  = fwd   + distance · n_x          # goal point, relative to the drone
e_right= right + distance · n_y
```

The setpoint distance is not a free parameter. The goal point's lateral offset is
`distance · sin(skew)`, so a **larger** distance gives the squaring-up loop more
authority. Moving the setpoint from 0.5 m to 1.0 m improved steady-state skew from about
5.8° to about 2.8° with no other change. It also has to match the camera's fixed focus
distance, or the tag sits outside the depth of field. Changing one means changing the
other and re-running `sitl_tag_sim.py`.

## The control law

```
yaw_correction = clamp(K_p,yaw · bearing,                              ±10°)
pitch          = clamp(K_p,pitch · e_fwd_capped + K_d,pitch · v_fwd,   ±5°)
roll           = clamp(K_p,roll  · e_right      + K_d,roll  · v_right, ±5°)
thrust         = 0.5 + clamp(K_p,thr · down + K_d,thr · v_down,        ±0.14)
```

Thrust is a **climb-rate demand**, not a throttle level: with `GUID_OPTIONS = 0`, 0.5 means
hold altitude and the scale comes from `WPNAV_SPEED_UP`.

| Parameter                 | Value  | Role                                              |
| ------------------------- | ------ | ------------------------------------------------- |
| `KP_YAW`                  | 0.5    | Tag bearing (deg) → yaw angle correction (deg)    |
| `KP_ROLL` / `KD_ROLL`     | 4.0 / +3.2 | Lateral goal-point error and its damping      |
| `KP_PITCH` / `KD_PITCH`   | −1.5 / −3.2 | Forward goal-point error and its damping     |
| `KP_THRUST` / `KD_THRUST` | −0.30 / −0.16 | Vertical tag offset and its damping        |
| `MAX_ROLL_DEG`, `MAX_PITCH_DEG` | 5°  | Command clamp                               |
| `MAX_YAW_CORRECTION_DEG`  | 10°    | How far ahead of current heading yaw may aim      |
| `MAX_APPROACH_ERR_M`      | 0.7 m  | Cap on the forward error the P term may act on    |
| `MAX_ANGLE_STEP_DEG`      | 0.5°   | Per-tick slew limit — 10°/s at 20 Hz              |
| `MAX_THRUST_STEP`         | 0.005  | Per-tick thrust slew limit                        |

Deadbands of 10 mm on bearing, 50 mm on position, and 50 mm on vertical offset keep the
vehicle from chattering against sensor noise at the setpoint.

## Why the derivative terms are mandatory

Tilt commands **acceleration**, but the loop controls **position**. That is a double
integrator, and a purely proportional controller on a double integrator always overshoots.
Simulation demonstrated it clearly: with P only, the drone pinned maximum forward tilt for
the entire approach, accelerated the whole way in, and flew straight through the tag.

The damping terms act on relative velocity from
[`vision/velocity_estimator.py`](../vision/velocity_estimator.py). Each `K_d` carries the
same sign as its `K_p`, so motion toward the target produces braking.

**`KD_ROLL` is positive on purpose**, and this is the least intuitive part of the loop. The
velocity signal is the *tag's* velocity relative to the camera, not the drone's velocity —
so `v_right` goes negative as the drone strafes right. Braking therefore needs a positive
coefficient. It was negative once. That is anti-damping, and it pumped energy into the
system until the drone orbited the tag.

## Why the approach is speed-capped

`MAX_APPROACH_ERR_M` bounds the forward error the proportional term is allowed to see.
Without it, the drone pins maximum forward tilt from far away and rushes the tag. Because
the goal point's lateral offset is only `distance · sin(skew)`, the squaring-up loop is
comparatively weak, so the drone arrives before it has squared up — and an AprilTag cannot
be decoded past roughly 60° of viewing angle. In simulation the viewing angle blew past 60°
and the tag simply disappeared. The cap bounds the approach speed so squaring-up keeps pace.

## Message format

Two details about `SET_ATTITUDE_TARGET` were found the hard way and must not be
"simplified" back:

1. **`type_mask` must be 0.** ArduCopter accepts either all three body-rate-ignore bits set
   or all three clear. The natural-looking choice — ignore roll and pitch rate, supply yaw
   rate — is an illegal mix, and the flight controller discards the entire message
   (`"The body rates are ill-defined"` → `hold_position(); return`). So all three rates are
   supplied as zeros and the attitude is carried entirely in the quaternion.
2. **The quaternion's yaw is an absolute earth-frame heading.** Sending `yaw = 0` does not
   mean "hold current heading"; it commands the vehicle to turn and face north. The target
   is rebuilt every tick as `current ATTITUDE yaw + correction`, which makes a fresh
   `ATTITUDE` message a hard prerequisite for sending anything at all. A yaw *rate* in
   `body_yaw_rate` does nothing — the quaternion's yaw always wins.

## Transfer from simulation to hardware

The gains transfer reasonably because the loop commands angles and a climb rate rather
than motor outputs: `a = g·tan(roll)` is airframe-independent, and `WPNAV_SPEED_UP` is the
same on both. The real 816 g airframe has roughly a 7:1 thrust-to-weight ratio and is
punchier than SITL's default quad, which puts these gains on the conservative side — the
safe direction to be wrong in.

Drag, wind, and camera latency are not modelled. These are a validated starting point, not
final numbers.

## Failure behaviour

- **Tag lost** (no accepted detection for 0.5 s): commands decay to a level,
  altitude-holding hover. No dead reckoning, no search pattern.
- **Link stale** (no flight-controller heartbeat for 1.5 s): treated as disengaged.
- **Heading stale** (no `ATTITUDE` for 0.5 s): nothing is sent at all.
- **Pilot disengages**: flipping out of `GUIDED_NOGPS` stops the vehicle acting on
  anything the Pi sends, instantly and independently of the Pi's own state.
