# Safety and limitations

## Design rules

These are hard constraints on the companion computer's authority, enforced in code rather
than by convention.

1. **The Pi never arms or disarms the vehicle.** Arming is a deliberate pilot action on the
   transmitter. The Pi reads armed state; it never sets it.
2. **The Pi never sends a command that directly actuates motors** during autonomous
   operation. All motion goes through ArduPilot's own control loops.
3. **A transmitter switch pulls the vehicle out of `GUIDED_NOGPS` at any time.** This is a
   normal flight-mode-channel mapping in the flight controller, entirely independent of the
   Pi and of GPS. It is both the engage switch and the emergency exit from autonomous
   control.
4. **Loss of health stops autonomous movement rather than compensating for it.** Stale
   detection, stale heading, or a stale link each have a defined, conservative response.
5. **ArduPilot's own failsafes are the primary safety net.** The Pi-side checks are a
   secondary layer, never the only one.

The one deliberate exception to rule 2 is the bench tooling. `motor_test_on_tag.py` and
`vision_to_motor_indicator.py` issue `MAV_CMD_DO_MOTOR_TEST` with propellers removed; this
is how command authority from the Pi to the flight controller was first demonstrated. Each
spin self-expires on the flight controller after 3 seconds, so a hung or crashed script
cannot leave a motor running.

## Engagement

The controller streams commands only when **all** of the following hold:

- The vehicle is armed
- The flight controller reports mode `GUIDED_NOGPS`
- A heartbeat has arrived within 1.5 s
- An `ATTITUDE` message has arrived within 0.5 s

Until then it computes commands and displays them but sends nothing. The slew state is also
held neutral while disengaged, so engagement always begins from a level command rather than
from whatever the controller happened to be computing.

On exit, the controller makes a best-effort neutral command. If the process dies outright,
ArduPilot's `GUID_TIMEOUT` levels the vehicle when the stream stops.

## Bounded authority

Even fully engaged, the commands are small. Maximum 5° of roll or pitch, slew-limited to
10°/s, with yaw aiming at most 10° ahead of the current heading and thrust confined to
±0.14 around the altitude-hold point.

For scale: a pilot in Stabilize routinely commands `ANGLE_MAX` — 30° — instantly. The
autonomous loop asks strictly *less* of the inner loop than every manual flight already
does.

## The current limitation: vibration

**This is the reason autonomous position hold is not yet reliable in physical flight.**

Detection currently holds for roughly 80% of flight time. The cause is mechanical vibration
reaching the camera. The IMX708 is a rolling-shutter sensor, so it reads the frame row by
row rather than all at once; vibration during that readout shears the image. The tag's
corner geometry is what the pose solve depends on, so the distortion lands directly on the
control input — sometimes as noise, occasionally as a pose metres from reality.

Software has been pushed close to its limit here:

- A 2 ms shutter freezes motion blur across the vibration cycle
- The fastest-readout sensor mode (~8.3 ms) minimises the shear itself
- Detector parameters are loosened for smeared corners and noisy frames
- A pose gate discards single-frame spikes before they reach the derivative term
- A drift-immune jitter metric makes the remaining vibration measurable in flight

What none of that can do is remove the shear. Readout time is a sensor property, and
vibration during readout is a mechanical input. **The remaining work is mechanical:**
damping the camera mount, balancing propellers and motors to attack the source, and — if
the shear persists after both — moving to a global-shutter sensor.

A second outstanding item is inner-loop tuning. `ATC_RAT_*` are still at stock defaults for
a larger airframe. The outer loop commands attitude and trusts ArduPilot to achieve it, so
a badly tuned inner loop degrades everything above it. This is measured rather than assumed:
the controller compares commanded against achieved attitude in flight and warns when the
sustained gap exceeds 3°.

## Known limits by design

- **Monocular vision cannot separate the drone's motion from the tag's.** The velocity
  estimate is explicitly *relative*. That is the correct signal to null for position
  holding — the controller should not care which one moved — but it would need fusing with
  IMU-based ego-motion if the tag's absolute motion were ever required.
- **The pose gate is a despiker, not a tracker.** Two consecutively corrupted frames pass
  it. This is acceptable because sustained corruption never refreshes the staleness clock
  and decays into the tag-lost hover.
- **AprilTags cannot be decoded past roughly 60° of viewing angle.** The approach speed cap
  exists specifically to stop the drone arriving at an angle where the tag disappears.
- **Tag loss triggers no search behaviour.** The controller holds a level hover and reports
  the loss. It does not dead-reckon, because a monocular pose estimate with no position
  source has nothing to dead-reckon from.
- **No altitude reference other than the tag.** Without a rangefinder or optical flow,
  vertical control depends entirely on the tag staying in frame.
