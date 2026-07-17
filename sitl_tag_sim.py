#!/usr/bin/env python3
"""
sitl_tag_sim.py — closed-loop tag-following simulation, for tuning the gains.

Places a VIRTUAL AprilTag in the SITL world, and every frame synthesises the
detection the real camera would produce from the simulated drone's actual
position and attitude. That synthetic detection is fed through
hover_on_tag.compute_commands() — the REAL control law, not a copy — and the
resulting attitude target is streamed to the simulated FC.

So this closes the whole loop: vision -> control law -> FC -> vehicle motion ->
new vision. It answers the questions the axis-by-axis test cannot:
  - does it CONVERGE on the tag (--distance, centred, square-on)?
  - does it OSCILLATE, overshoot, or diverge?
  - are the gains in the right ballpark?

WHY THE GAINS TRANSFER TO THE REAL DRONE (mostly):
We command angles and a climb rate, not motor outputs, and ArduPilot's inner
loop absorbs the airframe. Lateral accel from a bank angle is a = g*tan(roll)
— pure physics, identical on any multirotor. And thrust->climb-rate uses
WPNAV_SPEED_UP, which is 250 on both SITL and the real FC. The real drone is
punchier (816 g, ~7:1 thrust/weight) than SITL's default quad, so gains tuned
here come out CONSERVATIVE on the real vehicle — the safe direction to err.

NOT MODELLED: drag, wind, camera noise/latency, rolling shutter. Treat the
output as "gains of the right order that converge cleanly", not final numbers.

!!! SITL ONLY !!! Arms and flies. Refuses to run against a serial device.

Usage:
    # terminal 1
    cd /tmp/sitl_run && ~/ardupilot/build/sitl/bin/arducopter --model quad \
        --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm
    # terminal 2
    python3 sitl_tag_sim.py                    # uses hover_on_tag's gains
    python3 sitl_tag_sim.py --kp-pitch -3.0    # override one gain to tune
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

import hover_on_tag as hot
from vision.velocity_estimator import VelocityEstimator

SEND_HZ = 20.0
VISION_HZ = 30.0          # camera frame rate we simulate
TAKEOFF_ALT_M = 5.0
CAM_HFOV_DEG = 66.0       # Camera Module 3 horizontal FOV — tag outside this is LOST
MAX_VIEW_ANGLE_DEG = 60.0 # AprilTag cannot be decoded edge-on past ~60 deg of skew
LINK_STALE_S = 1.5

# Parameters copied from the real flight controller so SITL's control layer
# behaves like the real one. These are the params our gains couple to.
REAL_FC_PARAMS = {
    "GUID_OPTIONS": 0,        # thrust = climb rate, 0.5 = hold altitude
    "WPNAV_SPEED_UP": 250,    # 2.5 m/s — sets the thrust->climb-rate scale
    "WPNAV_SPEED_DN": 150,
    "ANGLE_MAX": 3000,        # 30 deg
    "ATC_SLEW_YAW": 6000,     # 60 deg/s — how fast yaw converges
    "ATC_ACCEL_R_MAX": 146100,
    "ATC_ACCEL_P_MAX": 146100,
    "ATC_ACCEL_Y_MAX": 29700,
    "INS_GYRO_FILTER": 57,
    "MOT_THST_EXPO": 0.54,
    "MOT_THST_HOVER": 0.20,   # real drone's ~7:1 thrust/weight
    "MOT_HOVER_LEARN": 0,     # don't let it drift during the run
}


def require_sitl(device):
    if not (device.startswith("tcp:") or device.startswith("udp:")):
        sys.exit(f"REFUSING: '{device}' is not a SITL endpoint. This script arms "
                 "and flies; simulator only.")


def set_param(conn, name, value):
    conn.mav.param_set_send(conn.target_system, conn.target_component,
                            name.encode(), float(value),
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.25)


def drain(conn):
    while conn.recv_match(blocking=False) is not None:
        pass


def set_mode_confirmed(conn, mode, timeout=25):
    t0, nxt = time.time(), 0.0
    while time.time() - t0 < timeout:
        if time.time() > nxt:
            conn.set_mode(mode)
            nxt = time.time() + 2.0
        conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if conn.flightmode == mode:
            return True
    return False


def synth_detection(conn, tag_n, tag_e, tag_d, tag_heading_rad):
    """Build the detection dict the real AprilTagDetector would emit.

    Uses the simulated drone's true position/attitude to compute where the tag
    sits in the drone's FRD body frame — i.e. what the camera would see.
    Returns None if the tag is behind the drone or outside the camera FOV
    (which exercises hover_on_tag's TAG_LOST path for free).
    """
    lp = conn.messages.get("LOCAL_POSITION_NED")
    att = conn.messages.get("ATTITUDE")
    if lp is None or att is None:
        return None

    # vector drone -> tag, in NED
    dn = tag_n - lp.x
    de = tag_e - lp.y
    dd = tag_d - lp.z

    yaw = att.yaw
    fwd = dn * math.cos(yaw) + de * math.sin(yaw)
    right = -dn * math.sin(yaw) + de * math.cos(yaw)
    down = dd

    if fwd <= 0.05:
        return None  # tag is behind us
    bearing = math.degrees(math.atan2(right, fwd))
    if abs(bearing) > CAM_HFOV_DEG / 2.0:
        return None  # outside the camera's field of view

    # Tag skew: horizontal angle between the tag's outward normal and the
    # direction from the tag to the drone. 0 => we are square-on to its face.
    nx, ny = math.cos(tag_heading_rad), math.sin(tag_heading_rad)  # tag normal (NED horiz)
    vx, vy = -dn, -de                                              # tag -> drone
    skew = math.degrees(math.atan2(nx * vy - ny * vx, nx * vx + ny * vy))

    # A real AprilTag cannot be decoded edge-on — past roughly 60 deg of viewing
    # angle the corners collapse and detection fails. Without this the sim
    # happily "sees" the tag at 100+ deg, which flatters the controller with
    # information the camera could never actually give it.
    if abs(skew) > MAX_VIEW_ANGLE_DEG:
        return None

    return {
        "tag_id": 0,
        "timestamp": time.monotonic(),
        "fwd_m": fwd, "right_m": right, "down_m": down,
        "yaw_deg": skew,
        "distance_m": math.sqrt(fwd * fwd + right * right + down * down),
    }


def run(args):
    require_sitl(args.device)

    # Apply gain overrides (tuning knobs) onto the real module.
    for name, val in (("KP_ROLL", args.kp_roll), ("KP_PITCH", args.kp_pitch),
                      ("KP_YAW", args.kp_yaw), ("KP_THRUST", args.kp_thrust),
                      ("KD_ROLL", args.kd_roll), ("KD_PITCH", args.kd_pitch),
                      ("KD_THRUST", args.kd_thrust)):
        if val is not None:
            setattr(hot, name, val)
    print("P gains: roll=%.3f pitch=%.3f yaw=%.3f thrust=%.3f" %
          (hot.KP_ROLL, hot.KP_PITCH, hot.KP_YAW, hot.KP_THRUST))
    print("D gains: roll=%.3f pitch=%.3f thrust=%.3f" %
          (hot.KD_ROLL, hot.KD_PITCH, hot.KD_THRUST))

    conn = mavutil.mavlink_connection(args.device)
    conn.wait_heartbeat()
    print(f"connected to SITL (system {conn.target_system})")

    for mid in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED):
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                   mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                   0, mid, 40000, 0, 0, 0, 0, 0)

    print("syncing SITL to the real FC's control params ...")
    for k, v in REAL_FC_PARAMS.items():
        set_param(conn, k, v)

    print("waiting for EKF, arming, taking off ...")
    t0, nxt = time.time(), 0.0
    while time.time() - t0 < 150 and not conn.motors_armed():
        conn.recv_match(blocking=True, timeout=0.5)
        if time.time() > nxt:
            conn.set_mode("GUIDED")
            conn.mav.command_long_send(conn.target_system, conn.target_component,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 0, 0, 0, 0, 0, 0)
            nxt = time.time() + 5.0
    if not conn.motors_armed():
        sys.exit("could not arm SITL")
    set_mode_confirmed(conn, "GUIDED")
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                               0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M)
    t0 = time.time()
    while time.time() - t0 < 60:
        drain(conn)
        lp = conn.messages.get("LOCAL_POSITION_NED")
        if lp and -lp.z > TAKEOFF_ALT_M * 0.9:
            break
        time.sleep(0.1)
    lp = conn.messages["LOCAL_POSITION_NED"]
    att = conn.messages["ATTITUDE"]
    print(f"airborne at {-lp.z:.1f} m, heading {math.degrees(att.yaw):+.0f} deg")

    # --- Place the virtual tag ------------------------------------------------
    # Ahead of the drone, at its altitude, its face turned toward the drone but
    # deliberately offset laterally and skewed, so every axis has real work to do.
    yaw0 = att.yaw
    tag_n = lp.x + args.tag_range * math.cos(yaw0) - args.tag_offset * math.sin(yaw0)
    tag_e = lp.y + args.tag_range * math.sin(yaw0) + args.tag_offset * math.cos(yaw0)
    tag_d = lp.z + args.tag_height          # +down => below the drone
    # tag normal points back toward the drone, plus a skew so it is not square-on
    tag_heading = yaw0 + math.pi + math.radians(args.tag_skew)
    print(f"virtual tag: {args.tag_range} m ahead, {args.tag_offset:+.1f} m lateral, "
          f"{args.tag_height:+.1f} m below, face skewed {args.tag_skew:+.0f} deg\n")

    if not set_mode_confirmed(conn, "GUIDED_NOGPS"):
        sys.exit("could not enter GUIDED_NOGPS")
    print("GUIDED_NOGPS — closing the loop\n")
    print(f"{'t':>5} {'dist':>6} {'lat':>7} {'vert':>7} {'skew':>7} "
          f"{'roll':>6} {'pitch':>6} {'dyaw':>6} {'thr':>5}  state")

    boot = time.monotonic()
    t_start = time.monotonic()
    last_send = last_vision = last_print = 0.0
    det = None
    vel = (0.0, 0.0, 0.0)
    velocity = VelocityEstimator()
    history = []
    lost_count = 0

    while time.monotonic() - t_start < args.duration:
        now = time.monotonic()
        drain(conn)

        if now - last_vision >= 1.0 / VISION_HZ:
            det = synth_detection(conn, tag_n, tag_e, tag_d, tag_heading)
            if det is not None:
                vel = velocity.update(det["tag_id"], det["fwd_m"], det["right_m"],
                                      det["down_m"], det["timestamp"])
            last_vision = now

        if now - last_send >= 1.0 / SEND_HZ:
            att = conn.messages.get("ATTITUDE")
            if att is not None:
                if det is not None:
                    roll, pitch, yaw_corr, thrust = hot.compute_commands(det, vel, args.distance)
                    state = "ACTIVE"
                else:
                    roll, pitch, yaw_corr, thrust = hot.NEUTRAL_COMMANDS
                    state = "TAG_LOST"
                    lost_count += 1
                hot.send_attitude_target(conn, boot, roll, pitch,
                                         att.yaw + math.radians(yaw_corr), thrust)
                last_send = now

        if now - last_print >= 0.5:
            t = now - t_start
            if det is not None:
                # errors the controller is trying to null (with cam offsets applied)
                f = det["fwd_m"] + hot.CAM_OFFSET_FWD_M
                r = det["right_m"] + hot.CAM_OFFSET_RIGHT_M
                d = det["down_m"] + hot.CAM_OFFSET_DOWN_M
                dist_err = f - args.distance
                history.append((t, dist_err, r, d, det["yaw_deg"]))
                print(f"{t:5.1f} {f:6.2f} {r:+7.3f} {d:+7.3f} {det['yaw_deg']:+7.1f} "
                      f"{roll:+6.1f} {pitch:+6.1f} {yaw_corr:+6.1f} {thrust:5.2f}  {state}")
            else:
                print(f"{t:5.1f} {'--':>6} {'--':>7} {'--':>7} {'--':>7} "
                      f"{'--':>6} {'--':>6} {'--':>6} {'--':>5}  {state}")
            last_print = now
        time.sleep(0.005)

    print("\nlanding ...")
    conn.set_mode("LAND")

    # --- Verdict --------------------------------------------------------------
    print("\n" + "=" * 64)
    print("CONVERGENCE REPORT")
    print("=" * 64)
    if not history:
        print("  tag never acquired — nothing to report")
        return
    tail = [h for h in history if h[0] >= args.duration * 0.6]  # last 40%
    if not tail:
        tail = history[-5:]

    def stats(idx, label, unit, tol):
        vals = [abs(h[idx]) for h in tail]
        mean = sum(vals) / len(vals)
        peak = max(vals)
        ok = mean < tol
        print(f"  [{'OK ' if ok else 'OFF'}] {label:22} mean |err| = {mean:6.3f} {unit}"
              f"   peak = {peak:6.3f} {unit}   (tol {tol})")
        return ok, mean, peak

    a, _, _ = stats(1, "distance to setpoint", "m", 0.15)
    b, _, _ = stats(2, "lateral offset", "m", 0.15)
    c, _, _ = stats(3, "vertical offset", "m", 0.15)
    d, _, _ = stats(4, "tag skew (square-on)", "deg", 8.0)

    # oscillation: sign changes in the distance error over the tail
    sgn = [1 if h[1] > 0 else -1 for h in history]
    flips = sum(1 for i in range(1, len(sgn)) if sgn[i] != sgn[i - 1])
    print(f"\n  distance-error sign flips: {flips}  "
          f"({'oscillating — reduce gain' if flips > 6 else 'settled'})")
    print(f"  frames with tag lost:      {lost_count}")
    print("=" * 64)
    if all([a, b, c, d]) and flips <= 6:
        print("CONVERGED — gains are in a sane range.")
    else:
        print("NOT converged. Raise the gain for any axis with large residual error;")
        print("lower it for any axis that oscillates (many sign flips).")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", default="tcp:127.0.0.1:5760")
    p.add_argument("--distance", type=float, default=1.0, help="hover setpoint (m)")
    p.add_argument("--duration", type=float, default=60.0)
    p.add_argument("--tag-range", type=float, default=4.0, help="initial distance to tag")
    p.add_argument("--tag-offset", type=float, default=1.0, help="initial lateral offset")
    p.add_argument("--tag-height", type=float, default=1.0, help="tag below drone (m)")
    p.add_argument("--tag-skew", type=float, default=20.0, help="tag face skew (deg)")
    p.add_argument("--kp-roll", type=float, default=None)
    p.add_argument("--kp-pitch", type=float, default=None)
    p.add_argument("--kp-yaw", type=float, default=None)
    p.add_argument("--kp-thrust", type=float, default=None)
    p.add_argument("--kd-roll", type=float, default=None)
    p.add_argument("--kd-pitch", type=float, default=None)
    p.add_argument("--kd-thrust", type=float, default=None)
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())
