#!/usr/bin/env python3
"""
sitl_validate.py — validate the GUIDED_NOGPS control path against ArduPilot SITL.

This answers the questions the bench physically CANNOT answer, because
ArduCopter refuses to run guided attitude control while it believes it is
landed (mode_guided.cpp: angle_control_run() early-returns into
make_safe_ground_handling() unless auto_armed && !land_complete). On the
ground the FC discards our attitude targets, which is why ATTITUDE_TARGET
echoes zero and motors never respond. In the air, it acts on them.

So we fly a SIMULATED copter and check, in flight:

  1. INGESTION  — does the FC's ATTITUDE_TARGET (#83) echo track the
                  SET_ATTITUDE_TARGET (#82) we send?
  2. SIGNS      — does each axis move the vehicle the way hover_on_tag.py
                  assumes? Commanding +roll must actually drift RIGHT,
                  -pitch must drive FORWARD, +yaw-rate must turn RIGHT,
                  thrust >0.5 must CLIMB. A wrong sign here means the real
                  drone would fly AWAY from the tag.

!!! SITL ONLY !!!
This script arms and flies the vehicle. It refuses to run against a serial
device (a real flight controller) — see require_sitl(). Never point it at
/dev/serial0. The Pi still never arms the real vehicle; that is the pilot's
job via the transmitter.

Usage (with SITL running):
    sim_vehicle.py -v ArduCopter -f quad --no-mavproxy   # in ardupilot/
    python3 sitl_validate.py
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

# Import the REAL controller module so we validate the shipping send path,
# type_mask and quaternion construction — not a reimplementation of them.
import hover_on_tag

SEND_HZ = 20.0
TAKEOFF_ALT_M = 15.0
NEUTRAL_THRUST = 0.5

# ArduPilot accepts ONLY all-three-rates-ignored or all-three-rates-supplied.
# A mix (e.g. ignore roll+pitch rate but supply yaw rate) is rejected outright:
#   "The body rates are ill-defined" -> hold_position(); return;
# We want roll/pitch ANGLES + a yaw RATE, so we must supply all three rates
# (mask 0) and put zeros in the roll/pitch rate fields.
TYPE_MASK = 0


def require_sitl(device):
    if not (device.startswith("tcp:") or device.startswith("udp:")):
        sys.exit(f"REFUSING TO RUN: device '{device}' is not a SITL endpoint.\n"
                 "This script ARMS AND FLIES the vehicle and is for the "
                 "simulator only (tcp:/udp:). Never run it against a real FC.")


def euler_to_quat(roll_rad, pitch_rad, yaw_rad=0.0):
    cr, sr = math.cos(roll_rad / 2), math.sin(roll_rad / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    cy, sy = math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def quat_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def send_target(conn, boot, roll_deg, pitch_deg, yaw_corr_deg, thrust):
    """Send via hover_on_tag's OWN send function, so we validate the shipping
    code path rather than a copy of it.

    yaw_corr_deg is a delta on the FC's current heading — exactly how
    hover_on_tag builds its absolute yaw target each tick.
    """
    att = conn.messages.get("ATTITUDE")
    if att is None:
        return  # no heading => do not send (same rule hover_on_tag enforces)
    target_yaw = att.yaw + math.radians(yaw_corr_deg)
    hover_on_tag.send_attitude_target(conn, boot, roll_deg, pitch_deg,
                                      target_yaw, thrust)


def wait_for(conn, cond, timeout, desc):
    start = time.time()
    while time.time() - start < timeout:
        conn.recv_match(blocking=True, timeout=0.5)
        if cond():
            return True
        time.sleep(0.05)
    print(f"  TIMEOUT waiting for {desc}")
    return False


def set_mode_confirmed(conn, mode, timeout=20):
    """Set flight mode and WAIT until the heartbeat confirms it.

    conn.set_mode() alone is fire-and-forget, and conn.flightmode only updates
    when messages are pumped — so a bare set_mode()+sleep() can silently leave
    you in the old mode. Retry until the FC actually reports the new mode.
    """
    t0 = time.time()
    next_try = 0.0
    while time.time() - t0 < timeout:
        if time.time() > next_try:
            conn.set_mode(mode)
            next_try = time.time() + 2.0
        conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if conn.flightmode == mode:
            return True
    return False


def set_param(conn, name, value):
    conn.mav.param_set_send(conn.target_system, conn.target_component,
                            name.encode(), float(value),
                            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.3)


def body_velocity(conn):
    """Forward/right velocity in the BODY frame, from NED velocity + yaw."""
    lp = conn.messages.get("LOCAL_POSITION_NED")
    att = conn.messages.get("ATTITUDE")
    if lp is None or att is None:
        return None
    yaw = att.yaw
    fwd = lp.vx * math.cos(yaw) + lp.vy * math.sin(yaw)
    right = -lp.vx * math.sin(yaw) + lp.vy * math.cos(yaw)
    return fwd, right, lp.vz  # vz: NED, positive = descending


def run(args):
    require_sitl(args.device)

    print(f"Connecting to SITL at {args.device} ...")
    conn = mavutil.mavlink_connection(args.device)
    conn.wait_heartbeat()
    print(f"Heartbeat OK — system {conn.target_system}\n")

    for mid in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET,
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED):
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, 50000,
            0, 0, 0, 0, 0)

    # thrust must mean CLIMB RATE (0.5 = hold alt) — what hover_on_tag assumes
    print("Setting GUID_OPTIONS=0 (thrust = climb rate, 0.5 = hold altitude)")
    set_param(conn, "GUID_OPTIONS", 0)

    # SITL needs ~25 s for GPS lock + EKF convergence before it will arm.
    print("Waiting 30 s for GPS lock / EKF convergence ...")
    t0 = time.time()
    while time.time() - t0 < 30:
        m = conn.recv_match(blocking=True, timeout=1)
        if m is not None and m.get_type() == "STATUSTEXT":
            print(f"  FC: {m.text}")

    # --- Take off in GUIDED (SITL has GPS, so plain GUIDED works) -------------
    # The EKF needs a position estimate before GUIDED will arm, and how long
    # that takes varies. Retry mode+arm every 5 s, pumping messages throughout
    # (a bare sleep() would leave conn.flightmode stale), until it takes.
    print("Waiting for EKF position estimate, then arming (GUIDED) ...")
    t0 = time.time()
    next_try = 0.0
    while time.time() - t0 < 150 and not conn.motors_armed():
        m = conn.recv_match(blocking=True, timeout=0.5)
        if m is not None and m.get_type() == "STATUSTEXT":
            txt = m.text
            if "EKF" not in txt and "GPS" not in txt:
                print(f"  FC: {txt}")
        if time.time() > next_try:
            conn.set_mode("GUIDED")
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            next_try = time.time() + 5.0
    # make sure GUIDED actually stuck before we ask for takeoff
    set_mode_confirmed(conn, "GUIDED")

    if not conn.motors_armed():
        sys.exit("could not arm SITL (see FC messages above)")
    print(f"  armed, mode = {conn.flightmode}")

    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                               0, 0, 0, 0, 0, 0, TAKEOFF_ALT_M)

    def at_alt():
        lp = conn.messages.get("LOCAL_POSITION_NED")
        return lp is not None and -lp.z > TAKEOFF_ALT_M * 0.9

    if not wait_for(conn, at_alt, 60, f"takeoff to {TAKEOFF_ALT_M} m"):
        sys.exit("takeoff failed")
    print(f"  airborne at {-conn.messages['LOCAL_POSITION_NED'].z:.1f} m\n")

    # --- Hand over to GUIDED_NOGPS -------------------------------------------
    # This is the real hand-over: pilot flies up manually / in GUIDED, then
    # switches to GUIDED_NOGPS and the companion takes over attitude control.
    print("Switching to GUIDED_NOGPS (the mode hover_on_tag.py uses) ...")
    if not set_mode_confirmed(conn, "GUIDED_NOGPS"):
        sys.exit("FC never entered GUIDED_NOGPS — is the mode available in this build?")
    print(f"  mode = {conn.flightmode}  (confirmed)\n")

    # --- Test sequence -------------------------------------------------------
    # (label, roll, pitch, yaw_rate, thrust, seconds, expectation-check)
    tests = [
        ("level hover (baseline)", 0.0, 0.0, 0.0, 0.5, 5.0, None),
        ("roll +8 (expect drift RIGHT)", 8.0, 0.0, 0.0, 0.5, 8.0, "right"),
        ("level (settle)", 0.0, 0.0, 0.0, 0.5, 6.0, None),
        ("pitch -8 (expect FORWARD)", 0.0, -8.0, 0.0, 0.5, 8.0, "fwd"),
        ("level (settle)", 0.0, 0.0, 0.0, 0.5, 6.0, None),
        ("yaw target +40deg (expect turn RIGHT)", 0.0, 0.0, 40.0, 0.5, 10.0, "yaw"),
        ("level (settle)", 0.0, 0.0, 0.0, 0.5, 4.0, None),
        ("thrust 0.75 (expect CLIMB)", 0.0, 0.0, 0.0, 0.75, 8.0, "climb"),
    ]

    boot = time.monotonic()
    results = []
    echo_ok = False

    for label, roll, pitch, yawrate, thrust, dur, expect in tests:
        print(f"--- {label}")
        t0 = time.monotonic()
        last_send = 0.0
        yaw0 = None
        alt0 = None
        samples = []
        echo_seen = []

        yaw_target_abs = None
        while time.monotonic() - t0 < dur:
            now = time.monotonic()
            # DRAIN the queue. Reading only one message per iteration lets the
            # backlog grow (the FC streams ~60 msg/s across 3 types), which made
            # every altitude/yaw reading progressively stale and produced bogus
            # "no response" results.
            while conn.recv_match(blocking=False) is not None:
                pass
            if now - last_send >= 1.0 / SEND_HZ:
                if expect == "yaw":
                    # Latch ONE absolute heading target and hold it, so we test
                    # convergence to a heading. (Re-adding the offset to the
                    # live heading every tick would keep the target permanently
                    # ahead and spin the vehicle forever — that is rate-like,
                    # not what the real closed loop does, since the tag bearing
                    # shrinks as the drone turns toward it.)
                    att = conn.messages.get("ATTITUDE")
                    if yaw_target_abs is None and att is not None:
                        yaw_target_abs = att.yaw + math.radians(yawrate)
                    if yaw_target_abs is not None:
                        hover_on_tag.send_attitude_target(
                            conn, boot, roll, pitch, yaw_target_abs, thrust)
                else:
                    send_target(conn, boot, roll, pitch, yawrate, thrust)
                last_send = now

            att = conn.messages.get("ATTITUDE")
            lp = conn.messages.get("LOCAL_POSITION_NED")
            at = conn.messages.get("ATTITUDE_TARGET")
            if att and yaw0 is None:
                yaw0 = math.degrees(att.yaw)
            if lp and alt0 is None:
                alt0 = -lp.z
            bv = body_velocity(conn)
            if bv:
                samples.append(bv)
            if at is not None:
                er, ep, _ = quat_to_euler(at.q)
                echo_seen.append((er, ep, at.thrust))
            time.sleep(0.02)

        # measure
        att = conn.messages.get("ATTITUDE")
        lp = conn.messages.get("LOCAL_POSITION_NED")
        n = max(1, len(samples) // 2)
        late = samples[-n:] if samples else [(0, 0, 0)]
        mean_fwd = sum(s[0] for s in late) / len(late)
        mean_right = sum(s[1] for s in late) / len(late)
        mean_vz = sum(s[2] for s in late) / len(late)
        meas_roll = math.degrees(att.roll) if att else 0.0
        meas_pitch = math.degrees(att.pitch) if att else 0.0
        dyaw = (math.degrees(att.yaw) - yaw0) if (att and yaw0 is not None) else 0.0
        dyaw = (dyaw + 180) % 360 - 180
        dalt = ((-lp.z) - alt0) if (lp and alt0 is not None) else 0.0

        # Echo check: compare ONLY roll/pitch. The echoed `thrust` field is the
        # FC's ACTUAL throttle output (~hover throttle), not a readback of our
        # command — with GUID_OPTIONS=0 our 0.5 means "climb rate 0", so it
        # would never match. Comparing it was a bug in this harness.
        if echo_seen:
            er, ep, et = echo_seen[-1]
            if abs(er - roll) < 2.5 and abs(ep - pitch) < 2.5:
                echo_ok = True
            print(f"    echo:  roll={er:+.1f} pitch={ep:+.1f} "
                  f"(sent roll={roll:+.1f} pitch={pitch:+.1f})  "
                  f"[fc throttle={et:.2f}]")
        else:
            print("    echo:  NONE RECEIVED")

        abs_alt = (-lp.z) if lp else float("nan")
        print(f"    imu:   roll={meas_roll:+.1f} pitch={meas_pitch:+.1f}  "
              f"dyaw={dyaw:+.1f}deg  dalt={dalt:+.2f}m  ALT={abs_alt:.1f}m  "
              f"armed={bool(conn.motors_armed())} mode={conn.flightmode}")
        print(f"    body:  fwd={mean_fwd:+.2f} right={mean_right:+.2f} "
              f"vz={mean_vz:+.2f} m/s")

        if expect == "right":
            ok = mean_right > 0.5
            results.append(("+roll drifts RIGHT", ok, f"right={mean_right:+.2f} m/s"))
        elif expect == "fwd":
            ok = mean_fwd > 0.5
            results.append(("-pitch drives FORWARD", ok, f"fwd={mean_fwd:+.2f} m/s"))
        elif expect == "yaw":
            ok = dyaw > 25
            results.append(("+yaw target turns RIGHT", ok, f"dyaw={dyaw:+.1f} deg (wanted +40)"))
        elif expect == "climb":
            ok = dalt > 0.5
            results.append(("thrust>0.5 CLIMBS", ok, f"dalt={dalt:+.2f} m"))
        print()

    results.insert(0, ("FC ingests SET_ATTITUDE_TARGET (echo tracks)", echo_ok, ""))

    # --- Land ---------------------------------------------------------------
    print("Landing ...")
    conn.set_mode("LAND")
    wait_for(conn, lambda: not conn.motors_armed(), 90, "disarm after land")

    print("\n" + "=" * 62)
    print("SITL VALIDATION RESULTS")
    print("=" * 62)
    for name, ok, detail in results:
        print(f"  [{'PASS' if ok else 'FAIL'}]  {name:<42} {detail}")
    print("=" * 62)
    if all(ok for _, ok, _ in results):
        print("All checks passed — control path and sign conventions are correct.")
    else:
        print("FAILURES above. A failed sign means hover_on_tag.py's gain for that")
        print("axis must have its sign flipped, or the drone will fly the WRONG WAY.")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", default="tcp:127.0.0.1:5760",
                   help="SITL endpoint. Must be tcp: or udp: — never a serial port.")
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())
