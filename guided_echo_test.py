#!/usr/bin/env python3
import argparse
import math
import time

from pymavlink import mavutil

from mavlink.connection import DEFAULT_BAUD, DEFAULT_DEVICE, FlightControllerLink

SEND_HZ = 20.0
ROLL_AMPLITUDE_DEG = 3.0   # gentle, and harmless with props off on the ground
ROLL_PERIOD_S = 4.0
NEUTRAL_THRUST = 0.5       # hold-altitude demand
LINK_STALE_S = 1.5

TYPE_MASK = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
)


def euler_to_quat(roll_rad, pitch_rad, yaw_rad=0.0):
    cr, sr = math.cos(roll_rad / 2), math.sin(roll_rad / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    cy, sy = math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def quat_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def run(args):
    fc = FlightControllerLink(device=args.device, baud=args.baud)
    fc.connect()
    conn = fc.raw_connection

    # Ask the FC to stream its own attitude target back at 10 Hz (read-only).
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET, 100000, 0, 0, 0, 0, 0)

    print("\nArm the vehicle, then flip your TX switch to GUIDED_NOGPS.")
    print("Watching — Ctrl+C to stop.\n")

    boot = time.monotonic()
    last_send = 0.0
    last_print = 0.0
    last_hb = 0.0
    sent = None
    engaged_ever = False

    try:
        while True:
            now = time.monotonic()
            fc.poll()

            if now - last_hb >= 1.0:
                fc.send_companion_heartbeat()
                last_hb = now

            status = fc.get_status()
            healthy = fc.is_link_healthy(LINK_STALE_S)
            mode = str(status["mode"]) if status else "?"
            armed = bool(status["armed"]) if status else False
            engaged = bool(status and healthy and armed
                           and mode.upper() == "GUIDED_NOGPS")

            if engaged and (now - last_send) >= 1.0 / SEND_HZ:
                engaged_ever = True
                roll = ROLL_AMPLITUDE_DEG * math.sin(
                    2 * math.pi * (now - boot) / ROLL_PERIOD_S)
                conn.mav.set_attitude_target_send(
                    int((now - boot) * 1000),
                    conn.target_system, conn.target_component, TYPE_MASK,
                    euler_to_quat(math.radians(roll), 0.0),
                    0.0, 0.0, 0.0, NEUTRAL_THRUST,
                )
                sent = (roll, 0.0, 0.0, NEUTRAL_THRUST)
                last_send = now

            if now - last_print >= 0.5:
                at = fc.get_latest("ATTITUDE_TARGET", max_age_s=1.0)
                if at is not None:
                    er, ep, _ = quat_to_euler(at.q)
                    echo = (er, ep, math.degrees(at.body_yaw_rate), at.thrust)
                else:
                    echo = None

                print(f"armed={'Y' if armed else 'N'}  mode={mode:<14} "
                      f"engaged={'YES' if engaged else 'no ':<3}")
                if engaged and sent:
                    print(f"   SENT: roll={sent[0]:+6.2f}  thrust={sent[3]:.2f}")
                    if echo:
                        print(f"   ECHO: roll={echo[0]:+6.2f}  thrust={echo[3]:.2f}")
                        match = (abs(echo[0] - sent[0]) < 2.0
                                 and abs(echo[3] - sent[3]) < 0.1)
                        print("   >>> " + ("ECHO TRACKS SENT — FC IS RECEIVING "
                                            "OUR COMMANDS. Control path OK."
                                            if match else
                                            "echo does not match sent — FC is NOT "
                                            "acting on our targets."))
                    else:
                        print("   ECHO: (no ATTITUDE_TARGET received)")
                else:
                    print("   not engaged -> sending NOTHING (this is correct). "
                          "Need armed + GUIDED_NOGPS.")
                print()
                last_print = now

    except KeyboardInterrupt:
        pass
    finally:
        if not engaged_ever:
            print("\nNever reached armed + GUIDED_NOGPS, so nothing was ever sent.")
            print("That alone explains a frozen/zero echo — it is not a comms fault.")
        fc.close()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--device", default=DEFAULT_DEVICE)
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    return p.parse_args()


if __name__ == "__main__":
    run(parse_args())
