#!/usr/bin/env python3
"""
mavlink_test.py — MAVLink-only connection validation harness, companion-computer side.

Verifies the Pi <-> flight-controller serial link works in isolation, with
zero camera/vision code path. Connects, waits for the first heartbeat, then
prints armed state, flight mode, and link health at 1 Hz while sending this
Pi's own presence-only heartbeat back.

Sends no command that can arm, disarm, change flight mode, or move the
vehicle — see mavlink/connection.py and .claude/CLAUDE.md ("Safety Rules").

Ctrl+C to stop.
"""
import argparse
import sys
import time

from mavlink.connection import DEFAULT_BAUD, DEFAULT_DEVICE, FlightControllerLink


def run(args):
    fc_link = FlightControllerLink(device=args.device, baud=args.baud)
    try:
        fc_link.connect(wait_heartbeat_timeout_s=args.timeout)
    except TimeoutError as e:
        print(f"[mavlink_test] {e}")
        sys.exit(1)

    print("Connected. Streaming status — Ctrl+C to stop.\n")

    last_print = 0.0
    last_own_heartbeat = 0.0

    try:
        while True:
            fc_link.poll()

            now = time.monotonic()
            if now - last_own_heartbeat >= 1.0:
                fc_link.send_companion_heartbeat()
                last_own_heartbeat = now

            if now - last_print >= 1.0:
                status = fc_link.get_status()
                if status is not None:
                    print(
                        f"armed={str(status['armed']):5}  mode={status['mode']:10}  "
                        f"link_healthy={str(status['link_healthy']):5}  "
                        f"age={status['age_s']:.2f}s"
                    )
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        fc_link.close()
        print("\nConnection closed.")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--timeout", type=float, default=10.0,
                         help="Seconds to wait for the first heartbeat before giving up.")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
