#!/usr/bin/env python3
"""
motor_test_on_tag.py — bench validation of Pi -> FC command authority.

The first time an AprilTag is (re)acquired, sends one ArduPilot
MAV_CMD_DO_MOTOR_TEST command to spin a single motor at low throttle for a
few seconds. While the tag stays continuously in frame, nothing further is
sent; losing and re-detecting the tag arms the trigger again exactly once.

This is the first script in this project with command authority over the
vehicle. MAV_CMD_DO_MOTOR_TEST is ArduPilot's own bench-test command: it only
runs while disarmed, spins exactly one motor for a bounded duration, and
needs no flight-mode change or arming. It is NOT the same code path as
GUIDED_NOGPS + SET_ATTITUDE_TARGET (continuous real-time attitude
streaming) — that is a separate, larger step gated on the roadmap items in
.claude/CLAUDE.md ("Control Architecture"): verified transmitter mode-switch
override, a rebuilt watchdog, and rate limiting. Don't grow this script into
that without going through those gates first.

MOTOR NUMBERING (read this before changing --motor):
  MOTOR_TEST_ORDER_DEFAULT numbers motors by TEST SEQUENCE — clockwise from
  front-right — NOT by ESC output channel. For a quad-X:
      1 = front-right   2 = back-right   3 = back-left   4 = front-left
  These match Mission Planner's "Test motor A/B/C/D" buttons (A=1..D=4), NOT
  the "Motor Number" output labels shown next to them. Mission Planner may
  label the back-right motor's output as "Motor Number 4", but the value to
  pass here for back-right is 2 (Test B). Confirm on the Motor Test tab first.

SAFETY:
- Propellers must be removed before running this.
- ArduPilot's own motor-test interlocks (disarmed state, safety switch)
  still apply; this script does not and cannot bypass them.

The MJPEG stream (same viewer as vision_test.py) is for visually confirming
tag detection and framing only — it plays no part in the trigger logic.

Browser: http://<pi-ip>:<port>/stream
Ctrl+C to stop.
"""
import argparse
import time

import cv2
from picamera2 import Picamera2
from pymavlink import mavutil

from mavlink.connection import DEFAULT_BAUD, DEFAULT_DEVICE, FlightControllerLink
from streaming.mjpeg_server import get_local_ip, start_mjpeg_server
from vision.apriltag_detector import AprilTagDetector

TAG_LOSS_GRACE_S = 0.3  # tag must be absent this long before a re-detection counts as new


def send_motor_test(fc_link, motor_number, throttle_pct, duration_s):
    print(f"[motor_test] Tag (re)acquired — spinning motor {motor_number} at "
          f"{throttle_pct:.0f}% for {duration_s:.1f}s")
    conn = fc_link.raw_connection
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,  # confirmation
        motor_number,
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
        throttle_pct,
        duration_s,
        0,  # motor count — 0/1 means just this one motor
        mavutil.mavlink.MOTOR_TEST_ORDER_DEFAULT,
        0,  # empty
    )


def run(args):
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={"size": (1280, 720), "format": "RGB888"},
        controls={"FrameRate": 30.0},
        buffer_count=4,
    ))
    picam2.start()

    detector = AprilTagDetector()

    fc_link = FlightControllerLink(device=args.mavlink_device, baud=args.mavlink_baud)
    fc_link.connect()

    httpd, stream_buffer = start_mjpeg_server(args.port)
    print(f"Stream live at http://{get_local_ip()}:{args.port}/stream (view-only, not used for triggering)")

    tag_present = False
    triggered_this_presence = False
    last_seen_time = 0.0

    print("Watching for AprilTag — Ctrl+C to stop.\n")

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)  # 180 deg - camera is upside down
            fc_link.poll()

            detections = detector.detect(frame)
            now = time.monotonic()

            if detections:
                detector.annotate(frame, detections)
                if not tag_present and (now - last_seen_time) > TAG_LOSS_GRACE_S:
                    triggered_this_presence = False  # tag is genuinely new, not a dropped frame
                tag_present = True
                last_seen_time = now

                if not triggered_this_presence:
                    send_motor_test(fc_link, args.motor, args.throttle, args.duration)
                    triggered_this_presence = True
            elif tag_present and (now - last_seen_time) > TAG_LOSS_GRACE_S:
                tag_present = False

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                stream_buffer.push(buf.tobytes())

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        httpd.shutdown()
        fc_link.close()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motor", type=int, default=2,
                         help="Motor test-SEQUENCE position (not the ESC output number). "
                              "With MOTOR_TEST_ORDER_DEFAULT, ArduPilot numbers motors "
                              "clockwise from front-right: 1=front-right, 2=back-right, "
                              "3=back-left, 4=front-left. These match Mission Planner's "
                              "Test A/B/C/D buttons, NOT the 'Motor Number' output labels. "
                              "Default 2 = back-right (Test B).")
    parser.add_argument("--throttle", type=float, default=4.0, help="Percent throttle.")
    parser.add_argument("--duration", type=float, default=3.0, help="Seconds to spin.")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--mavlink-device", default=DEFAULT_DEVICE)
    parser.add_argument("--mavlink-baud", type=int, default=DEFAULT_BAUD)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
