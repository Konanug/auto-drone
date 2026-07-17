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
streaming) — that is a separate, larger step, gated on a verified transmitter
mode-switch override, a rebuilt watchdog, and rate limiting. Don't grow this
script into that without going through those gates first.

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
from pymavlink import mavutil

from mavlink.connection import DEFAULT_BAUD, DEFAULT_DEVICE, FlightControllerLink
from streaming.mjpeg_server import get_local_ip, start_mjpeg_server
from vision import camera as cam
from vision import preprocess as pre
from vision.apriltag_detector import AprilTagDetector

STREAM_INTERVAL_S = 1 / 12.0   # debug stream at ~12 fps, not 30
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
    picam2 = cam.open_camera(args)

    detector = AprilTagDetector(detect_scale=args.detect_scale)

    prep = pre.Preprocessor(args)

    print(f"[vision] detect_scale={args.detect_scale}  preprocessing: {prep.describe()}")

    fc_link = FlightControllerLink(device=args.mavlink_device, baud=args.mavlink_baud)
    fc_link.connect()

    httpd, stream_buffer = start_mjpeg_server(args.port)
    print(f"Stream live at http://{get_local_ip()}:{args.port}/stream (view-only, not used for triggering)")

    tag_present = False
    triggered_this_presence = False
    last_seen_time = 0.0

    print("Watching for AprilTag — Ctrl+C to stop.\n")

    last_stream = 0.0
    try:
        while True:
            frame = picam2.capture_array()   # 180° flip happens in hardware
            frame = prep.apply(frame)    # what we detect on IS what we stream
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

            # Throttle the stream encode: it costs ~13 ms of a 33 ms frame budget
            # and a human does not need 30 fps. Detection still runs every frame.
            if now - last_stream >= STREAM_INTERVAL_S:
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    stream_buffer.push(buf.tobytes())
                last_stream = now

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        httpd.shutdown()
        fc_link.close()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    cam.add_camera_args(parser)
    pre.add_preprocess_args(parser)
    parser.add_argument("--detect-scale", type=float, default=0.5,
                        help="Detect on a downscaled image (corners are still "
                             "refined at full res, so accuracy is kept). 1.0 costs "
                             "71 ms/frame on the Pi and starves the loop to ~8 fps; "
                             "0.5 costs 17 ms. Default 0.5.")
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
