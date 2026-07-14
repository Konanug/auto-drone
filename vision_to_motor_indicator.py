#!/usr/bin/env python3
"""
vision_to_motor_indicator.py — visible proof that vision directives reach the FC.

Maps AprilTag offset thresholds to individual motor spins, so you can SEE the
flight controller acting on vision-derived directives in real time:

    too far from tag   -> motor 1 spins   (front-right)
    too far left       -> motor 2 spins   (back-right)
    too far right      -> motor 3 spins   (back-left)
    too close to tag   -> motor 4 spins   (front-left)

All at a fixed low throttle (default 4%). This is a DETECTION INDICATOR, not a
controller — nothing here closes a control loop or moves the vehicle in a
meaningful way; each spin is ArduPilot's own bench MAV_CMD_DO_MOTOR_TEST, which
only runs while disarmed and spins one motor for a bounded time.

Motor numbers are MOTOR_TEST_ORDER_DEFAULT test-SEQUENCE positions (= Mission
Planner's Test A/B/C/D buttons), NOT ESC output-channel labels. See
motor_test_on_tag.py for the full explanation of that distinction.

CONCURRENCY NOTE: ArduPilot runs only ONE motor test at a time — a new
DO_MOTOR_TEST overrides the previous. When two conditions are active at once
(one distance + one lateral), this script round-robins between their motors
fast enough that both visibly spin; they are not truly simultaneous.

SAFETY:
- Propellers must be removed before running this.
- ArduPilot's own motor-test interlocks (disarmed, safety switch) still apply.
- Use --dry-run to verify the vision->condition logic with NO motor commands
  sent at all, before letting it spin anything.

The MJPEG stream (same viewer as vision_test.py) is for visually confirming tag
framing only — it plays no part in the logic.

Browser: http://<pi-ip>:<port>/stream
Ctrl+C to stop.
"""
import argparse
import time

import cv2
import numpy as np
from pymavlink import mavutil

from mavlink.connection import DEFAULT_BAUD, DEFAULT_DEVICE, FlightControllerLink
from streaming.mjpeg_server import get_local_ip, start_mjpeg_server
from vision import camera as cam
from vision import preprocess as pre
from vision.apriltag_detector import AprilTagDetector

# Condition name -> motor test-sequence number (see module docstring)
STREAM_INTERVAL_S = 1 / 12.0   # debug stream at ~12 fps, not 30
MOTOR_FOR = {"far": 1, "left": 2, "right": 3, "close": 4}

MOTOR_TEST_TIMEOUT_S = 3.0   # each spin self-expires after this on the FC (safety cap if the loop hangs/dies)
REASSERT_S = 2.0             # single active motor: re-assert this often (< timeout => seamless, continuous spin)
MULTIPLEX_S = 0.3            # 2+ active motors: round-robin this fast so both visibly spin


def send_motor_test(conn, motor_number, throttle_pct, timeout_s):
    conn.mav.command_long_send(
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,  # confirmation
        motor_number,
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,
        throttle_pct,
        timeout_s,
        0,  # motor count — 0/1 means just this one motor
        mavutil.mavlink.MOTOR_TEST_ORDER_DEFAULT,
        0,  # empty
    )


def tag_center_px(det):
    return det["corners"].reshape(-1, 2).mean(axis=0)


def draw_legend(frame, title, rows):
    """Translucent titled legend box pinned to the top-right corner."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale, thick, line_h, pad = 0.5, 1, 20, 8
    texts = [title] + [f"{k:<5}: {v}" for k, v in rows]
    text_w = max(cv2.getTextSize(t, font, scale, thick)[0][0] for t in texts)
    x2, y1 = frame.shape[1] - 10, 10
    x1, y2 = x2 - (text_w + 2 * pad), y1 + (line_h * len(texts) + 2 * pad)

    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 1)

    y = y1 + pad + 12
    for i, t in enumerate(texts):
        color = (0, 200, 255) if i == 0 else (255, 255, 255)
        cv2.putText(frame, t, (x1 + pad, y), font, scale, color, thick, cv2.LINE_AA)
        y += line_h


def draw_verification_overlay(frame, det, image_center_px):
    """Vector from the screen origin (frame center) to the tag center, plus a
    top-right legend of distance and pitch/yaw/roll relative to straight-on.
    Purely for visual verification — affects no logic."""
    cx, cy = int(image_center_px[0]), int(image_center_px[1])
    tx, ty = tag_center_px(det)
    tx, ty = int(tx), int(ty)
    dx, dy = tx - cx, ty - cy
    mag = (dx * dx + dy * dy) ** 0.5

    cv2.arrowedLine(frame, (cx, cy), (tx, ty), (0, 200, 255), 2, tipLength=0.04)
    cv2.putText(frame, f"({dx:+d},{dy:+d}) {mag:.0f}px", (tx + 8, ty - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)

    draw_legend(frame, "tag pose (straight-on = 0)", [
        ("dist", f"{det['distance_m']:.2f} m"),
        ("offX", f"{dx:+d} px"),
        ("offY", f"{dy:+d} px"),
        ("pitch", f"{det['pitch_deg']:+6.1f} deg"),
        ("yaw", f"{det['yaw_deg']:+6.1f} deg"),
        ("roll", f"{det['roll_deg']:+6.1f} deg"),
    ])


def active_conditions(det, image_center_px, args):
    """Return the set of condition names currently triggered for one detection."""
    conds = []
    # distance axis (near/far are mutually exclusive)
    if det["distance_m"] > args.far:
        conds.append("far")
    elif det["distance_m"] < args.close:
        conds.append("close")
    # lateral axis, using pixel offset (calibration-free, robust)
    offset_x = tag_center_px(det)[0] - image_center_px[0]
    if offset_x < -args.lateral_px:
        conds.append("left")
    elif offset_x > args.lateral_px:
        conds.append("right")
    return conds


def run(args):
    picam2 = cam.open_camera(args)

    detector = AprilTagDetector(detect_scale=args.detect_scale)

    prep = pre.Preprocessor(args)

    print(f"[vision] detect_scale={args.detect_scale}  preprocessing: {prep.describe()}")

    fc_link = None
    conn = None
    if not args.dry_run:
        fc_link = FlightControllerLink(device=args.mavlink_device, baud=args.mavlink_baud)
        fc_link.connect()
        conn = fc_link.raw_connection
    else:
        print("[dry-run] No MAVLink connection — logging decisions only, no motor commands.")

    httpd, stream_buffer = start_mjpeg_server(args.port)
    print(f"Stream live at http://{get_local_ip()}:{args.port}/stream (view-only)")
    print("Watching for AprilTag — Ctrl+C to stop.\n")

    frame_w, frame_h = args.resolution
    image_center_px = (frame_w / 2, frame_h / 2)
    last_print = 0.0
    last_stream = 0.0
    last_refresh = 0.0
    rr_index = 0       # round-robin cursor across active motors
    last_motor = None  # motor currently commanded to spin (None = stopped)

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)  # 180 deg - camera is upside down
            frame = prep.apply(frame)    # what we detect on IS what we stream
            if fc_link is not None:
                fc_link.poll()

            detections = detector.detect(frame)
            now = time.monotonic()

            # Origin crosshair at frame center — reference point for the vector,
            # drawn every frame so it's visible even when no tag is present.
            cv2.drawMarker(frame, (int(image_center_px[0]), int(image_center_px[1])),
                           (0, 255, 255), cv2.MARKER_CROSS, 22, 2)

            conds = []
            if detections:
                detector.annotate(frame, detections)
                draw_verification_overlay(frame, detections[0], image_center_px)
                # act on the first detected tag
                conds = active_conditions(detections[0], image_center_px, args)

            active_motors = [MOTOR_FOR[c] for c in conds]

            # Keep the active motor(s) spinning. A single condition re-asserts
            # slowly (well within the timeout) so it spins continuously and
            # seamlessly at the set throttle for as long as the tag stays in that
            # position; two active conditions round-robin fast so both visibly
            # spin, since ArduPilot runs only one motor test at a time.
            if active_motors:
                interval = MULTIPLEX_S if len(active_motors) > 1 else REASSERT_S
                if (now - last_refresh) >= interval:
                    motor = active_motors[rr_index % len(active_motors)]
                    if conn is not None:
                        send_motor_test(conn, motor, args.throttle, MOTOR_TEST_TIMEOUT_S)
                    last_motor = motor
                    rr_index += 1
                    last_refresh = now
            elif last_motor is not None:
                # Condition cleared — stop promptly instead of waiting for the
                # timeout to lapse. A 0% test cancels the in-progress spin.
                if conn is not None:
                    send_motor_test(conn, last_motor, 0.0, 0.0)
                last_motor = None

            # On-screen + terminal status
            cv2.putText(frame, f"active: {','.join(conds) if conds else 'none'}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

            if now - last_print >= 0.1:  # 10 Hz
                if detections:
                    det = detections[0]
                    offx = tag_center_px(det)[0] - image_center_px[0]
                    motor_str = ",".join(str(MOTOR_FOR[c]) for c in conds) or "-"
                    print(f"dist={det['distance_m']:.2f}m  offset_x={offx:+.0f}px  "
                          f"conds={conds}  motor(s)={motor_str}")
                else:
                    print("[indicator] no tag detected")
                last_print = now

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
        # Cancel any in-progress spin promptly (otherwise it self-expires within
        # MOTOR_TEST_TIMEOUT_S). A 0% test replaces the active one.
        if conn is not None and last_motor is not None:
            send_motor_test(conn, last_motor, 0.0, 0.0)
        picam2.stop()
        httpd.shutdown()
        if fc_link is not None:
            fc_link.close()


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    cam.add_camera_args(parser)
    pre.add_preprocess_args(parser)
    parser.add_argument("--detect-scale", type=float, default=0.5,
                        help="Detect on a downscaled image (corners are still "
                             "refined at full res, so accuracy is kept). 1.0 costs "
                             "71 ms/frame on the Pi and starves the loop to ~8 fps; "
                             "0.5 costs 17 ms. Default 0.5.")
    parser.add_argument("--far", type=float, default=1.5,
                         help="distance_m above this = too far (motor 1). Default 1.5.")
    parser.add_argument("--close", type=float, default=0.7,
                         help="distance_m below this = too close (motor 4). Default 0.7.")
    parser.add_argument("--lateral-px", type=float, default=120.0,
                         help="tag center this many px off-center triggers left/right "
                              "(motor 2/3). Default 120.")
    parser.add_argument("--throttle", type=float, default=4.0, help="Percent throttle.")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--dry-run", action="store_true",
                         help="Log condition/motor decisions without connecting to the FC "
                              "or sending any motor command.")
    parser.add_argument("--mavlink-device", default=DEFAULT_DEVICE)
    parser.add_argument("--mavlink-baud", type=int, default=DEFAULT_BAUD)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
