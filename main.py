#!/usr/bin/env python3
"""
Cam_Test — AprilTag-following drone, companion-computer side.

Detects an AprilTag, estimates its pose in the ArduPilot body (FRD) frame,
and — if a flight controller is connected — monitors it over MAVLink
(heartbeat, armed state, flight mode). Serves an annotated MJPEG stream for
viewing over the network.

This script sends no command that can arm, disarm, change flight mode, or
move the vehicle. It is a monitoring/integration step, not a control loop.
See .claude/CLAUDE.md ("Control Architecture") for the GUIDED_NOGPS +
SET_ATTITUDE_TARGET design this is building toward and the safety gates
required before that lands.

Browser: http://<pi-ip>:<port>/stream
Ctrl+C to stop.
"""
import argparse
import time

import cv2
from picamera2 import Picamera2

from mavlink.connection import DEFAULT_BAUD, DEFAULT_DEVICE, FlightControllerLink
from safety.watchdog import Watchdog
from streaming.mjpeg_server import get_local_ip, start_mjpeg_server
from vision.apriltag_detector import AprilTagDetector

# ── Main capture + detection loop ─────────────────────────────────────────────


def run(args):
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={"size": tuple(args.resolution), "format": "RGB888"},
        controls={"FrameRate": 30.0},
        buffer_count=4,
    ))
    picam2.start()

    detector = AprilTagDetector()
    watchdog = Watchdog()

    fc_link = None
    if not args.no_mavlink:
        fc_link = FlightControllerLink(device=args.mavlink_device, baud=args.mavlink_baud)
        try:
            fc_link.connect()
            watchdog.note_mavlink_heartbeat()
        except TimeoutError as e:
            print(f"[main] {e}")
            print("[main] Continuing without a flight-controller link "
                  "(vision-only). Pass --no-mavlink to silence this.")
            fc_link = None

    httpd, stream_buffer = start_mjpeg_server(args.port)
    print(f"Stream live at http://{get_local_ip()}:{args.port}/stream")
    print("Waiting for AprilTags — Ctrl+C to stop.\n")

    last_print = 0.0

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)  # 180° — camera is upside down
            watchdog.note_frame()

            detections = detector.detect(frame)
            if detections:
                watchdog.note_detection()
                detector.annotate(frame, detections)

            if fc_link is not None:
                fc_link.poll()
                if fc_link.is_link_healthy():
                    watchdog.note_mavlink_heartbeat()

            now = time.monotonic()
            if now - last_print >= 0.1:  # 10 Hz terminal
                for det in detections:
                    print(
                        f"[ID={det['tag_id']:2d}]  "
                        f"dist={det['distance_m']:.3f}m  "
                        f"fwd={det['fwd_m']:+.3f}m  right={det['right_m']:+.3f}m  "
                        f"down={det['down_m']:+.3f}m  "
                        f"yaw={det['yaw_deg']:+.1f}deg  pitch={det['pitch_deg']:+.1f}deg  "
                        f"roll={det['roll_deg']:+.1f}deg"
                    )
                if fc_link is not None:
                    status = fc_link.get_status()
                    if status is not None:
                        print(
                            f"[FC] armed={status['armed']}  mode={status['mode']}  "
                            f"link_age={status['age_s']:.1f}s"
                        )
                wd = watchdog.status()
                if not watchdog.all_healthy():
                    print(f"[watchdog] camera_ok={wd['camera_ok']}  "
                          f"tag_ok={wd['tag_ok']}  mavlink_ok={wd['mavlink_ok']}")
                last_print = now

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                stream_buffer.push(buf.tobytes())

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        httpd.shutdown()
        if fc_link is not None:
            fc_link.close()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--resolution", type=int, nargs=2, default=(1280, 720))
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--mavlink-device", default=DEFAULT_DEVICE)
    parser.add_argument("--mavlink-baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--no-mavlink", action="store_true",
                         help="Run vision-only, without attempting a flight-controller link.")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
