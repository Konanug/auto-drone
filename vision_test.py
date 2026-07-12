#!/usr/bin/env python3
"""
Cam_Test — vision-only validation harness.

Runs AprilTag detection + pose + velocity estimation with NO MAVLink
involvement at all — this is for validating the Raspberry Pi / camera side
of the pipeline (position accuracy, jitter, tag velocity while moving the
tag by hand) before any flight-controller integration exists.

Overlays a center crosshair and an offset line so you can see, live, how far
the tag is from frame-center — this is the signal a future follow-controller
would act on, so it's worth confirming it looks right before anything acts
on it.

Browser: http://<pi-ip>:<port>/stream
Ctrl+C to stop.

Suggested validation workflow:
  1. Print the tag (assets/apriltag_print.pdf) and mount it upright.
  2. Place it at known distances (0.5m, 1.0m, 2.0m, ...) directly in front
     of the camera and compare against the printed distance_m.
  3. Move the tag off-center and confirm the offset line/numbers point the
     right way (right/down offsets should match which way you moved it).
  4. Wave the tag side to side and watch v_right track the motion; hold it
     still and confirm velocity settles back near zero.
  5. Pass --log to record a CSV for offline analysis of jitter/accuracy.
"""
import argparse
import csv
import time

import cv2
from picamera2 import Picamera2

from streaming.mjpeg_server import get_local_ip, start_mjpeg_server
from vision.apriltag_detector import AprilTagDetector
from vision.velocity_estimator import VelocityEstimator

CSV_FIELDS = [
    "timestamp", "tag_id", "distance_m", "fwd_m", "right_m", "down_m",
    "yaw_deg", "pitch_deg", "roll_deg", "v_fwd_mps", "v_right_mps", "v_down_mps",
    "offset_x_px", "offset_y_px",
]


def tag_center_px(det):
    corners = det["corners"].reshape(-1, 2)
    return corners.mean(axis=0)


def draw_offset_overlay(frame, center_px, tag_px):
    cx, cy = int(center_px[0]), int(center_px[1])
    tx, ty = int(tag_px[0]), int(tag_px[1])
    cv2.drawMarker(frame, (cx, cy), (0, 200, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.line(frame, (cx, cy), (tx, ty), (0, 200, 255), 2)
    cv2.putText(frame, f"offset=({tx - cx:+d}, {ty - cy:+d})px", (tx + 10, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)


def run(args):
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={"size": tuple(args.resolution), "format": "RGB888"},
        controls={"FrameRate": 30.0},
        buffer_count=4,
    ))
    picam2.start()

    detector = AprilTagDetector()
    velocity = VelocityEstimator()

    httpd, stream_buffer = start_mjpeg_server(args.port)
    print(f"Stream live at http://{get_local_ip()}:{args.port}/stream")

    csv_writer = None
    csv_file = None
    if args.log:
        csv_file = open(args.log, "w", newline="")
        csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        csv_writer.writeheader()
        print(f"Logging to {args.log}")

    print("No MAVLink connection is made by this script — vision-only.")
    print("Waiting for AprilTags — Ctrl+C to stop.\n")

    frame_w, frame_h = args.resolution
    image_center_px = (frame_w / 2, frame_h / 2)
    last_print = 0.0
    last_seen_tag = None

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)  # 180° — camera is upside down

            detections = detector.detect(frame)
            now = time.monotonic()

            if detections:
                last_seen_tag = now
                detector.annotate(frame, detections)

            rows = []
            for det in detections:
                tx, ty = tag_center_px(det)
                offset_x = tx - image_center_px[0]
                offset_y = ty - image_center_px[1]
                draw_offset_overlay(frame, image_center_px, (tx, ty))

                v_fwd, v_right, v_down = velocity.update(
                    det["tag_id"], det["fwd_m"], det["right_m"], det["down_m"],
                    det["timestamp"],
                )

                rows.append({
                    "timestamp": det["timestamp"],
                    "tag_id": det["tag_id"],
                    "distance_m": det["distance_m"],
                    "fwd_m": det["fwd_m"], "right_m": det["right_m"], "down_m": det["down_m"],
                    "yaw_deg": det["yaw_deg"], "pitch_deg": det["pitch_deg"], "roll_deg": det["roll_deg"],
                    "v_fwd_mps": v_fwd, "v_right_mps": v_right, "v_down_mps": v_down,
                    "offset_x_px": offset_x, "offset_y_px": offset_y,
                })

            if csv_writer is not None:
                for row in rows:
                    csv_writer.writerow(row)

            if now - last_print >= 0.1:  # 10 Hz terminal
                if rows:
                    for r in rows:
                        print(
                            f"[ID={r['tag_id']:2d}]  dist={r['distance_m']:.3f}m  "
                            f"fwd={r['fwd_m']:+.3f}m right={r['right_m']:+.3f}m down={r['down_m']:+.3f}m  "
                            f"vfwd={r['v_fwd_mps']:+.2f} vright={r['v_right_mps']:+.2f} "
                            f"vdown={r['v_down_mps']:+.2f} m/s  "
                            f"px_offset=({r['offset_x_px']:+.0f},{r['offset_y_px']:+.0f})"
                        )
                elif last_seen_tag is None or (now - last_seen_tag) > 1.0:
                    print("[vision_test] no tag detected")
                last_print = now

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                stream_buffer.push(buf.tobytes())

    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        httpd.shutdown()
        if csv_file is not None:
            csv_file.close()
            print(f"\nSaved log to {args.log}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--resolution", type=int, nargs=2, default=(1280, 720))
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--log", default=None, help="Optional CSV path to log every detection to.")
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
