#!/usr/bin/env python3
"""
Captures chessboard images for camera calibration — HEADLESS.

The old version used cv2.imshow, which needs an X display and fails over plain
SSH ("could not connect to display"). This one streams the live view to the
browser like every other tool in the project, and AUTO-CAPTURES frames as you
move the board — no window, no keypress.

Print a standard OpenCV chessboard (default 9x6 INTERNAL corners = a 10x7
squares board). Watch the stream, and slowly move the board around: near/far,
tilted, and into every corner of the frame. It saves a frame automatically each
time the board reaches a spot it has not covered yet. Aim for the target count.

IMPORTANT — focus:
  Camera intrinsics depend on lens position, so you MUST calibrate at the SAME
  fixed focus you operate at. This locks focus at --focus-m (default 1.0 m, to
  match vision/camera.py). Hold the board around that distance so it stays sharp;
  boards far from the focus distance will blur and won't be detected.

IMPORTANT — square size:
  Measure one chessboard square with a ruler and set SQUARE_SIZE_M in
  calibrate_camera.py before running that. It does not affect capture, but the
  calibration is wrong without it.

Run:
    python3 calibration/capture_calibration_images.py            # 1 m, 20 frames
    python3 calibration/capture_calibration_images.py --count 25 --focus-m 1.0

Then:
    python3 calibration/calibrate_camera.py

Output: calibration/images/frame_XXX.png
Browser: http://<pi-ip>:8080/stream   (Ctrl+C to stop early)
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# This script lives in calibration/, so the project root is not on sys.path
# when run as `python3 calibration/capture_calibration_images.py`.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from streaming.mjpeg_server import get_local_ip, start_mjpeg_server  # noqa: E402
from vision import camera as cam  # noqa: E402

CHESSBOARD_SIZE = (9, 6)  # internal corners (columns, rows)
OUT_DIR = Path(__file__).resolve().parent / "images"

# Auto-capture gates: the board must land far enough (in px) from every frame
# already saved, so the captures spread across the image instead of piling up.
MIN_CENTROID_MOVE_PX = 120
MIN_SECONDS_BETWEEN = 0.8


def _settings(resolution, focus_m):
    """Namespace for vision.camera: AUTO exposure (this is a static, well-lit
    bench task — no vibration to freeze), focus LOCKED at the operating
    distance, autofocus off."""
    ns = argparse.Namespace()
    ns.resolution = resolution
    ns.fps = 30.0
    ns.exposure_us = 2000
    ns.gain = 8.0
    ns.focus_m = focus_m
    ns.auto_exposure = True       # let the sensor expose the board properly
    ns.autofocus = False          # focus MUST stay fixed for valid intrinsics
    return ns


def centroid(corners):
    return corners.reshape(-1, 2).mean(axis=0)


def is_new_pose(c, saved_centroids):
    return all(np.linalg.norm(c - s) > MIN_CENTROID_MOVE_PX for s in saved_centroids)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--resolution", type=int, nargs=2, default=(1280, 720))
    parser.add_argument("--focus-m", type=float, default=1.0,
                        help="Lock focus at this distance (m). MUST match the "
                             "operating focus in vision/camera.py. Default 1.0.")
    parser.add_argument("--count", type=int, default=20,
                        help="Target number of frames to capture (default 20).")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    OUT_DIR.mkdir(exist_ok=True)
    # Start clean: mixing boards from different focus/sessions corrupts calibration.
    old = list(OUT_DIR.glob("frame_*.png"))
    for f in old:
        f.unlink()
    if old:
        print(f"Cleared {len(old)} old frame(s) from {OUT_DIR}")

    picam2 = cam.open_camera(_settings(tuple(args.resolution), args.focus_m))
    httpd, stream = start_mjpeg_server(args.port)
    print(f"Stream: http://{get_local_ip()}:{args.port}/stream")
    print(f"Chessboard: {CHESSBOARD_SIZE[0]}x{CHESSBOARD_SIZE[1]} internal corners")
    print(f"Move the board slowly around the frame (near/far, tilted). "
          f"Auto-saving {args.count} varied views.\n")

    # Faster than findChessboardCorners for the live loop; refine only on save.
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE

    saved = 0
    saved_centroids = []
    last_save = 0.0
    last_stream = 0.0

    try:
        while saved < args.count:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            # 180deg flip — the camera is mounted upside down and EVERY
            # operational script flips before detecting. Calibration must run on
            # the same flipped frames, or the principal point (and tangential
            # distortion sign) won't match how the intrinsics are actually used.
            frame = cv2.flip(frame, -1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, flags)
            display = frame.copy()
            now = time.monotonic()

            status = "no board"
            if found:
                cv2.drawChessboardCorners(display, CHESSBOARD_SIZE, corners, found)
                c = centroid(corners)
                fresh = is_new_pose(c, saved_centroids)
                if fresh and (now - last_save) > MIN_SECONDS_BETWEEN:
                    path = OUT_DIR / f"frame_{saved:03d}.png"
                    cv2.imwrite(str(path), frame)   # save the CLEAN frame
                    saved_centroids.append(c)
                    saved += 1
                    last_save = now
                    print(f"  saved {saved}/{args.count}  ({path.name})")
                    status = "SAVED"
                else:
                    status = "board found — move to a NEW spot" if not fresh \
                        else "board found — hold steady"

            # progress overlay + where we've already captured
            for s in saved_centroids:
                cv2.circle(display, (int(s[0]), int(s[1])), 8, (0, 180, 0), -1)
            cv2.putText(display, f"saved {saved}/{args.count}  |  {status}",
                        (10, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                        (0, 255, 0) if found else (0, 0, 255), 2)

            if now - last_stream >= 1 / 12.0:
                ok, buf = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    stream.push(buf.tobytes())
                last_stream = now
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        httpd.shutdown()

    print(f"\nCaptured {saved} frames to {OUT_DIR}")
    if saved >= 10:
        print("Next: measure a square, set SQUARE_SIZE_M in calibrate_camera.py, "
              "then run: python3 calibration/calibrate_camera.py")
    else:
        print("Fewer than 10 frames — run again and cover more of the frame.")


if __name__ == "__main__":
    main()
