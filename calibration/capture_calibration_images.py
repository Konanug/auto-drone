#!/usr/bin/env python3
"""
Captures chessboard images for camera calibration.

Print a standard OpenCV chessboard (9x6 internal corners is the default
below — a 10x7 squares board) and hold it in front of the camera at
varied distances, angles, and positions across the frame. Aim for 15-25
good captures.

Run:
    python3 calibration/capture_calibration_images.py

Controls:
    <space>  save current frame if a chessboard is detected
    q        quit

Output: calibration/images/frame_XXX.png
"""
import argparse
from pathlib import Path

import cv2
from picamera2 import Picamera2

CHESSBOARD_SIZE = (9, 6)  # internal corners (columns, rows)
OUT_DIR = Path(__file__).resolve().parent / "images"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resolution", type=int, nargs=2, default=(1280, 720))
    args = parser.parse_args()

    OUT_DIR.mkdir(exist_ok=True)

    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={"size": tuple(args.resolution), "format": "RGB888"},
    ))
    picam2.start()

    print(f"Chessboard: {CHESSBOARD_SIZE[0]}x{CHESSBOARD_SIZE[1]} internal corners")
    print("Press <space> to save a frame when corners are highlighted, q to quit.")

    saved = 0
    try:
        while True:
            frame = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

            found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE)
            display = frame_bgr.copy()
            if found:
                cv2.drawChessboardCorners(display, CHESSBOARD_SIZE, corners, found)

            cv2.putText(display, f"saved: {saved}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow("Calibration capture", display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" ") and found:
                path = OUT_DIR / f"frame_{saved:03d}.png"
                cv2.imwrite(str(path), frame_bgr)
                print(f"Saved {path}")
                saved += 1
    finally:
        picam2.stop()
        cv2.destroyAllWindows()

    print(f"\nCaptured {saved} frames to {OUT_DIR}")
    print("Next: python3 calibration/calibrate_camera.py")


if __name__ == "__main__":
    main()
