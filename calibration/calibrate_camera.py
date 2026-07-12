#!/usr/bin/env python3
"""
Runs OpenCV camera calibration over chessboard images captured by
capture_calibration_images.py and saves the result for vision/apriltag_detector.py
to load.

Run:
    python3 calibration/calibrate_camera.py

Output: config/camera_intrinsics.npz  (camera_matrix, dist_coeffs)
"""
from pathlib import Path

import cv2
import numpy as np

CHESSBOARD_SIZE = (9, 6)   # internal corners (columns, rows) — must match capture script
SQUARE_SIZE_M = 0.025      # physical size of one chessboard square, metres — measure yours

IMAGES_DIR = Path(__file__).resolve().parent / "images"
OUT_PATH = Path(__file__).resolve().parent.parent / "config" / "camera_intrinsics.npz"


def main():
    images = sorted(IMAGES_DIR.glob("*.png"))
    if len(images) < 10:
        print(f"Only found {len(images)} images in {IMAGES_DIR} — "
              f"capture at least 10-15 varied views first "
              f"(see capture_calibration_images.py).")
        return

    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_M

    obj_points = []
    img_points = []
    image_size = None

    for path in images:
        img = cv2.imread(str(path))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE)
        if not found:
            print(f"  skip {path.name} — no chessboard found")
            continue

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        obj_points.append(objp)
        img_points.append(corners)

    if len(obj_points) < 10:
        print(f"Only {len(obj_points)} usable images — need at least 10. "
              f"Capture more varied views.")
        return

    print(f"Calibrating from {len(obj_points)} images...")
    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None
    )

    print(f"RMS reprojection error: {rms:.4f} px (lower is better; <1.0 is good)")
    print(f"Camera matrix:\n{camera_matrix}")
    print(f"Distortion coefficients:\n{dist_coeffs}")

    OUT_PATH.parent.mkdir(exist_ok=True)
    np.savez(OUT_PATH, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    print(f"\nSaved to {OUT_PATH}")
    print("vision/apriltag_detector.py will pick this up automatically.")


if __name__ == "__main__":
    main()
