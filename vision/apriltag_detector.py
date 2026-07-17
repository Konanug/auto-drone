"""AprilTag detection + pose estimation, camera frame -> ArduPilot FRD body frame.

Uses OpenCV's ArUco module with the AprilTag 36h11 dictionary (true AprilTag
family, detected via OpenCV's built-in implementation — no separate apriltag
library needed).
"""
import time
from pathlib import Path

import cv2
import numpy as np

from vision.frame_transform import pose_to_ardupilot

TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36H11
TAG_SIZE_M = 0.168  # printed tag side length, metres — see tools/generate_tag.py

DEFAULT_INTRINSICS_PATH = (
    Path(__file__).resolve().parent.parent / "config" / "camera_intrinsics.npz"
)

# Fallback intrinsics estimated from Camera Module 3 datasheet specs at 1280x720
# (focal 4.74mm, sensor 6.45x3.63mm -> fx=941, fy=939, cx=640, cy=360).
# These are NOT measured. Distance/pose numbers from this fallback are
# approximate — run calibration/calibrate_camera.py before trusting them
# for anything closer to control than a print statement.
_FALLBACK_CAMERA_MATRIX = np.array([
    [941,   0, 640],
    [  0, 939, 360],
    [  0,   0,   1],
], dtype=np.float64)
_FALLBACK_DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)


def load_intrinsics(path=DEFAULT_INTRINSICS_PATH):
    path = Path(path)
    if path.exists():
        data = np.load(path)
        return data["camera_matrix"], data["dist_coeffs"]
    print(f"[apriltag_detector] WARNING: no calibration file at {path} — "
          f"using uncalibrated fallback intrinsics. "
          f"Run calibration/calibrate_camera.py to generate one.")
    return _FALLBACK_CAMERA_MATRIX, _FALLBACK_DIST_COEFFS


def _build_params():
    """Detector parameters tuned for a blurry, vibrating, high-gain image.

    OpenCV's defaults leave cornerRefinementMethod at NONE, so corners land on
    whole pixels — that is free pose jitter, and it costs almost nothing to fix.
    The rest loosen the detector's assumptions about crisp geometry, because a
    vibrating airframe does not produce crisp geometry.

    NOTE what tuning CANNOT do: if motion blur exceeds the tag's BIT-CELL size
    (tag_px / 8), the bits are physically destroyed and no parameter recovers
    them. Benchmarked: at 2 m the tag is ~79 px, so cells are ~10 px, and a
    17 px blur gives 0% detection on every configuration tried. Fix blur with a
    shorter shutter (see vision/camera.py), not with detector settings.
    """
    p = cv2.aruco.DetectorParameters()
    # Subpixel corners: the single cheapest accuracy win available.
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.cornerRefinementWinSize = 5
    # Tolerate rounded/smeared corners rather than demanding sharp polygons.
    p.polygonalApproxAccuracyRate = 0.06
    # See the tag from further away.
    p.minMarkerPerimeterRate = 0.02
    # Accept more bit errors — a noisy high-gain frame is still decodable.
    p.errorCorrectionRate = 0.8
    return p


class AprilTagDetector:
    """AprilTag detection + pose.

    detect_scale exists because detection cost scales with PIXEL COUNT while
    pose accuracy depends only on CORNER precision. On this Pi, detecting at
    full 1280x720 costs ~71 ms — more than double the 33 ms budget for 30 fps,
    which starved the whole control loop down to ~8 fps. So we detect on a
    downscaled image (fast), then refine the corners back on the FULL-resolution
    image with cornerSubPix (accurate, and cheap because it only looks at small
    windows around four points). Pose is always computed with the full-res
    intrinsics against full-res corner coordinates.
    """

    def __init__(self, tag_size_m=TAG_SIZE_M, intrinsics_path=DEFAULT_INTRINSICS_PATH,
                 detect_scale=0.5):
        self.tag_size_m = tag_size_m
        self.camera_matrix, self.dist_coeffs = load_intrinsics(intrinsics_path)
        self.detect_scale = float(detect_scale)
        dictionary = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
        self.detector = cv2.aruco.ArucoDetector(dictionary, _build_params())
        self._subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                                 30, 0.01)

    def detect(self, frame_bgr):
        """Detect tags in a BGR frame.

        Returns a list of dicts (one per tag), each with pose in both the
        camera frame and the ArduPilot FRD body frame. Empty list if no tag
        is visible. Corner coordinates are always in FULL-resolution pixels,
        regardless of detect_scale.
        """
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        s = self.detect_scale
        if s < 0.999:
            small = cv2.resize(gray, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            corners, ids, _ = self.detector.detectMarkers(small)
            if ids is None:
                return []
            # Back to full-res coordinates, then recover the precision we gave
            # up by refining against the full-res image.
            corners = [c / s for c in corners]
            for c in corners:
                cv2.cornerSubPix(gray, c.reshape(-1, 1, 2), (5, 5), (-1, -1),
                                 self._subpix_criteria)
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)
            if ids is None:
                return []

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.tag_size_m, self.camera_matrix, self.dist_coeffs
        )

        results = []
        for i, tag_id in enumerate(ids.flatten()):
            tvec = tvecs[i][0]
            rvec = rvecs[i][0]
            fwd, right, down, yaw, pitch, roll = pose_to_ardupilot(rvec, tvec)
            results.append({
                "tag_id": int(tag_id),
                "corners": corners[i],
                "rvec": rvec,
                "tvec": tvec,
                "distance_m": float(np.linalg.norm(tvec)),
                "fwd_m": fwd,
                "right_m": right,
                "down_m": down,
                "yaw_deg": yaw,
                "pitch_deg": pitch,
                "roll_deg": roll,
                "timestamp": time.monotonic(),
            })
        return results

    def annotate(self, frame_bgr, detections):
        """Draws tag outlines and pose axes onto frame_bgr in place."""
        for det in detections:
            corners = det["corners"].reshape(-1, 2).astype(np.int32)
            cv2.polylines(frame_bgr, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
            label_pos = tuple(corners[0])
            cv2.putText(frame_bgr, f"id={det['tag_id']}", label_pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.drawFrameAxes(
                frame_bgr, self.camera_matrix, self.dist_coeffs,
                det["rvec"], det["tvec"], self.tag_size_m * 0.5
            )
        return frame_bgr
