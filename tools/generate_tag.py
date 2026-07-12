#!/usr/bin/env python3
"""
Generates a printable AprilTag (family: 36h11, ID 0).

Run:
    python3 tools/generate_tag.py

Output: assets/apriltag_36h11_id0.png

Printing instructions:
  - Open the PNG and print at exactly 16.8 cm × 16.8 cm (including white border).
  - The white border is part of the tag spec — do not crop it.
  - Use plain paper; lamination is fine but avoid glossy reflections.
"""

from pathlib import Path

import cv2

FAMILY    = cv2.aruco.DICT_APRILTAG_36H11
TAG_ID    = 0
PX_SIZE   = 1000   # pixel size of the black pattern area
BORDER_PX = 100    # white quiet-zone border (required by spec)
OUT_FILE  = Path(__file__).resolve().parent.parent / "assets" / "apriltag_36h11_id0.png"

dictionary = cv2.aruco.getPredefinedDictionary(FAMILY)
tag        = cv2.aruco.generateImageMarker(dictionary, TAG_ID, PX_SIZE)

tag_with_border = cv2.copyMakeBorder(
    tag, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
    cv2.BORDER_CONSTANT, value=255,
)

cv2.imwrite(str(OUT_FILE), tag_with_border)
total_px = PX_SIZE + 2 * BORDER_PX
print(f"Saved {OUT_FILE}  ({total_px}×{total_px} px)")
print(f"Print at 16.8 cm × 16.8 cm for accurate pose estimation.")
print(f"Tag family: 36h11  |  ID: {TAG_ID}")
