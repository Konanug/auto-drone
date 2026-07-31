# Assets

Media used by the README and the documents under [`docs/`](../docs).

| Folder         | Contents                                                                 |
| -------------- | ------------------------------------------------------------------------ |
| `hero/`        | Single lead image or GIF for the top of the README                       |
| `drone/`       | Photographs of the assembled airframe, wiring, and installed components  |
| `cad/`         | SolidWorks renders and exploded views of the custom mounts               |
| `diagrams/`    | System architecture, wiring, and power-distribution diagrams             |
| `demos/`       | Short flight and detection clips (GIF or MP4)                            |
| `screenshots/` | Annotated camera feed, SITL sessions, telemetry views                    |
| `results/`     | Plots generated from `--log` CSVs (convergence, detection rate, jitter)  |
| `print/`       | Printable AprilTag and calibration chessboard                            |

## Naming

Lowercase, hyphen-separated, descriptive of the content rather than the capture
device — `camera-mount-cad.png`, not `IMG_4932.png`.

## Print targets

`print/apriltag-36h11-id0.png` and `print/apriltag-print.pdf` are the tag the
detector expects: family 36h11, ID 0, **168 mm side length including the white
quiet-zone border**. Print at exactly that size and do not crop the border —
pose estimation scales directly with the physical tag size set in
[`vision/apriltag_detector.py`](../vision/apriltag_detector.py). Regenerate with
`python3 tools/generate_tag.py`.

`print/calibration-chessboard.png` is the 9×6 internal-corner board used by
[`calibration/`](../calibration). Measure one printed square and set
`SQUARE_SIZE_M` in `calibrate_camera.py` before calibrating.
