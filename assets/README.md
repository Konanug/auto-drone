# Assets

Media used by the README and the documents under [`docs/`](../docs).

| Folder         | Contents                                                                 |
| -------------- | ------------------------------------------------------------------------ |
| `hero/`        | Single lead image or GIF for the top of the README                       |
| `drone/`       | Photographs of the assembled airframe, wiring, and installed components  |
| `cad/`         | SolidWorks renders — full-assembly and exploded views, and the explode clip |
| `diagrams/`    | System architecture, wiring, and power-distribution diagrams             |
| `demos/`       | Short flight and detection clips (GIF or MP4)                            |
| `screenshots/` | Annotated camera feed, SITL sessions, telemetry views                    |
| `results/`     | Plots generated from `--log` CSVs (convergence, detection rate, jitter)  |
| `print/`       | Printable AprilTag and calibration chessboard                            |

## Naming

Lowercase, hyphen-separated, descriptive of the content rather than the capture
device — `camera-mount-cad.png`, not `IMG_4932.png`.

## The explode clip

`cad/airframe-exploded.mp4` is the animated version of the exploded render.
Keep it small — this is a README, not a download:

```bash
ffmpeg -i raw-export.mp4 -an -vcodec libx264 -crf 26 -preset slow \
       -pix_fmt yuv420p -vf "scale=1280:-2" -movflags +faststart \
       cad/airframe-exploded.mp4
```

`-pix_fmt yuv420p` is not optional if it is to play in every browser, and
`+faststart` puts the index at the front so playback starts before the whole
file has downloaded. Aim for under 5 MB.

**Getting it to render on github.com.** A committed `.mp4` written with image
syntax produces a broken link, and `<video>` is stripped from README markdown.
One route works: drag the file into any issue or PR comment box on this repo,
let GitHub upload it, copy the `user-attachments/assets/...` URL it becomes,
cancel the comment, and paste that URL on its own line in the README. Commit the
file here as well — the upload URL is what plays on the web, the committed copy
is what a clone gets.

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
