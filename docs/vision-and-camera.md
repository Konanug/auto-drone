# Vision and camera

Most of the engineering here is not in the detector. It is in getting the camera to
produce a frame worth detecting on, from a platform that vibrates.

## The problem

Motors on this airframe vibrate at roughly 65–100 Hz, a 10–15 ms period. Picamera2's
default 20 ms exposure — pushed longer still by auto-exposure indoors — integrates across
one to two full vibration cycles, smearing the tag's edges by the entire vibration
amplitude.

There is a hard limit on what image processing can recover. If motion blur exceeds the
tag's **bit-cell size** (`tag_px / 8`), the bits are physically destroyed and no filter or
detector parameter brings them back. Benchmarked against synthetically degraded tags: at
2 m the tag is about 79 px, so its cells are about 10 px, and a 17 px blur gave 0%
detection on every configuration tried.

Blur is fixed with a shorter shutter, not with post-processing.

## Camera configuration

[`vision/camera.py`](../vision/camera.py) goes fully manual:

| Setting             | Value           | Reason                                                        |
| ------------------- | --------------- | ------------------------------------------------------------- |
| Exposure            | 2 ms            | At ~100 Hz this captures ~1/5 of a cycle instead of two whole ones |
| Analogue gain       | 8.0             | Pays for the light the short shutter gives up                 |
| Focus               | Fixed, 1.0 m    | Continuous autofocus hunts on a vibrating platform, and every hunt is a blurred frame |
| Noise reduction     | Off             | Denoising rounds off exactly the high-contrast corners the detector keys on |
| Sharpness           | 2.0             | Preserves edge contrast                                        |
| Sensor mode         | 1536×864        | Fastest readout available on the IMX708                       |
| Frame rate          | 60 fps, `queue=False` | Halves the average age of each returned frame           |

AprilTag detection tolerates **noise** far better than it tolerates **blur**. A grainy
sharp frame decodes; a clean smeared one does not. That trade is the whole basis of the
configuration above.

Focus distance must match the controller's hover setpoint (`--focus-m` and `--distance`),
or the tag sits outside the depth of field.

## Rolling shutter

A short exposure freezes motion blur but does **not** freeze the row-by-row readout skew —
the "jello" effect. That skew is proportional to the sensor mode's readout time, which
software can only minimise, never eliminate.

| Sensor mode  | Readout time |
| ------------ | ------------ |
| 1536×864     | ~8.3 ms      |
| 2304×1296    | ~17.9 ms     |
| 4608×2592    | ~69.7 ms     |

The fast mode is pinned explicitly rather than left to libcamera's resolution heuristic,
which a future resolution change could silently alter. It is pinned by geometry and bit
depth rather than by Bayer format string, because the 180° hardware rotation reverses the
Bayer order and a hardcoded format would fight the transform.

**Jello that survives all of this is mechanical.** That is the project's current limiting
factor — see [Safety and limitations](safety-and-limitations.md).

## Detection

[`vision/apriltag_detector.py`](../vision/apriltag_detector.py) uses OpenCV's ArUco module
with the AprilTag 36h11 dictionary, so no separate AprilTag library is needed.

Detection cost scales with pixel count, but pose accuracy depends only on corner precision.
Detecting at full 1280×720 costs about 71 ms on this Pi — more than double the 33 ms budget
for 30 fps, which starved the whole control loop down to about 8 fps. So detection runs on
a half-scale image, and the corners are then refined back against the **full-resolution**
frame with `cornerSubPix`. That costs about 17 ms and gives up nothing that matters,
because refinement only inspects small windows around four points. Pose is always solved
with full-resolution intrinsics against full-resolution corner coordinates.

Detector parameters are loosened from OpenCV's defaults for a blurry, high-gain image:

| Parameter                      | Value    | Reason                                        |
| ------------------------------ | -------- | --------------------------------------------- |
| `cornerRefinementMethod`       | SUBPIX   | Default is NONE, which puts corners on whole pixels — free pose jitter |
| `polygonalApproxAccuracyRate`  | 0.06     | Tolerates rounded and smeared corners         |
| `minMarkerPerimeterRate`       | 0.02     | Sees the tag from further away                |
| `errorCorrectionRate`          | 0.8      | Accepts more bit errors from a noisy frame    |

## Pose estimation and filtering

Pose is solved from the four tag corners and converted to ArduPilot's forward-right-down
body frame by [`vision/frame_transform.py`](../vision/frame_transform.py).

[`vision/pose_filter.py`](../vision/pose_filter.py) then gates single-frame spikes. Jello
occasionally corrupts one frame's corner geometry badly enough that the solver reports the
tag metres from where it is. That is poison for the derivative term: a 0.5 m spike across
one 33 ms frame looks like 15 m/s and would slam the attitude command into its clamp.

The gate is deliberately cheap. A detection is rejected if it implies a tag speed above
4 m/s relative to the last accepted pose — but a jump that **persists** into the next frame
is accepted, so genuine motion pays only one frame of delay while a jump that snaps back is
discarded. It is a despiker, not a tracker: two consecutively corrupted frames pass. That
is acceptable because sustained garbage never refreshes the staleness clock, so it decays
into the normal tag-lost neutral hover.

`vision_test.py` deliberately does **not** use the gate. Diagnostics must show raw output.

## Preprocessing

CLAHE and unsharp masking are available but **default to off**. Benchmarked against
synthetically degraded tags, the tuned detector parameters recovered every case CLAHE also
recovered — so CLAHE bought nothing while costing about 5 ms of a 33 ms budget. Where it
should earn its keep is uneven lighting (a backlit tag, hard shadows, a bright window
behind it), which the synthetic benchmark did not model. Measure with `camera_tune.py`
before enabling either.

## Camera calibration

The detector loads `config/camera_intrinsics.npz` if present. Without it, it falls back to
intrinsics estimated from the Camera Module 3 datasheet and prints a warning. The fallback
is fine for bring-up but **not accurate enough for distance-based control**.

```bash
# Headless — watch http://<pi-ip>:8080/stream, frames auto-capture as you move the board
python3 calibration/capture_calibration_images.py

# Measure one printed square, set SQUARE_SIZE_M in the script, then:
python3 calibration/calibrate_camera.py
```

Two things must hold or the intrinsics are silently wrong:

- **Calibrate at the operating focus.** Intrinsics depend on lens position, and focus is
  locked at 1 m in flight, so capture locks it at 1 m too.
- **Capture through the same 180° rotation.** The camera is mounted upside down and the
  rotation happens in hardware for every consumer, so the principal point and tangential
  distortion signs match how the intrinsics are actually used.

Print [`assets/print/calibration-chessboard.png`](../assets/print/calibration-chessboard.png)
(9×6 internal corners) and cover the whole frame at varied distances and tilts.

## Measuring the result

`camera_tune.py` sweeps exposure and reports detection rate, corner jitter, drift,
sharpness, and brightness per setting.

The jitter metric is not a plain standard deviation of corner positions. A drone hovering
without GPS drifts, sliding the tag across the image, and a naive standard deviation would
report that drift as vibration. Vibration is high-frequency and drift is low-frequency, so
the metric takes the second time difference of corner positions, which cancels
constant-velocity motion exactly.

That cancellation only holds for uniformly spaced samples. An early version silently
collapsed dropped frames together — with 37% detection, a five-frame gap looked like one
step and the drone's drift across it reappeared as huge fake jitter. A real flight logged
15 px of "vibration" this way; correlating it at +0.60 with drift gave it away. The metric
now differences only triplets of consecutive **detected** frames and returns `n/a` when
there are too few, rather than fabricating a number.

One caveat the tool enforces: on a still bench a **longer** exposure always scores sharper,
because there is more light and no vibration to freeze. Sweeping statically tunes you
straight back into motion blur. Run it with the motors spinning — propellers off, in
Stabilize only — and pass `--vibrating`.
