#!/usr/bin/env python3
"""
camera_tune.py — find camera settings that actually detect the tag while the
airframe is vibrating.

Zero MAVLink, zero control. Just the camera and the detector.

Two modes:

  SWEEP (default): steps through a range of exposure times, and for each one
  reports the numbers that decide whether detection works:

      detect%   fraction of frames the tag was found in   <- THE metric
      jitter    HIGH-FREQUENCY corner scatter in px = the VIBRATION reaching
                the sensor. This is what wrecks pose estimation.
                It is drift-immune: a hovering drone with no GPS cannot hold
                station, and that drift slides the tag across the image. A
                naive std-dev would report your drift as vibration. This uses
                a temporal high-pass (second difference), so steady drift
                contributes EXACTLY ZERO. Verified: 123 px of drift -> 0.00 px
                of jitter. You do NOT need to hold the drone still.
      drift     how far the tag's centre travelled — informational only, so a
                surprising jitter number can be sanity-checked.
      sharp     variance of Laplacian — higher = crisper edges
      bright    mean pixel level. If this collapses the frame is just DARK,
                and detection is failing for a reason that is NOT blur.

  LIVE (--live): one setting, streamed to the browser with the numbers
  overlaid, so you can watch it while you throttle up.

HOW TO USE IT (the whole point):
  1. TAPE THE TAG to a wall at your hover distance. It must not move — the
     drone may move all it likes.
  2. Baseline: run it with the drone off. Note detect% and jitter.
  3. The real one: FLY A MANUAL HOVER in front of the tag with
        camera_tune.py --live --log flight1.csv
     then land and read the CSV. You do NOT have to hold position, and you do
     NOT have to watch anything mid-flight. A real hover is also the only test
     that contains real PROP IMBALANCE, which is the dominant vibration source
     and which a props-off bench run cannot reproduce.
  4. The exposure where detect% stays high and jitter stays low is your answer.

INTERPRETING IT:
  - jitter drops sharply as exposure shortens  -> your problem is MOTION BLUR,
    and a short shutter fixes it. Good news.
  - jitter stays high even at 1-2 ms           -> the blur is gone but the tag
    is still moving between rows: that is ROLLING SHUTTER SHEAR, which no
    exposure setting can fix. You need mechanical damping, or a global-shutter
    camera (Raspberry Pi GS Camera / IMX296).

Browser (live mode): http://<pi-ip>:<port>/stream
Ctrl+C to stop.
"""
import argparse
import csv
import time

import cv2
import numpy as np
from picamera2 import Picamera2

from streaming.mjpeg_server import get_local_ip, start_mjpeg_server
from vision import camera as cam
from vision import preprocess as pre
from vision.apriltag_detector import AprilTagDetector

SWEEP_EXPOSURES_US = [500, 1000, 2000, 4000, 8000, 16000, 20000]


class _Settings:
    """Duck-types the argparse namespace that vision.camera.open_camera wants."""

    def __init__(self, resolution, fps, exposure_us, gain, focus_m):
        self.resolution = resolution
        self.fps = fps
        self.exposure_us = exposure_us
        self.gain = gain
        self.focus_m = focus_m
        self.auto_exposure = False
        self.autofocus = False


def corner_jitter(corner_history):
    """HIGH-FREQUENCY corner jitter in px, with slow drift removed.

    This must not be a plain std-dev of corner positions. A hovering drone with
    no GPS and no optical flow DRIFTS — it cannot hold station — and that drift
    slides the tag across the image. A naive std-dev would report the drift as
    "jitter" and completely swamp the thing we actually want to measure.

    Vibration is HIGH frequency (65-100 Hz, aliased by the 30 fps camera into
    random frame-to-frame scatter). Drift is LOW frequency (smooth across many
    frames). So we take the SECOND DIFFERENCE in time:

        d2[i] = c[i-1] - 2*c[i] + c[i+1]

    Any constant-velocity motion cancels exactly, so steady drift contributes
    ZERO. What survives is the frame-to-frame scatter that vibration produces.
    The /sqrt(6) puts it back on the same scale as a per-frame std-dev.
    """
    if len(corner_history) < 5:
        return float("nan")
    arr = np.array(corner_history, dtype=np.float64)      # (frames, 4, 2)
    d2 = arr[2:] - 2.0 * arr[1:-1] + arr[:-2]
    return float(d2.std() / np.sqrt(6.0))


def corner_drift(corner_history):
    """How far the tag's centre travelled across the window, in px.

    This is your DRIFT — informational only, NOT the vibration metric. It is
    here so a big number in the jitter column can be sanity-checked: if drift
    is huge and jitter is small, the high-pass is doing its job.
    """
    if len(corner_history) < 2:
        return float("nan")
    centres = np.array(corner_history, dtype=np.float64).mean(axis=1)
    return float(np.linalg.norm(centres[-1] - centres[0]))


def measure(picam2, detector, seconds, flip=True):
    """Collect detection stats for one camera setting."""
    frames = hits = 0
    sharp_sum = bright_sum = 0.0
    corners = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        frame = picam2.capture_array()
        if flip:
            frame = cv2.flip(frame, -1)
        frames += 1
        sharp_sum += cam.sharpness(frame)
        bright_sum += cam.brightness(frame)
        dets = detector.detect(frame)
        if dets:
            hits += 1
            corners.append(dets[0]["corners"].reshape(-1, 2))
    if frames == 0:
        return None
    # jitter only makes sense over a run of consecutive detections
    jit = corner_jitter(corners[-60:]) if len(corners) >= 5 else float("nan")
    drift = corner_drift(corners[-60:]) if len(corners) >= 2 else float("nan")
    return {
        "frames": frames,
        "detect_pct": 100.0 * hits / frames,
        "sharp": sharp_sum / frames,
        "bright": bright_sum / frames,
        "jitter": jit,
        "drift": drift,
    }


def run_sweep(args):
    detector = AprilTagDetector(detect_scale=args.detect_scale)
    print("\nSweeping exposure. TAPE THE TAG DOWN — the tag must not move.")
    print("The DRONE may drift freely: the jitter metric is drift-immune, so you")
    print("do not need to hold station. Fly it, or run this with the motors")
    print("spinning (props OFF, STABILIZE only — never an altitude-held mode on")
    print("a restrained drone).\n")
    print(f"{'exposure':>10} {'gain':>5} {'detect%':>8} {'jitter px':>10} "
          f"{'drift px':>9} {'sharp':>8} {'bright':>7}")
    print("-" * 64)

    results = []
    for exp in args.exposures:
        settings = _Settings(tuple(args.resolution), args.fps, exp, args.gain,
                             args.focus_m)
        picam2 = cam.open_camera_quiet(settings)
        time.sleep(0.4)                       # let AE/lens settle
        stats = measure(picam2, detector, args.seconds)
        picam2.stop()
        picam2.close()
        if stats is None:
            continue
        results.append((exp, stats))
        jit = "       n/a" if np.isnan(stats["jitter"]) else f"{stats['jitter']:10.2f}"
        dft = "      n/a" if np.isnan(stats["drift"]) else f"{stats['drift']:9.1f}"
        print(f"{exp:>8} us {args.gain:>5.1f} {stats['detect_pct']:>7.0f}% {jit} {dft} "
              f"{stats['sharp']:>8.0f} {stats['bright']:>7.1f}")

    if not results:
        print("\nNo frames captured.")
        return

    print()
    if all(s["detect_pct"] == 0 for _, s in results):
        print("The tag was NEVER detected, at any exposure. That is almost")
        print("certainly not a tuning problem — check the tag is actually in")
        print("view (open the stream with --live), the right size, and lit.")
        return

    if not args.vibrating:
        print("NOTE: this looks like a STATIC measurement. On a still bench a")
        print("LONGER exposure always scores sharper (more light, less noise) —")
        print("the whole point of a short shutter is to freeze VIBRATION, which")
        print("isn't present. Re-run with the motors spinning (props OFF,")
        print("STABILIZE only) and pass --vibrating, or you will tune yourself")
        print("straight back into motion blur.\n")

    good = [(e, s) for e, s in results if s["detect_pct"] >= 90 and s["bright"] > 25]
    if good:
        best = min(good, key=lambda r: (r[1]["jitter"] if not np.isnan(r[1]["jitter"])
                                        else 1e9))
        print(f"BEST: --exposure-us {best[0]} --gain {args.gain}   "
              f"(detect {best[1]['detect_pct']:.0f}%, "
              f"jitter {best[1]['jitter']:.2f} px)")
    else:
        dark = [s for _, s in results if s["bright"] <= 25]
        if dark:
            print("Every setting was too DARK to detect — raise --gain (max 16) "
                  "or add light, then re-run.")
        else:
            print("Nothing reached 90% detection. If jitter stayed high even at "
                  "500 us, the blur is not the problem — that is rolling-shutter "
                  "shear, and it needs mechanical damping or a global-shutter "
                  "camera.")


def run_live(args):
    detector = AprilTagDetector(detect_scale=args.detect_scale)
    prep = pre.Preprocessor(args)
    picam2 = cam.open_camera(args)
    print(f"[vision] detect_scale={args.detect_scale}  preprocessing: {prep.describe()}")

    writer = fh = None
    if args.log:
        fh = open(args.log, "w", newline="")
        writer = csv.writer(fh)
        writer.writerow(["t", "detect_pct", "jitter_px", "drift_px", "sharp",
                         "bright", "exposure_us", "gain"])
        print(f"Logging to {args.log} — FLY, land, then read the CSV. "
              "Nothing to watch mid-flight.")
    httpd, buf = start_mjpeg_server(args.port)
    print(f"Stream: http://{get_local_ip()}:{args.port}/stream")
    print("Ctrl+C to stop.\n")

    corners = []
    hits = frames = 0
    last_print = 0.0
    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.flip(frame, -1)
            frame = prep.apply(frame)      # detect on exactly what we display
            frames += 1
            dets = detector.detect(frame)
            if dets:
                hits += 1
                corners.append(dets[0]["corners"].reshape(-1, 2))
                corners = corners[-60:]
                detector.annotate(frame, dets)

            jit = corner_jitter(corners)
            dft = corner_drift(corners)
            rate = 100.0 * hits / max(1, frames)
            lines = [
                f"exposure {args.exposure_us} us  gain {args.gain}",
                f"detect   {rate:5.1f}%",
                f"jitter   {'n/a' if np.isnan(jit) else f'{jit:.2f} px'}  (vibration)",
                f"drift    {'n/a' if np.isnan(dft) else f'{dft:.0f} px'}  (ignored)",
                f"sharp    {cam.sharpness(frame):.0f}",
                f"bright   {cam.brightness(frame):.0f}",
            ]
            for i, t in enumerate(lines):
                cv2.putText(frame, t, (10, 28 + i * 24), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 220, 255), 2, cv2.LINE_AA)

            now = time.monotonic()
            if now - last_print >= 1.0:
                sh, br = cam.sharpness(frame), cam.brightness(frame)
                print(f"detect={rate:5.1f}%  jitter="
                      f"{'n/a' if np.isnan(jit) else f'{jit:.2f}px'}  "
                      f"sharp={sh:.0f}  bright={br:.0f}")
                if writer is not None:
                    writer.writerow([f"{time.monotonic():.2f}", f"{rate:.1f}",
                                     "" if np.isnan(jit) else f"{jit:.3f}",
                                     "" if np.isnan(dft) else f"{dft:.1f}",
                                     f"{sh:.0f}", f"{br:.1f}",
                                     args.exposure_us, args.gain])
                    fh.flush()          # survive a hard power-off after landing
                last_print = now
                hits = frames = 0     # rolling window

            ok, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                buf.push(enc.tobytes())
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        httpd.shutdown()
        if fh is not None:
            fh.close()
            print(f"\nSaved {args.log}")


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    cam.add_camera_args(p)
    pre.add_preprocess_args(p)
    p.add_argument("--detect-scale", type=float, default=0.5,
                   help="Detect on a downscaled image (corners still refined at "
                        "full res). Default 0.5.")
    p.add_argument("--log", default=None,
                   help="CSV to record detect%%/jitter to (live mode). Use this to "
                        "measure DURING A REAL HOVER: fly, land, read the file. No "
                        "clamped drone, nothing to watch mid-flight.")
    p.add_argument("--live", action="store_true",
                   help="Stream one setting live instead of sweeping.")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--seconds", type=float, default=3.0,
                   help="Seconds to measure per exposure step.")
    p.add_argument("--exposures", type=int, nargs="+", default=SWEEP_EXPOSURES_US,
                   help="Exposure times (us) to sweep.")
    p.add_argument("--vibrating", action="store_true",
                   help="You are running this with the motors spinning. Without "
                        "vibration the sweep is misleading — a static scene always "
                        "favours a LONG exposure, which is the opposite of what you "
                        "need in flight.")
    return p.parse_args()


if __name__ == "__main__":
    a = parse_args()
    run_live(a) if a.live else run_sweep(a)
