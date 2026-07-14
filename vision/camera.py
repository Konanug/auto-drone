"""Camera setup tuned for AprilTag detection on a VIBRATING airframe.

Every script used to open the camera with picamera2's defaults, which are the
worst possible choice here:

  ExposureTime  default 20000 us (20 ms), and auto-exposure pushes it even
                longer indoors. The motors vibrate at roughly 65-100 Hz
                (10-15 ms period), so a 20 ms frame integrates across ONE TO
                TWO FULL VIBRATION CYCLES — the tag's edges get smeared by the
                entire vibration amplitude. This is the single biggest cause
                of failed detection in flight.
  AnalogueGain  default 1.0, with 16x of unused headroom.
  LensPosition  default 1.0 dioptre = focused at 1 m, while we hover at 0.5 m.
  NoiseReduction / Sharpness  tuned for pretty photos, not for crisp edges.

So we go fully manual:

  - SHORT exposure (default 2 ms) to freeze the vibration. At ~100 Hz this
    captures only ~1/5 of a cycle instead of 2 whole ones.
  - HIGH analogue gain to pay for the light we just gave up. AprilTag
    detection tolerates NOISE far better than it tolerates BLUR — a grainy
    sharp frame decodes, a clean smeared one does not.
  - FIXED focus at the working distance. Continuous autofocus hunts on a
    vibrating, moving platform and every hunt is a blurred frame.
  - Noise reduction OFF and sharpness up, because denoising rounds off exactly
    the high-contrast corners the detector keys on.

Exposure and gain interact: if you shorten exposure you must raise gain, or
the frame goes black and detection fails for a different reason. Use
camera_tune.py to find the pair that actually works in your lighting.
"""
import cv2
from picamera2 import Picamera2

try:
    from libcamera import controls as libcontrols
    _AF_MANUAL = int(libcontrols.AfModeEnum.Manual)
    _NR_OFF = int(libcontrols.draft.NoiseReductionModeEnum.Off)
except Exception:            # pragma: no cover - non-Pi dev machines
    libcontrols = None
    _AF_MANUAL = 0
    _NR_OFF = 0

DEFAULT_RESOLUTION = (1280, 720)
DEFAULT_FPS = 30.0

# 2 ms: short enough to freeze ~100 Hz vibration, long enough that gain 8 is
# still a usable image under normal indoor light.
DEFAULT_EXPOSURE_US = 2000
DEFAULT_GAIN = 8.0

# LensPosition is in DIOPTRES (1 / distance_in_metres), not metres.
# 0.5 m -> 2.0 dioptres. This matches hover_on_tag's default --distance.
DEFAULT_FOCUS_M = 0.5

MIN_EXPOSURE_US = 1
MAX_EXPOSURE_US = 66666
MAX_GAIN = 16.0


def focus_m_to_dioptres(distance_m):
    """LensPosition wants dioptres. 0 (or <=0) means focus at infinity."""
    if distance_m is None or distance_m <= 0:
        return 0.0
    return 1.0 / distance_m


def add_camera_args(parser):
    """Attach the camera-tuning flags to an argparse parser."""
    g = parser.add_argument_group("camera (vibration/blur tuning)")
    g.add_argument("--resolution", type=int, nargs=2, default=DEFAULT_RESOLUTION)
    g.add_argument("--fps", type=float, default=DEFAULT_FPS)
    g.add_argument("--exposure-us", type=int, default=DEFAULT_EXPOSURE_US,
                   help=f"Shutter time in microseconds (default {DEFAULT_EXPOSURE_US} "
                        "= 2 ms). SHORTER freezes vibration; raise --gain to "
                        "compensate for the lost light.")
    g.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                   help=f"Analogue gain, 1.0-{MAX_GAIN} (default {DEFAULT_GAIN}). "
                        "Pays for the short exposure. Noise is fine; blur is not.")
    g.add_argument("--focus-m", type=float, default=DEFAULT_FOCUS_M,
                   help=f"Fixed focus distance in metres (default {DEFAULT_FOCUS_M}). "
                        "0 = infinity. Autofocus stays OFF either way.")
    g.add_argument("--auto-exposure", action="store_true",
                   help="Revert to picamera2's auto exposure/gain. This is what "
                        "produced the 20 ms motion-blurred frames — for comparison only.")
    g.add_argument("--autofocus", action="store_true",
                   help="Re-enable continuous autofocus. It hunts on a vibrating "
                        "platform; for comparison only.")
    return parser


def open_camera(args):
    """Configure and start the camera from parsed args. Returns Picamera2."""
    picam2 = _build(args)
    describe(args)
    return picam2


def open_camera_quiet(args):
    """Same, without printing — for sweeps that reopen the camera repeatedly."""
    return _build(args)


def _build(args):
    picam2 = Picamera2()

    controls = {"FrameRate": float(getattr(args, "fps", DEFAULT_FPS))}

    if getattr(args, "auto_exposure", False):
        controls["AeEnable"] = True
    else:
        exposure = max(MIN_EXPOSURE_US, min(MAX_EXPOSURE_US, int(args.exposure_us)))
        gain = max(1.0, min(MAX_GAIN, float(args.gain)))
        controls["AeEnable"] = False
        controls["ExposureTime"] = exposure
        controls["AnalogueGain"] = gain

    if getattr(args, "autofocus", False):
        if libcontrols is not None:
            controls["AfMode"] = int(libcontrols.AfModeEnum.Continuous)
    else:
        controls["AfMode"] = _AF_MANUAL
        controls["LensPosition"] = focus_m_to_dioptres(getattr(args, "focus_m",
                                                               DEFAULT_FOCUS_M))

    # Denoising rounds off the sharp corners the tag detector depends on.
    controls["NoiseReductionMode"] = _NR_OFF
    controls["Sharpness"] = 2.0

    picam2.configure(picam2.create_video_configuration(
        main={"size": tuple(args.resolution), "format": "RGB888"},
        controls=controls,
        buffer_count=4,
    ))
    picam2.start()
    return picam2


def describe(args):
    """Print the settings actually in force, so a bad frame is explainable."""
    if getattr(args, "auto_exposure", False):
        exp = "AUTO (may reach 20+ ms — expect motion blur)"
    else:
        exp = f"{args.exposure_us} us ({args.exposure_us / 1000.0:.1f} ms), gain {args.gain}"
    if getattr(args, "autofocus", False):
        foc = "CONTINUOUS autofocus (will hunt while vibrating)"
    else:
        d = getattr(args, "focus_m", DEFAULT_FOCUS_M)
        foc = f"fixed at {d} m ({focus_m_to_dioptres(d):.2f} dioptres)"
    print(f"[camera] exposure: {exp}")
    print(f"[camera] focus:    {foc}")


def sharpness(frame_bgr):
    """Variance of the Laplacian — a standard focus/blur score.

    Higher = crisper edges. Only meaningful COMPARED against itself on the
    same scene: use it to tell whether a settings change made the image
    sharper, not as an absolute number.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def brightness(frame_bgr):
    """Mean pixel level 0-255. If this collapses, the exposure/gain pair is
    too dark and detection will fail for a reason that is NOT blur."""
    return float(frame_bgr.mean())
