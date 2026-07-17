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
  LensPosition  a fixed default, and continuous autofocus hunts on a
                vibrating platform — every hunt is a blurred frame.
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

ROLLING SHUTTER ("jello"): a short exposure freezes blur but NOT the row-by-row
readout skew — that is proportional to the sensor mode's READOUT TIME, which
software can only minimise, never eliminate. So we pin the IMX708's fastest-
readout mode (1536x864, ~8.3 ms readout vs ~17.9 ms for 2304x1296) instead of
letting libcamera pick one by resolution heuristics, run the sensor at 60 fps
with queue=False so every capture_array() returns a freshly exposed frame
(the detect loop is CPU-bound near 30 fps either way — the higher rate buys
FRESHNESS, i.e. control latency, not loop rate), and rotate 180 deg in the ISP
(the camera is mounted upside-down) so no consumer pays for cv2.flip. Jello
that survives all this is mechanical: damp the camera mount or move to a
global-shutter sensor.
"""
import cv2
from picamera2 import Picamera2

try:
    from libcamera import Transform, controls as libcontrols
    _AF_MANUAL = int(libcontrols.AfModeEnum.Manual)
    _NR_OFF = int(libcontrols.draft.NoiseReductionModeEnum.Off)
except Exception:            # pragma: no cover - non-Pi dev machines
    libcontrols = None
    Transform = None
    _AF_MANUAL = 0
    _NR_OFF = 0

DEFAULT_RESOLUTION = (1280, 720)

# 60 fps: the detect loop stays CPU-bound near 30 fps, but with queue=False the
# average age of the frame each capture returns halves (<=16.7 ms vs <=33 ms).
DEFAULT_FPS = 60.0

# Pin the fast-readout sensor mode. Rolling-shutter shear is proportional to
# readout time, and libcamera's mode choice is a resolution heuristic that a
# future resolution/fps tweak could silently change. Pinned by geometry + bit
# depth, NOT by Bayer format string: the 180 deg flip reverses the Bayer order
# (BGGR -> RGGB), so a hardcoded format would fight the transform.
SENSOR_MODE_SIZE = (1536, 864)
SENSOR_MODE_BIT_DEPTH = 10

# Readout time ~= 1 / mode max fps. Shear amplitude scales with this.
READOUT_MS_BY_MODE = {(1536, 864): 8.3, (2304, 1296): 17.9, (4608, 2592): 69.7}

# 2 ms: short enough to freeze ~100 Hz vibration, long enough that gain 8 is
# still a usable image under normal indoor light.
DEFAULT_EXPOSURE_US = 2000
DEFAULT_GAIN = 8.0

# LensPosition is in DIOPTRES (1 / distance_in_metres), not metres.
# 1.0 m -> 1.0 dioptre. This MATCHES hover_on_tag's default --distance; if you
# change one, change the other or the tag sits outside the depth of field.
DEFAULT_FOCUS_M = 1.0

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
    describe(args, picam2)
    return picam2


def open_camera_quiet(args):
    """Same, without printing — for sweeps that reopen the camera repeatedly."""
    return _build(args)


def _build(args):
    picam2 = Picamera2()

    fps = float(getattr(args, "fps", DEFAULT_FPS))
    controls = {}

    if getattr(args, "auto_exposure", False):
        controls["AeEnable"] = True
    else:
        exposure = max(MIN_EXPOSURE_US, min(MAX_EXPOSURE_US, int(args.exposure_us)))
        gain = max(1.0, min(MAX_GAIN, float(args.gain)))
        # libcamera silently CLAMPS exposure to the frame period; a 20 ms
        # comparison run at 60 fps would really shoot at 16 ms. Stretch the
        # frame rate instead so the requested exposure is the real one.
        if exposure > 1e6 / fps / 1.05:
            stretched = 1e6 / (exposure * 1.05)
            print(f"[camera] exposure {exposure} us exceeds the frame period at "
                  f"{fps:g} fps — frame rate lowered to {stretched:.1f}")
            fps = stretched
        controls["AeEnable"] = False
        controls["ExposureTime"] = exposure
        controls["AnalogueGain"] = gain

    controls["FrameRate"] = fps

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

    # sensor=  pins the fast-readout mode (see SENSOR_MODE_SIZE comment).
    # transform  rotates 180 deg in hardware — the camera is mounted upside-
    #            down, and calibration was captured on flipped frames, so this
    #            MUST stay equivalent to the cv2.flip(-1) it replaces.
    # queue=False  makes capture_array() wait for the NEXT freshly exposed
    #            frame instead of returning one queued up to a frame earlier.
    picam2.configure(picam2.create_video_configuration(
        main={"size": tuple(args.resolution), "format": "RGB888"},
        controls=controls,
        buffer_count=4,
        sensor={"output_size": SENSOR_MODE_SIZE,
                "bit_depth": SENSOR_MODE_BIT_DEPTH},
        transform=Transform(hflip=1, vflip=1),
        queue=False,
    ))
    picam2.start()
    return picam2


def describe(args, picam2=None):
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
    if picam2 is not None:
        cfg = picam2.camera_configuration()
        sensor = cfg.get("sensor")
        size = tuple(getattr(sensor, "output_size", None)
                     or cfg.get("raw", {}).get("size", ()))
        readout = READOUT_MS_BY_MODE.get(size)
        readout_s = f"readout ~{readout} ms" if readout else "readout UNKNOWN"
        fps = float(getattr(args, "fps", DEFAULT_FPS))
        print(f"[camera] sensor:   {size[0]}x{size[1]} ({readout_s}) | "
              f"HW 180° flip | queue=False | {fps:g} fps target")
        if size != SENSOR_MODE_SIZE:
            print(f"[camera] WARNING: expected sensor mode "
                  f"{SENSOR_MODE_SIZE[0]}x{SENSOR_MODE_SIZE[1]} — readout time "
                  f"(jello) and calibration validity are now unknown")


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
