"""Optional image preprocessing to help detection on a bad frame.

Deliberately DEFAULT-OFF, because none of it is free and none of it is proven
for this airframe yet. Benchmarked against synthetically blurred/noisy tags,
the tuned detector parameters (see apriltag_detector) recovered every case that
CLAHE also recovered — so CLAHE bought nothing there, while costing ~5 ms of a
33 ms frame budget. Where it SHOULD earn its keep is uneven lighting (a
backlit tag, hard shadows, a bright window behind it), which the synthetic
benchmark did not model. So: it is available, it is measurable with
camera_tune.py, and you should only turn it on if the numbers say it helps.

The one thing worth being clear about: NO amount of filtering recovers motion
blur that exceeds the tag's bit-cell size (tag_px / 8). At 2 m the tag is
~79 px, so its cells are ~10 px, and a 17 px blur is 0% detectable no matter
what you do to the image afterwards. Blur is fixed with a SHORTER SHUTTER
(vision/camera.py), not here. Filters can only rescue a frame whose
information still exists.
"""
import cv2

DEFAULT_CLAHE_CLIP = 3.0
DEFAULT_CLAHE_TILE = 8


def add_preprocess_args(parser):
    g = parser.add_argument_group("image preprocessing (default: off)")
    g.add_argument("--clahe", action="store_true",
                   help="Local contrast equalisation (CLAHE). Helps a backlit or "
                        "unevenly-lit tag; costs ~5 ms/frame. Cannot fix blur.")
    g.add_argument("--clahe-clip", type=float, default=DEFAULT_CLAHE_CLIP,
                   help=f"CLAHE strength (default {DEFAULT_CLAHE_CLIP}).")
    g.add_argument("--sharpen", action="store_true",
                   help="Unsharp mask. Can crisp a SOFT frame; on a genuinely "
                        "blurred one it mostly amplifies noise.")
    return parser


class Preprocessor:
    """Applies the enabled filters. Returns a BGR frame so callers can both
    detect on it AND show it — what you see in the stream is then exactly what
    the detector saw, which is the whole point of putting it on the stream."""

    def __init__(self, args=None):
        self.use_clahe = bool(getattr(args, "clahe", False))
        self.use_sharpen = bool(getattr(args, "sharpen", False))
        clip = float(getattr(args, "clahe_clip", DEFAULT_CLAHE_CLIP))
        self._clahe = (cv2.createCLAHE(clipLimit=clip,
                                       tileGridSize=(DEFAULT_CLAHE_TILE,
                                                     DEFAULT_CLAHE_TILE))
                       if self.use_clahe else None)

    @property
    def enabled(self):
        return self.use_clahe or self.use_sharpen

    def apply(self, frame_bgr):
        if not self.enabled:
            return frame_bgr
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        if self._clahe is not None:
            gray = self._clahe.apply(gray)
        if self.use_sharpen:
            blur = cv2.GaussianBlur(gray, (0, 0), 3)
            gray = cv2.addWeighted(gray, 1.8, blur, -0.8, 0)
        # Back to BGR so the rest of the pipeline (annotate, stream) is unchanged.
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    def describe(self):
        if not self.enabled:
            return "none (raw frame)"
        bits = []
        if self.use_clahe:
            bits.append("CLAHE")
        if self.use_sharpen:
            bits.append("unsharp")
        return " + ".join(bits) + "  (stream shows the processed frame)"
