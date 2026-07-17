"""Per-tag velocity estimation from consecutive AprilTagDetector detections.

Finite-difference velocity (m/s) in the ArduPilot FRD body frame, smoothed
with exponential moving average. This exists to validate tag-motion tracking
on its own — a future follow-controller would consume it as feedforward, but
nothing here sends anything anywhere; it's pure signal processing on
detector output.

IMPORTANT: this is the tag's velocity *relative to the camera*, not the tag's
velocity in the room and not the drone's own ground velocity. A single
monocular camera can never tell those apart — it only ever sees relative
motion between itself and the tag. Today that's fine because nothing can
move the drone yet, so any relative motion is the tag's. Once a follow-
controller exists, relative velocity is still the *correct* signal to null
out (the controller shouldn't care whether the drone drifted or the tag
moved — same correction either way). Only if something later needs the
tag's motion independent of the drone's own maneuvering would this need to
be combined with ArduPilot's own IMU/attitude estimate to subtract the
drone's ego-motion — not needed at this phase.

The smoothing is a TIME CONSTANT, not a per-sample blend, so the estimator's
bandwidth is frame-rate-invariant: hover_on_tag's KD_* damping was tuned in
SITL against this exact filter at 30 Hz, and it must keep behaving the same
if the vision loop rate changes.
"""
import math

# tau = -dt / ln(1 - alpha): the old per-sample alpha of 0.5 at the 30 Hz the
# KD_* gains were tuned against (sitl_tag_sim.py runs vision at exactly 30 Hz)
# is a 48 ms time constant — identical behavior today, invariant tomorrow.
DEFAULT_TAU_S = 0.048
DEFAULT_MAX_GAP_S = 0.5      # gap longer than this = re-acquisition, velocity resets to 0


class _TagVelocity:
    def __init__(self, tau_s, max_gap_s):
        self.tau_s = tau_s
        self.max_gap_s = max_gap_s
        self._last_pos = None
        self._last_t = None
        self.velocity = (0.0, 0.0, 0.0)  # (fwd, right, down) m/s

    def update(self, fwd_m, right_m, down_m, timestamp):
        if self._last_pos is None or (timestamp - self._last_t) > self.max_gap_s:
            self.velocity = (0.0, 0.0, 0.0)
        else:
            dt = timestamp - self._last_t
            if dt > 0:
                pos = (fwd_m, right_m, down_m)
                raw = tuple((c - p) / dt for c, p in zip(pos, self._last_pos))
                a = 1.0 - math.exp(-dt / self.tau_s)
                self.velocity = tuple(
                    a * r + (1 - a) * v for r, v in zip(raw, self.velocity)
                )
        self._last_pos = (fwd_m, right_m, down_m)
        self._last_t = timestamp
        return self.velocity


class VelocityEstimator:
    """Tracks per-tag-ID velocity across frames.

    Call update() once per detected tag, per frame it's seen in. A tag not
    seen for longer than max_gap_s is treated as re-acquired on its next
    sighting (velocity resets to zero rather than spiking from the gap).
    """

    def __init__(self, tau_s=DEFAULT_TAU_S, max_gap_s=DEFAULT_MAX_GAP_S):
        self.tau_s = tau_s
        self.max_gap_s = max_gap_s
        self._tracks = {}

    def update(self, tag_id, fwd_m, right_m, down_m, timestamp):
        """Returns (v_fwd, v_right, v_down) in m/s for this tag."""
        track = self._tracks.setdefault(
            tag_id, _TagVelocity(self.tau_s, self.max_gap_s)
        )
        return track.update(fwd_m, right_m, down_m, timestamp)

    def reset(self, tag_id=None):
        if tag_id is None:
            self._tracks.clear()
        else:
            self._tracks.pop(tag_id, None)
