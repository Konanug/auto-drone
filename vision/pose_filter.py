"""Single-frame pose-spike gate for control consumers.

Rolling shutter + vibration ("jello") occasionally corrupts a tag's corner
geometry for one frame. The pose solver then reports a tag position metres
from where the tag really is — for exactly one frame. That is poison for
hover_on_tag's derivative damping: a 0.5 m position spike over one 33 ms
frame looks like 15 m/s, and KD_* would slam the attitude command against
its clamp.

The gate is deliberately dumb and cheap: a new detection is accepted unless
it implies a physically implausible speed (default 4 m/s — far outside the
controller's authority) relative to the LAST ACCEPTED position. A jump that
persists for two consecutive frames is accepted (real motion just pays one
frame of delay); a jump that snaps back was jello and is dropped entirely.

Known limitation, by design: two consecutive frames corrupted in the SAME
consistent way pass the gate. This is a single-frame despiker, not a
tracker. Sustained garbage never refreshes the caller's staleness clock, so
it decays into the existing TAG_LOST -> neutral-hover failsafe.

vision_test.py does NOT use this — diagnostics must show the raw output.
"""
import math

DEFAULT_MAX_SPEED_MPS = 4.0
DEFAULT_MAX_GAP_S = 0.5   # aligned with TAG_STALE_S / velocity max_gap_s


class PoseGate:
    def __init__(self, max_speed_mps=DEFAULT_MAX_SPEED_MPS,
                 max_gap_s=DEFAULT_MAX_GAP_S):
        self.max_speed_mps = max_speed_mps
        self.max_gap_s = max_gap_s
        self._accepted_pos = None
        self._accepted_t = None
        self._pending_pos = None
        self._pending_t = None
        self.rejected_count = 0
        self.last_reject_speed = 0.0

    @staticmethod
    def _pos(det):
        return (det["fwd_m"], det["right_m"], det["down_m"])

    @staticmethod
    def _speed(pos, t, ref_pos, ref_t):
        dt = t - ref_t
        if dt <= 0:
            return 0.0   # same-timestamp duplicate; nothing to judge
        return math.dist(pos, ref_pos) / dt

    def _plausible(self, pos, t, ref_pos, ref_t):
        return self._speed(pos, t, ref_pos, ref_t) <= self.max_speed_mps

    def accept(self, det):
        """True if this detection should feed control; False = drop it."""
        pos, t = self._pos(det), det["timestamp"]

        # Re-acquisition after a gap: nothing to compare against. The velocity
        # estimator resets itself over the same gap, so no spike either way.
        if (self._accepted_t is None
                or (t - self._accepted_t) > self.max_gap_s):
            self._accept(pos, t)
            return True

        speed = self._speed(pos, t, self._accepted_pos, self._accepted_t)
        if speed <= self.max_speed_mps:
            self._accept(pos, t)
            return True

        # Implausible jump. If the PREVIOUS frame made the same jump and this
        # one is consistent with it, it's real motion — let it through.
        if (self._pending_pos is not None
                and self._plausible(pos, t, self._pending_pos, self._pending_t)):
            self._accept(pos, t)
            return True

        self._pending_pos, self._pending_t = pos, t
        self.rejected_count += 1
        self.last_reject_speed = speed   # the jump vs last ACCEPTED pose
        return False

    def _accept(self, pos, t):
        self._accepted_pos, self._accepted_t = pos, t
        self._pending_pos = self._pending_t = None
