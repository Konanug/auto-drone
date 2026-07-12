"""Health/freshness watchdog for the camera, detector, and MAVLink link.

Current scope: observation and logging only. The Pi has no control
authority yet, so "safe behavior" today just means surfacing when
something has gone stale.

Once control logic exists, every outgoing command must be gated on
all_healthy(), and losing health must immediately stop sending new
attitude/velocity targets so ArduPilot's own failsafes and the pilot's
mode switch are the ones left in charge — this watchdog is a secondary
check, never the primary safety mechanism.
"""
import time


class Watchdog:
    def __init__(self, frame_timeout_s=0.5, detection_timeout_s=1.0,
                 mavlink_timeout_s=3.0):
        self.frame_timeout_s = frame_timeout_s
        self.detection_timeout_s = detection_timeout_s
        self.mavlink_timeout_s = mavlink_timeout_s
        self._last_frame_time = None
        self._last_detection_time = None
        self._last_mavlink_time = None

    def note_frame(self):
        self._last_frame_time = time.monotonic()

    def note_detection(self):
        self._last_detection_time = time.monotonic()

    def note_mavlink_heartbeat(self):
        self._last_mavlink_time = time.monotonic()

    @staticmethod
    def _age(t):
        return None if t is None else time.monotonic() - t

    def status(self):
        frame_age = self._age(self._last_frame_time)
        detection_age = self._age(self._last_detection_time)
        mavlink_age = self._age(self._last_mavlink_time)
        return {
            "camera_ok": frame_age is not None and frame_age < self.frame_timeout_s,
            "tag_ok": detection_age is not None and detection_age < self.detection_timeout_s,
            "mavlink_ok": mavlink_age is not None and mavlink_age < self.mavlink_timeout_s,
            "frame_age_s": frame_age,
            "detection_age_s": detection_age,
            "mavlink_age_s": mavlink_age,
        }

    def all_healthy(self):
        s = self.status()
        return s["camera_ok"] and s["tag_ok"] and s["mavlink_ok"]
