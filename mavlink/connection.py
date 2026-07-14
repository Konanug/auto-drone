"""Read-only MAVLink link to the ArduPilot flight controller.

Current scope: heartbeat + telemetry monitoring only. Nothing in this module
sends a command that can arm, disarm, change flight mode, or move the
vehicle — the only outgoing message is this Pi's own heartbeat, which is
pure presence-announcement and carries no command authority.

Adding command authority (e.g. SET_ATTITUDE_TARGET for GUIDED_NOGPS control)
is a deliberate future step requiring explicit sign-off — see the "Control
Architecture" section in .claude/CLAUDE.md for the intended design and the
safety gates required before that lands.
"""
import time

from pymavlink import mavutil

DEFAULT_DEVICE = "/dev/serial0"
DEFAULT_BAUD = 921600
HEARTBEAT_TIMEOUT_S = 3.0


class FlightControllerLink:
    def __init__(self, device=DEFAULT_DEVICE, baud=DEFAULT_BAUD):
        self.device = device
        self.baud = baud
        self._conn = None
        self._last_heartbeat_time = 0.0
        self._last_heartbeat_msg = None
        self._latest = {}        # last message seen, keyed by type
        self._latest_time = {}   # monotonic time each was seen

    def connect(self, wait_heartbeat_timeout_s=10.0):
        self._conn = mavutil.mavlink_connection(self.device, baud=self.baud)
        print(f"[mavlink] Waiting for heartbeat on {self.device} @ {self.baud}...")

        # wait_heartbeat() would accept the first HEARTBEAT from anyone on the
        # link — including a GCS (e.g. Mission Planner on USB) whose traffic
        # ArduPilot relays across all connected MAVLink channels. Skip those
        # and bind only to an actual autopilot.
        deadline = time.monotonic() + wait_heartbeat_timeout_s
        msg = None
        while time.monotonic() < deadline:
            candidate = self._conn.recv_match(
                type="HEARTBEAT", blocking=True, timeout=deadline - time.monotonic()
            )
            if candidate is None:
                break
            if candidate.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue
            msg = candidate
            break

        if msg is None:
            raise TimeoutError(
                f"No MAVLink heartbeat received on {self.device} within "
                f"{wait_heartbeat_timeout_s}s. Check wiring, baud rate, and "
                f"that SERIALx_PROTOCOL=2 is set for this port in Mission Planner."
            )
        self._conn.target_system = msg.get_srcSystem()
        self._conn.target_component = msg.get_srcComponent()
        self._last_heartbeat_time = time.monotonic()
        self._last_heartbeat_msg = msg
        print(f"[mavlink] Heartbeat OK — system {self._conn.target_system}, "
              f"component {self._conn.target_component}")

    def poll(self):
        """Drain pending MAVLink messages and refresh cached status.

        Non-blocking — call frequently from the main loop.
        """
        if self._conn is None:
            raise RuntimeError("connect() must be called before poll()")
        while True:
            msg = self._conn.recv_match(blocking=False)
            if msg is None:
                break
            self._latest[msg.get_type()] = msg
            self._latest_time[msg.get_type()] = time.monotonic()
            if (msg.get_type() == "HEARTBEAT"
                    and msg.get_srcSystem() == self._conn.target_system
                    and msg.get_srcComponent() == self._conn.target_component):
                self._last_heartbeat_time = time.monotonic()
                self._last_heartbeat_msg = msg
            elif msg.get_type() == "STATUSTEXT":
                # FC-side messages (prearm failures, failsafes, motor-test
                # status...) — surface them instead of dropping them silently.
                print(f"[mavlink] FC: {msg.text}")
            elif msg.get_type() == "COMMAND_ACK":
                result_names = {0: "ACCEPTED", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                                3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
                if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print(f"[mavlink] FC REJECTED command {msg.command}: "
                          f"{result_names.get(msg.result, msg.result)}")

    def get_latest(self, msg_type, max_age_s=None):
        """Last message of msg_type seen by poll(), or None. If max_age_s is
        given, returns None when the cached message is older than that."""
        if msg_type not in self._latest:
            return None
        if max_age_s is not None and \
                (time.monotonic() - self._latest_time[msg_type]) > max_age_s:
            return None
        return self._latest[msg_type]

    def is_link_healthy(self, timeout_s=HEARTBEAT_TIMEOUT_S):
        return (time.monotonic() - self._last_heartbeat_time) < timeout_s

    def get_status(self):
        """Best-effort snapshot of vehicle state from the last heartbeat.

        Returns None until the first heartbeat has been received.
        """
        if self._last_heartbeat_msg is None:
            return None
        hb = self._last_heartbeat_msg
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        mode = mavutil.mode_string_v10(hb)
        return {
            "armed": armed,
            "mode": mode,
            "link_healthy": self.is_link_healthy(),
            "age_s": time.monotonic() - self._last_heartbeat_time,
        }

    def send_companion_heartbeat(self):
        """Announces the Pi's presence as an onboard controller.

        Presence-only — this message carries no command authority and
        cannot arm, disarm, change mode, or move the vehicle.
        """
        if self._conn is None:
            return
        self._conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )

    @property
    def raw_connection(self):
        """The underlying pymavlink connection, for callers that need to send
        something beyond this module's read-only scope (e.g. a bench-test
        command). This class itself still only ever sends its own heartbeat.
        """
        return self._conn

    def close(self):
        if self._conn is not None:
            self._conn.close()
            self._conn = None
