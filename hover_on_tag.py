#!/usr/bin/env python3
"""
hover_on_tag.py — GUIDED_NOGPS visual-servo hover controller.

Holds the drone hovering --distance metres (default 1.0) from the AprilTag,
centered on it and square to its face, by streaming SET_ATTITUDE_TARGET while
ArduPilot is armed and in GUIDED_NOGPS.

The control law is a decoupled goal-point PD controller:

    goal point  = the spot --distance out along the TAG'S NORMAL
    pitch/roll <- PD onto that goal point (position error + velocity damping)
    yaw        <- bearing to tag (independently keeps the nose on the tag)
    thrust     <- vertical tag offset (climbs/descends to center the tag)

VALIDATED IN SITL (sitl_validate.py, sitl_tag_sim.py). Three bugs were found
that way, each of which would have crashed the real drone:
  1. an illegal type_mask (mixing ignored/supplied body rates) — the FC
     silently DISCARDED every message;
  2. a hardcoded yaw of 0 in the quaternion — the quaternion's yaw is an
     ABSOLUTE earth-frame heading, so that commands a turn to face NORTH;
  3. pure-P control on a double integrator — the drone pinned max tilt and
     flew straight THROUGH the tag. Hence the KD_* velocity damping.

!!! YOU CANNOT TEST THIS ON A BENCH. DO NOT TRY. !!!
ArduCopter DISCARDS attitude targets while it believes it is landed
(mode_guided.cpp -> make_safe_ground_handling()), so a grounded drone shows
nothing. And a CLAMPED drone is worse: it cannot rotate, so ArduPilot's
attitude/altitude integrators wind up against the restraint and RAMP THE
MOTORS TO MAXIMUM. Both were observed on this airframe. Validate in SITL,
then in the air. There is no bench configuration that works.

WHO DOES WHAT (safety design — see .claude/CLAUDE.md "Safety Rules"):
- The PILOT arms via the transmitter and engages/disengages this controller
  by flipping the TX flight-mode switch into/out of GUIDED_NOGPS. That switch
  must be configured and TESTED in Mission Planner before any armed run.
- This script never arms, never changes mode, never sends anything except
  SET_ATTITUDE_TARGET and its own presence heartbeat. Until it observes
  armed + GUIDED_NOGPS in the FC heartbeat it computes and displays commands
  but sends NOTHING (shadow mode).
- Tag lost (or camera/detection/link stale): streams NEUTRAL — level attitude,
  zero yaw rate, altitude-hold thrust — and warns. It never searches or
  guesses. Control resumes automatically when the tag is re-acquired.
- If this script dies mid-flight, ArduPilot's GUID_TIMEOUT (default 3 s)
  levels the vehicle on its own; the TX switch remains the emergency exit.

THRUST FIELD SEMANTICS:
  GUID_OPTIONS=0 (CONFIRMED on this FC), so ArduCopter treats
  SET_ATTITUDE_TARGET.thrust as a CLIMB-RATE demand, not a throttle:
  0.5 = hold altitude, >0.5 climb, <0.5 descend (scaled by WPNAV_SPEED_UP,
  which is 250 on both this FC and SITL — so KD/KP_THRUST transfer directly).

CONTROL SIGNS: all four axes were verified in SITL against a flying vehicle —
+roll drifts right, -pitch drives forward, +yaw target turns right, thrust>0.5
climbs. Do not "fix" a sign without re-running sitl_validate.py. Note KD_ROLL
is POSITIVE while KD_PITCH is negative; that is deliberate, not a typo (see
its comment).

PREREQUISITE, NOT YET DONE: the FC's inner loop (ATC_RAT_*) is still on stock
defaults aimed at a ~10" copter. AUTOTUNE the real airframe first — this outer
loop only commands angles and trusts ArduPilot to deliver them.

Browser: http://<pi-ip>:<port>/stream
Ctrl+C to stop.
"""
import argparse
import math
import time

import cv2
import numpy as np
from pymavlink import mavutil

from mavlink.connection import DEFAULT_BAUD, DEFAULT_DEVICE, FlightControllerLink
from streaming.mjpeg_server import get_local_ip, start_mjpeg_server
from vision import camera as cam
from vision import preprocess as pre
from vision.apriltag_detector import AprilTagDetector
from vision.pose_filter import PoseGate
from vision.velocity_estimator import VelocityEstimator

STREAM_INTERVAL_S = 1 / 12.0   # debug stream at ~12 fps, not 30
# ── Camera -> flight-controller mounting offset (measured) ────────────────────
# Translation from the camera lens to the FC/vehicle center, in the FRD body
# frame, metres. The tag position is measured by the camera; these shift it to
# be relative to the vehicle center.
# Measured: camera is 90 mm in front of and 30 mm above the FC.
CAM_OFFSET_FWD_M = 0.090     # camera ahead of FC = positive
CAM_OFFSET_RIGHT_M = 0.0     # camera right of FC = positive
CAM_OFFSET_DOWN_M = -0.030   # camera below FC = positive (ours is above => negative)

# ── Controller gains / limits ─────────────────────────────────────────────────
# TUNED IN SITL CLOSED-LOOP (sitl_tag_sim.py), and RE-VALIDATED at the 1.0 m
# setpoint. Converges from a 4 m/+20 deg and a 6 m/-35 deg start: distance err
# <0.08 m, lateral <0.01 m, vertical <0.05 m, skew <3 deg, zero frames lost.
#
# The setpoint is NOT a free parameter. The goal point's lateral offset is
# distance * sin(skew), so a LARGER --distance gives the squaring-up loop MORE
# authority: moving 0.5 -> 1.0 m improved steady-state skew from 5.8 to 2.8 deg
# on its own. Re-run sitl_tag_sim.py if you change --distance again. They are on the CONSERVATIVE side for the real
# 816 g / ~7:1-thrust airframe, which is punchier than SITL's default quad.
# Re-run sitl_tag_sim.py after changing any of them.

# Yaw: bearing (deg) -> yaw ANGLE correction (deg), applied relative to the
# FC's CURRENT heading. NOT a yaw rate: SITL showed the rate feedforward is
# always overridden by the quaternion's yaw target, so rate commands do
# nothing. Positive bearing (tag right of nose) -> positive correction ->
# turn right toward the tag. Verified in SITL: commanding current+60 deg
# turned the vehicle +60.2 deg.
KP_YAW = 0.5
MAX_YAW_CORRECTION_DEG = 10.0   # cap how far ahead of current heading we aim,
                                # so the turn eases in instead of snapping
DEADBAND_RIGHT_M = 0.010     # 10 mm lateral tolerance — inside it, no yaw correction

# Roll: LATERAL error to the goal point (m) -> roll angle (deg). Positive =
# goal is to our right => roll right to strafe there. This is a POSITION gain,
# mirroring pitch — it is NOT driven by the tag's skew directly. Driving roll
# from skew while yaw chases the bearing makes the two loops fight and the
# drone ORBITS the tag forever (seen in SITL). See compute_commands().
KP_ROLL = 4.0
MAX_ROLL_DEG = 5.0
DEADBAND_POS_M = 0.05        # position deadband on the goal-point error

# Pitch: FORWARD error to the goal point (m) -> pitch angle (deg). Negative
# pitch = nose down = move forward (FRD), so a goal ahead => negative pitch.
KP_PITCH = -1.5
MAX_PITCH_DEG = 5.0
# Cap the forward error the P term may act on. Without this the drone pins max
# forward tilt from far away, RUSHES the tag, and arrives before the (much
# weaker) lateral loop has squared it up — SITL showed the viewing angle blowing
# past 60 deg and the tag becoming undetectable. Capping it bounds the approach
# speed so squaring-up keeps pace.
MAX_APPROACH_ERR_M = 0.7

# Thrust: vertical tag offset (down_m, +tag below camera axis => drone too
# high) -> climb-rate demand around NEUTRAL_THRUST. Tag below => descend =>
# thrust below neutral.
KP_THRUST = -0.30
NEUTRAL_THRUST = 0.5
MAX_THRUST_DELTA = 0.14
DEADBAND_DOWN_M = 0.05

# ── Velocity damping (the D in PD) — DO NOT REMOVE ────────────────────────────
# Tilt commands ACCELERATION, but we are controlling POSITION: that is a double
# integrator, and a pure-P controller on a double integrator ALWAYS overshoots.
# SITL proved it: with P only, the drone pinned max forward tilt for the whole
# approach, accelerated the entire way in, and flew straight through the tag.
#
# These damp the *relative* velocity from vision/velocity_estimator.py. Each KD
# carries the same sign as its KP, so motion toward the target produces braking
# (e.g. closing at -1 m/s with KD_PITCH=-3.2 gives +3.2 deg of nose-up brake).
KD_PITCH = -3.2     # damps v_fwd   (m/s -> deg)
KD_ROLL = 3.2       # damps v_right (m/s -> deg). POSITIVE on purpose:
                    # v_right goes NEGATIVE as the drone strafes right, so a
                    # positive coefficient brakes. It was negative once —
                    # that is ANTI-damping and SITL pumped it into an orbit.
KD_THRUST = -0.16   # damps v_down  (m/s -> thrust)

# Slew limits per send tick (SEND_HZ), so commands ramp instead of stepping.
SEND_HZ = 20.0
MAX_ANGLE_STEP_DEG = 0.5     # roll/pitch: 0.5°/tick @ 20 Hz = 10°/s max slew
MAX_THRUST_STEP = 0.005

# ── Freshness watchdog ─────────────────────────────────────────────────────────
TAG_STALE_S = 0.5       # no detection for this long => TAG_LOST
LINK_STALE_S = 1.5      # no FC heartbeat for this long => treat as TAG_LOST/neutral
ATTITUDE_STALE_S = 0.5  # no FC ATTITUDE for this long => we don't know our heading,
                        # so we must not send at all (see NO_HEADING in run())

GUIDED_NOGPS_MODE_NAME = "GUIDED_NOGPS"

# ATTITUDE_TARGET type_mask.
#
# VERIFIED AGAINST ArduPilot SOURCE + SITL — do not "simplify" this back:
# ArduCopter accepts ONLY all-three-body-rates-ignored, or all-three-supplied.
# Any mix is rejected outright (GCS_MAVLink_Copter.cpp: "The body rates are
# ill-defined" -> hold_position(); return). Ignoring roll+pitch rate while
# supplying yaw rate — the obvious-looking choice — makes the FC silently
# DISCARD every message. So: mask 0 (supply all three rates, as zeros) and
# carry roll/pitch/yaw purely as the attitude quaternion.
TYPE_MASK = 0


def euler_to_quat(roll_rad, pitch_rad, yaw_rad=0.0):
    """ZYX euler (rad) -> [w, x, y, z] quaternion (MAVLink order)."""
    cr, sr = math.cos(roll_rad / 2), math.sin(roll_rad / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    cy, sy = math.cos(yaw_rad / 2), math.sin(yaw_rad / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def quat_to_euler(q):
    """[w, x, y, z] quaternion -> (roll_deg, pitch_deg, yaw_deg). Inverse of
    euler_to_quat, used to decode the FC's echoed ATTITUDE_TARGET."""
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def clamp(value, limit):
    return max(-limit, min(limit, value))


def slew(current, target, max_step):
    return current + clamp(target - current, max_step)


def deadband(value, band):
    return 0.0 if abs(value) < band else value


def compute_commands(det, vel, target_distance_m):
    """Tag detection + relative velocity -> (roll, pitch, yaw_corr, thrust).

    PD control. `vel` is (v_fwd, v_right, v_down) in m/s from
    vision/velocity_estimator.py — the tag's velocity relative to the camera,
    which is exactly the signal to null regardless of whether the drone or the
    tag moved. The D terms are what stop it overshooting (see KD_* above).

    yaw_correction_deg is a DELTA applied to the FC's current heading at the
    send site — not a rate, and not an absolute heading. 0 = hold heading.
    Pure function; clamping here, slewing at the send site.
    """
    fwd = det["fwd_m"] + CAM_OFFSET_FWD_M
    right = det["right_m"] + CAM_OFFSET_RIGHT_M
    down = det["down_m"] + CAM_OFFSET_DOWN_M
    v_fwd, v_right, v_down = vel
    skew_rad = math.radians(det["yaw_deg"])   # 0 = square-on to the tag face

    # ── Where we actually want to BE ──────────────────────────────────────
    # The goal point is `target_distance_m` out along the tag's outward normal.
    # Driving the drone straight at that point decouples position from skew.
    #
    # Do NOT go back to the obvious-looking scheme (yaw centres the tag, roll
    # nulls the skew): those two loops chase each other — strafing changes the
    # bearing, which yaws, which changes the skew, which strafes — and SITL
    # showed the drone just ORBITS the tag forever, never converging.
    horiz = math.hypot(fwd, right)
    if horiz < 1e-3:
        ux, uy = -1.0, 0.0
    else:
        ux, uy = -fwd / horiz, -right / horiz     # unit vector tag -> drone
    c, s = math.cos(-skew_rad), math.sin(-skew_rad)
    nx = ux * c - uy * s                          # the tag's outward normal,
    ny = ux * s + uy * c                          # in our body frame
    e_fwd = fwd + target_distance_m * nx          # goal point, relative to us
    e_right = right + target_distance_m * ny

    # Yaw stays independent: just keep the nose pointed at the tag.
    bearing_deg = math.degrees(math.atan2(deadband(right, DEADBAND_RIGHT_M), fwd))

    yaw_corr = clamp(KP_YAW * bearing_deg, MAX_YAW_CORRECTION_DEG)
    pitch = clamp(KP_PITCH * deadband(clamp(e_fwd, MAX_APPROACH_ERR_M),
                                      DEADBAND_POS_M)
                  + KD_PITCH * v_fwd, MAX_PITCH_DEG)
    roll = clamp(KP_ROLL * deadband(e_right, DEADBAND_POS_M)
                 + KD_ROLL * v_right, MAX_ROLL_DEG)
    thrust = NEUTRAL_THRUST + clamp(
        KP_THRUST * deadband(down, DEADBAND_DOWN_M)
        + KD_THRUST * v_down, MAX_THRUST_DELTA)

    return roll, pitch, yaw_corr, thrust


NEUTRAL_COMMANDS = (0.0, 0.0, 0.0, NEUTRAL_THRUST)


def send_attitude_target(conn, boot_time, roll_deg, pitch_deg, target_yaw_rad, thrust):
    """Stream one attitude target.

    target_yaw_rad is an ABSOLUTE earth-frame heading, NOT an offset. Passing
    0 here does not mean "keep current heading" — it commands the vehicle to
    turn and face NORTH. Callers must pass (current FC yaw + correction).
    """
    conn.mav.set_attitude_target_send(
        int((time.monotonic() - boot_time) * 1000),
        conn.target_system,
        conn.target_component,
        TYPE_MASK,
        euler_to_quat(math.radians(roll_deg), math.radians(pitch_deg), target_yaw_rad),
        0.0,  # body roll rate  — supplied-as-zero (mask 0 requires all three)
        0.0,  # body pitch rate — supplied-as-zero
        0.0,  # body yaw rate   — supplied-as-zero; yaw is carried in the quaternion
        thrust,
    )


def draw_legend(frame, title, rows):
    """Translucent titled legend box pinned to the top-right corner."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale, thick, line_h, pad = 0.5, 1, 20, 8
    texts = [title] + [f"{k:<7}: {v}" for k, v in rows]
    text_w = max(cv2.getTextSize(t, font, scale, thick)[0][0] for t in texts)
    x2, y1 = frame.shape[1] - 10, 10
    x1, y2 = x2 - (text_w + 2 * pad), y1 + (line_h * len(texts) + 2 * pad)

    overlay = frame.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.45, frame, 0.55, 0, frame)
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 1)

    y = y1 + pad + 12
    for i, t in enumerate(texts):
        color = (0, 200, 255) if i == 0 else (255, 255, 255)
        cv2.putText(frame, t, (x1 + pad, y), font, scale, color, thick, cv2.LINE_AA)
        y += line_h


def is_engaged(status, link_healthy):
    """True only when the pilot has armed AND selected GUIDED_NOGPS."""
    if status is None or not link_healthy:
        return False
    return status["armed"] and str(status["mode"]).upper() == GUIDED_NOGPS_MODE_NAME


def run(args):
    picam2 = cam.open_camera(args)

    detector = AprilTagDetector(detect_scale=args.detect_scale)

    prep = pre.Preprocessor(args)

    print(f"[vision] detect_scale={args.detect_scale}  preprocessing: {prep.describe()}")
    velocity = VelocityEstimator()   # supplies the D term — see KD_* above

    # Jello/vibration can corrupt one frame's corners into a pose metres off;
    # the D term would turn that single-frame spike into a pinned attitude
    # command. Gate implausible jumps BEFORE staleness/velocity see them —
    # rejected garbage then decays into the normal TAG_LOST neutral hover.
    pose_gate = None if args.no_pose_gate else PoseGate(args.pose_gate_max_mps)
    print("[vision] pose gate: "
          + ("OFF (diagnostics mode)" if pose_gate is None
             else f"on, max plausible speed {args.pose_gate_max_mps:g} m/s"))

    fc_link = None
    conn = None
    if not args.dry_run:
        fc_link = FlightControllerLink(device=args.mavlink_device, baud=args.mavlink_baud)
        fc_link.connect()
        conn = fc_link.raw_connection
        # Ask the FC to stream two things back at 10 Hz (read-only telemetry
        # requests — these command nothing):
        #   ATTITUDE_TARGET — what the FC is TRYING to achieve. Mirrors our
        #     SET_ATTITUDE_TARGET once engaged, so it proves command ingestion.
        #     Reads zero while disarmed/not engaged, because we send nothing then.
        #   ATTITUDE — what the FC's IMU actually MEASURES. Always live, even
        #     disarmed, so it proves the FC->Pi telemetry path independently.
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_TARGET,
                       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, 100000,  # 100 ms == 10 Hz
                0, 0, 0, 0, 0)
    else:
        print("[dry-run] No MAVLink connection — computing and displaying commands only.")

    httpd, stream_buffer = start_mjpeg_server(args.port)
    print(f"Stream live at http://{get_local_ip()}:{args.port}/stream")
    print("Pilot engages by arming + flipping the TX switch to GUIDED_NOGPS; "
          "flipping back to Stabilize disengages instantly.")
    print("Watching for AprilTag — Ctrl+C to stop.\n")

    boot_time = time.monotonic()
    send_interval = 1.0 / SEND_HZ
    last_send = 0.0
    last_heartbeat = 0.0
    last_print = 0.0
    last_gate_print = 0.0
    last_stream = 0.0
    last_detection_time = 0.0
    last_det = None
    last_vel = (0.0, 0.0, 0.0)

    # Slewed command state (what we actually stream), starts neutral.
    cmd_roll, cmd_pitch, cmd_yaw_corr, cmd_thrust = NEUTRAL_COMMANDS

    try:
        while True:
            frame = picam2.capture_array()   # 180° flip happens in hardware
            frame = prep.apply(frame)    # what we detect on IS what we stream
            now = time.monotonic()

            if fc_link is not None:
                fc_link.poll()
                if now - last_heartbeat >= 1.0:
                    fc_link.send_companion_heartbeat()
                    last_heartbeat = now

            detections = detector.detect(frame)
            if detections and (pose_gate is None or pose_gate.accept(detections[0])):
                last_det = detections[0]
                last_detection_time = now
                # Relative velocity -> the D term. The estimator resets itself
                # on re-acquisition, so a lost/regained tag can't spike this.
                last_vel = velocity.update(
                    last_det["tag_id"], last_det["fwd_m"], last_det["right_m"],
                    last_det["down_m"], last_det["timestamp"])
                # Cheap outline only — no axes/vector, per compute budget.
                pts = last_det["corners"].reshape(-1, 2).astype(np.int32)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            elif detections:
                # Pose gate rejected a spike: red outline, don't feed control.
                pts = detections[0]["corners"].reshape(-1, 2).astype(np.int32)
                cv2.polylines(frame, [pts], True, (0, 0, 255), 2)
                if now - last_gate_print >= 1.0:
                    print(f"[pose-gate] rejected implausible "
                          f"{pose_gate.last_reject_speed:.1f} m/s jump "
                          f"({pose_gate.rejected_count} total)")
                    last_gate_print = now

            tag_fresh = (now - last_detection_time) <= TAG_STALE_S and last_det is not None
            status = fc_link.get_status() if fc_link is not None else None
            link_healthy = (fc_link.is_link_healthy(LINK_STALE_S)
                            if fc_link is not None else False)
            engaged = is_engaged(status, link_healthy)

            # FC's echoed attitude TARGET — proof it's ingesting our stream.
            # Reads zero until engaged, because we send nothing until then.
            fc_echo = None
            # FC's MEASURED attitude from its IMU — live even while disarmed.
            fc_att = None
            fc_yaw_rad = None
            if fc_link is not None:
                at = fc_link.get_latest("ATTITUDE_TARGET", max_age_s=1.0)
                if at is not None:
                    fr, fp, _ = quat_to_euler(at.q)
                    fc_echo = (fr, fp, math.degrees(at.body_yaw_rate), at.thrust)
                a = fc_link.get_latest("ATTITUDE", max_age_s=ATTITUDE_STALE_S)
                if a is not None:
                    fc_yaw_rad = a.yaw
                    fc_att = (math.degrees(a.roll), math.degrees(a.pitch),
                              math.degrees(a.yaw))

            # We build the yaw target as (FC's current heading + correction), so
            # a stale ATTITUDE means we do not know the current heading. Sending
            # anyway would put a stale/zero yaw in the quaternion and command the
            # vehicle to turn to an arbitrary absolute heading (yaw 0 = NORTH).
            # So: no fresh heading => send nothing, and let ArduPilot's
            # GUID_TIMEOUT level the vehicle. Never guess a heading.
            heading_ok = fc_yaw_rad is not None

            # ── State machine ──────────────────────────────────────────────
            if not engaged:
                state = "WAITING" if fc_link is not None else "DRY-RUN"
            elif not heading_ok:
                state = "NO_HEADING"
            elif tag_fresh:
                state = "ACTIVE"
            else:
                state = "TAG_LOST"

            # Target commands for this tick
            if tag_fresh:
                target = compute_commands(last_det, last_vel, args.distance)
            else:
                target = NEUTRAL_COMMANDS

            # Slew actual commands toward the target
            cmd_roll = slew(cmd_roll, target[0], MAX_ANGLE_STEP_DEG)
            cmd_pitch = slew(cmd_pitch, target[1], MAX_ANGLE_STEP_DEG)
            cmd_yaw_corr = target[2]  # angle delta vs current heading, already clamped
            cmd_thrust = slew(cmd_thrust, target[3], MAX_THRUST_STEP)

            if not engaged:
                # Never stream targets unless the pilot has engaged; also keep
                # the slew state neutral so engagement always starts gently.
                cmd_roll, cmd_pitch, cmd_yaw_corr, cmd_thrust = NEUTRAL_COMMANDS

            # ── Send (ACTIVE and TAG_LOST stream; WAITING/NO_HEADING never) ──
            if engaged and heading_ok and (now - last_send) >= send_interval:
                # Absolute heading target = where the FC is pointing now, plus
                # our correction. Recomputed every tick against fresh telemetry.
                target_yaw_rad = fc_yaw_rad + math.radians(cmd_yaw_corr)
                send_attitude_target(conn, boot_time, cmd_roll, cmd_pitch,
                                     target_yaw_rad, cmd_thrust)
                last_send = now

            # ── Legend + terminal ──────────────────────────────────────────
            rows = [("state", state)]
            if fc_link is not None:
                if status is not None:
                    rows.append(("armed", "ARMED" if status["armed"] else "disarmed"))
                    rows.append(("mode", str(status["mode"])))
                rows.append(("link", "ok" if link_healthy else "STALE"))
            else:
                rows.append(("fc", "dry-run (no link)"))
            if last_det is not None and tag_fresh:
                rows += [
                    ("dist", f"{last_det['fwd_m']:.2f} m"),
                    ("right", f"{last_det['right_m']:+.2f} m"),
                    ("down", f"{last_det['down_m']:+.2f} m"),
                    ("skew", f"{last_det['yaw_deg']:+.1f} deg"),
                ]
            rows += [
                ("cmd", f"r{cmd_roll:+.0f} p{cmd_pitch:+.0f} "
                        f"dyaw{cmd_yaw_corr:+.0f} t{cmd_thrust:.2f}"),
            ]
            if fc_link is not None:
                if fc_echo is not None:
                    rows.append(("fc echo", f"r{fc_echo[0]:+.0f} p{fc_echo[1]:+.0f} "
                                            f"yr{fc_echo[2]:+.0f} t{fc_echo[3]:.2f}"))
                else:
                    rows.append(("fc echo", "-- none --"))
                if fc_att is not None:
                    rows.append(("fc imu", f"r{fc_att[0]:+.0f} p{fc_att[1]:+.0f} "
                                           f"y{fc_att[2]:+.0f}"))
                else:
                    rows.append(("fc imu", "-- none --"))
            draw_legend(frame, f"hover_on_tag  [{state}]", rows)
            if state == "TAG_LOST":
                cv2.putText(frame, "TAG LOST - HOLDING NEUTRAL HOVER", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            if now - last_print >= 0.1:  # 10 Hz
                if status is not None:
                    fc_str = (f"armed={'Y' if status['armed'] else 'N'} "
                              f"mode={status['mode']} | ")
                else:
                    fc_str = "" if fc_link is None else "link stale | "
                echo_str = ""
                if fc_echo is not None:
                    echo_str = (f" || FC echo r={fc_echo[0]:+.1f} p={fc_echo[1]:+.1f} "
                                f"yawrate={fc_echo[2]:+.1f} thrust={fc_echo[3]:.3f}")
                if tag_fresh:
                    print(f"[{state}] {fc_str}dist={last_det['fwd_m']:.2f}m "
                          f"right={last_det['right_m']:+.2f}m down={last_det['down_m']:+.2f}m "
                          f"skew={last_det['yaw_deg']:+.1f}deg | "
                          f"cmd r={cmd_roll:+.1f} p={cmd_pitch:+.1f} "
                          f"dyaw={cmd_yaw_corr:+.1f} thrust={cmd_thrust:.3f}{echo_str}")
                else:
                    tail = ("no tag — streaming neutral"
                            if state == "TAG_LOST" else "no tag")
                    print(f"[{state}] {fc_str}{tail}{echo_str}")
                last_print = now

            # Throttle the stream encode: it costs ~13 ms of a 33 ms frame budget
            # and a human does not need 30 fps. Detection still runs every frame.
            if now - last_stream >= STREAM_INTERVAL_S:
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    stream_buffer.push(buf.tobytes())
                last_stream = now

    except KeyboardInterrupt:
        pass
    finally:
        # Best effort: leave the FC on a neutral target on the way out. Its
        # own GUID_TIMEOUT levels the vehicle if streaming just stops.
        if conn is not None and is_engaged(fc_link.get_status(),
                                           fc_link.is_link_healthy(LINK_STALE_S)):
            send_attitude_target(conn, boot_time, *NEUTRAL_COMMANDS)
        picam2.stop()
        httpd.shutdown()
        if fc_link is not None:
            fc_link.close()


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    cam.add_camera_args(parser)
    pre.add_preprocess_args(parser)
    parser.add_argument("--detect-scale", type=float, default=0.5,
                        help="Detect on a downscaled image (corners are still "
                             "refined at full res, so accuracy is kept). 1.0 costs "
                             "71 ms/frame on the Pi and starves the loop to ~8 fps; "
                             "0.5 costs 17 ms. Default 0.5.")
    parser.add_argument("--distance", type=float, default=1.0,
                         help="Hover distance from the tag, metres. Default 1.0. "
                              "NOTE: --focus-m should match this, and changing it "
                              "changes the goal-point geometry — re-run sitl_tag_sim.py.")
    parser.add_argument("--no-pose-gate", action="store_true",
                         help="Disable the single-frame pose-spike gate "
                              "(vision/pose_filter.py). Diagnostics only — the "
                              "gate protects the D term from jello-corrupted poses.")
    parser.add_argument("--pose-gate-max-mps", type=float, default=4.0,
                         help="Pose gate's max plausible tag speed, m/s. Default 4.0.")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--dry-run", action="store_true",
                         help="No FC connection at all — vision + computed commands "
                              "+ legend only.")
    parser.add_argument("--mavlink-device", default=DEFAULT_DEVICE)
    parser.add_argument("--mavlink-baud", type=int, default=DEFAULT_BAUD)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
