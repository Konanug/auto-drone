"""Camera-frame -> ArduPilot body-frame (FRD) pose conversion.

Camera frame (OpenCV, after 180° flip for upside-down mount):
    X = right,  Y = down,  Z = forward

ArduPilot body frame (FRD):
    X = forward (nose),  Y = right,  Z = down

For a front-facing camera the mapping is:
    body X = cam Z,  body Y = cam X,  body Z = cam Y
"""
import cv2
import numpy as np

R_CAM_TO_BODY = np.array([
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0],
], dtype=np.float64)


def pose_to_ardupilot(rvec, tvec):
    """
    Returns position (fwd, right, down) in metres and ZYX Euler angles
    (yaw, pitch, roll) in degrees, all in ArduPilot FRD body frame.
    """
    fwd   = float(tvec[2])
    right = float(tvec[0])
    down  = float(tvec[1])

    R_cam, _ = cv2.Rodrigues(rvec)
    R_body = R_CAM_TO_BODY @ R_cam @ R_CAM_TO_BODY.T

    # ZYX decomposition — matches ArduPilot convention:
    #   yaw   positive = clockwise from above (right turn)
    #   pitch positive = nose up
    #   roll  positive = right wing down
    pitch = np.degrees(np.arctan2(-R_body[2, 0],
                                   np.sqrt(R_body[2, 1]**2 + R_body[2, 2]**2)))
    yaw   = np.degrees(np.arctan2(R_body[1, 0], R_body[0, 0]))
    roll  = np.degrees(np.arctan2(R_body[2, 1], R_body[2, 2]))

    return fwd, right, down, yaw, pitch, roll
