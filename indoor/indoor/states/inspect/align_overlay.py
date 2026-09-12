"""Debug-overlay drawing for ArUco-based alignment states.

Mirrors the visual style already used for the gate/box detector debug
frames: a small green monospace HUD in the top-left corner, plus a
camera-center marker, the detected marker's pixel center, and an error
vector drawn between them. Meant to be called from the same place that
already draws the marker pose (`Aruco.pose_estimate(image, draw=True)`)
so the extra overlay lands on the same annotated frame that gets saved
to disk.
"""
from __future__ import annotations

import cv2 as cv
import numpy as np

_FONT = cv.FONT_HERSHEY_SIMPLEX
_FONT_SCALE = 0.5
_FONT_THICKNESS = 1
_GREEN = (0, 255, 0)
_BLACK = (0, 0, 0)
_CYAN = (255, 255, 0)
_MAGENTA = (255, 0, 255)


def build_detector(marker_dict) -> cv.aruco.ArucoDetector:
    """Build a throwaway ArucoDetector just for pixel-center lookup.

    Accepts whatever form `config.aruco_marker_dict` is already in, since
    that's also what gets passed straight into Aruco(marker_dict=...):
    - a string name (e.g. "DICT_4X4_50")
    - an int/cv.aruco predefined-dictionary constant (e.g. cv.aruco.DICT_4X4_50)
    - an already-built cv.aruco.Dictionary object
    """
    if isinstance(marker_dict, str):
        dictionary = cv.aruco.getPredefinedDictionary(getattr(cv.aruco, marker_dict))
    elif isinstance(marker_dict, cv.aruco.Dictionary):
        dictionary = marker_dict
    else:
        dictionary = cv.aruco.getPredefinedDictionary(marker_dict)
    params = cv.aruco.DetectorParameters()
    return cv.aruco.ArucoDetector(dictionary, params)


def _put_hud_line(image: np.ndarray, text: str, line: int) -> None:
    origin = (8, 20 + line * 18)
    # Black outline first for legibility over busy backgrounds, green fill
    # on top — same trick as the gate/box detector overlays.
    cv.putText(image, text, origin, _FONT, _FONT_SCALE, _BLACK, _FONT_THICKNESS + 2, cv.LINE_AA)
    cv.putText(image, text, origin, _FONT, _FONT_SCALE, _GREEN, _FONT_THICKNESS, cv.LINE_AA)


def marker_pixel_center(image: np.ndarray, detector: cv.aruco.ArucoDetector):
    """Re-detect corners purely for visualization and return the pixel centroid.

    Returns None if no marker is found. This duplicates the detection that
    already happens inside Aruco.pose_estimate() — cheap relative to a
    single 30Hz control loop, and keeps this module independent of the
    Aruco class internals.
    """
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None
    pts = corners[0].reshape(-1, 2)
    return float(pts[:, 0].mean()), float(pts[:, 1].mean())


def draw_align_overlay(
    image: np.ndarray,
    detector: cv.aruco.ArucoDetector | None,
    marker_id,
    error_yaw,
    debug: dict | None,
) -> None:
    """Draw camera-center / marker-center / error-vector plus a PID HUD.

    Mutates `image` in place, same convention as pose_estimate(draw=True).
    `debug` is whatever the calling state last published to the blackboard
    under the 'align_debug' key (see center_fixed.py / reacquire.py) — it
    will be one control-loop tick behind the frame being drawn, which is
    fine for a debug HUD.
    """
    h, w = image.shape[:2]
    cam_center = (w // 2, h // 2)
    cv.drawMarker(image, cam_center, _CYAN, cv.MARKER_CROSS, 20, 2)
    cv.circle(image, cam_center, 6, _CYAN, 2)

    marker_center = None
    if marker_id is not None and detector is not None:
        marker_center = marker_pixel_center(image, detector)

    if marker_center is not None:
        mc = (int(marker_center[0]), int(marker_center[1]))
        cv.circle(image, mc, 8, _MAGENTA, -1)
        cv.arrowedLine(image, cam_center, mc, _GREEN, 2, tipLength=0.15)

    if not debug:
        _put_hud_line(image, f'aruco id={marker_id}', 0)
        return

    ex = debug.get('error_x', 0.0)
    ey = debug.get('error_y', 0.0)
    ez = debug.get('error_z', 0.0)
    vx = debug.get('output_x', 0.0)
    vy = debug.get('output_y', 0.0)
    vz = debug.get('output_z', 0.0)
    vyaw = debug.get('output_yaw', 0.0)
    lost = debug.get('lost_count', 0)
    lost_tol = debug.get('lost_tolerance', 0)
    within_xy = int(bool(debug.get('within_xy', False)))
    landing = int(bool(debug.get('landing', False)))
    yaw_txt = f'{error_yaw:+.1f}' if error_yaw is not None else '--'

    _put_hud_line(
        image,
        f'aruco {lost}/{lost_tol}  ex={ex:+.2f}m ey={ey:+.2f}m ez={ez:+.2f}m yaw={yaw_txt}deg',
        0,
    )
    _put_hud_line(
        image,
        f'vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} vyaw={vyaw:+.2f}  xy_ok={within_xy} land={landing}',
        1,
    )