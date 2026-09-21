from rclpy.time import Time, Duration

import cv2
import numpy as np
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference
from nectar.vision import DepthCam

from ...config import Config

_MOVE_PRECISION = 0.10
_CREEP_VX = 0.12
_ROI_Y = (0.25, 0.75)
_ROI_X = (0.30, 0.70)
_DEPTH_MIN = 0.15
_DEPTH_MAX = 5.0
_CYAN = (255, 255, 0)
_GREEN = (0, 255, 0)
_FONT = cv2.FONT_HERSHEY_SIMPLEX


def _colorize_depth(depth) -> np.ndarray:
    vis = np.zeros((*depth.shape[:2], 3), dtype=np.uint8)
    valid = np.isfinite(depth) & (depth > _DEPTH_MIN) & (depth < _DEPTH_MAX)
    if not np.any(valid):
        return vis
    norm = np.clip((depth - _DEPTH_MIN) / (_DEPTH_MAX - _DEPTH_MIN), 0.0, 1.0)
    gray = np.zeros(depth.shape[:2], dtype=np.uint8)
    gray[valid] = (norm[valid] * 255.0).astype(np.uint8)
    vis = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    vis[~valid] = 0
    return vis


def _closest_forward_m(depth_cam):
    if depth_cam is None:
        return None, None, None
    try:
        frame = depth_cam.get_depth_frame()
    except Exception:
        return None, None, None
    if frame is None or getattr(frame, "size", 0) == 0:
        return None, None, None
    h, w = frame.shape[:2]
    y1, y2 = int(h * _ROI_Y[0]), int(h * _ROI_Y[1])
    x1, x2 = int(w * _ROI_X[0]), int(w * _ROI_X[1])
    roi = frame[y1:y2, x1:x2]
    valid = roi[np.isfinite(roi) & (roi > _DEPTH_MIN) & (roi < _DEPTH_MAX)]
    if valid.size < 16:
        return None, frame, (x1, y1, x2, y2)
    closest = float(np.percentile(valid, 15))
    band = valid[valid <= closest + 0.3]
    if band.size < 8:
        return closest, frame, (x1, y1, x2, y2)
    return float(np.median(band)), frame, (x1, y1, x2, y2)


def _overlay_tubes_roi(depth, rects, closest) -> np.ndarray:
    vis = _colorize_depth(depth)
    x1, y1, x2, y2 = rects
    cv2.rectangle(vis, (x1, y1), (x2, y2), _CYAN, 2)
    label = "—" if closest is None else f"{closest:.2f}m"
    cv2.putText(vis, f"tubes roi {label}", (x1, max(16, y1 - 6)), _FONT, 0.5, _GREEN, 1)
    return vis


class Tubes(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        drone: MavlinkDrone = blackboard.get("drone")

        safe_alt = config.safe_alt
        self.start_time: Time = blackboard.get("start_time")
        self.timeout: int = config.timeout

        self.start_mission: Time = config.obstacle_start_time
        self.mission_timeout: int = config.obstacle_timeout

        start_x = config.obstacle_start_x
        start_y = config.obstacle_start_y

        alt = config.obstacle_tubes_alt
        standoff = config.obstacle_tubes_standoff

        if self.check_timeout():
            return TIMEOUT

        if config.obstacle_tubes_skip:
            points = [
                (start_x + 4.75, start_y, safe_alt),
                (start_x + 6.25, start_y, safe_alt),
            ]
            return self._fly_points(drone, points)

        depth_cam = blackboard.get("depth_cam")
        handler = blackboard.get("image_handler_north")
        save_jpg = blackboard.get("save_jpg")

        if depth_cam is None and handler is not None:
            if isinstance(getattr(handler, "camera", None), DepthCam):
                depth_cam = handler.camera

        # if depth_cam is not None:
        #     if not self._creep_to_standoff(drone, depth_cam, standoff, save_jpg):
        #         return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f"Correcting altitude to: {alt}")
        drone.move_to(
            x=None,
            y=None,
            z=alt,
            reference=MoveReference.TAKEOFF
        )

        points = [
            (0.0, config.obstacle_tubes_y_avoid, 0.0),
            (config.obstacle_tubes_x_avoid, 0.0, 0.0),
            (0.0, -config.obstacle_tubes_y_avoid + 0.30, 0.0),
        ]
        return self._fly_points(drone, points)

    def _creep_to_standoff(self, drone, depth_cam, standoff: float, save_jpg) -> bool:
        yasmin.YASMIN_LOG_INFO(
            f"Tubes: optional depth creep until closest ~{standoff:.2f} m."
        )
        start = self.node.get_clock().now()
        while True:
            if self.check_timeout():
                drone.move_velocity()
                return False
            if self.node.get_clock().now() - start > Duration(seconds=60.0):
                yasmin.YASMIN_LOG_WARN("Tubes: depth creep time cap, stop.")
                drone.move_velocity()
                return True
            closest, depth, rects = _closest_forward_m(depth_cam)
            if depth is not None and rects is not None and save_jpg is not None:
                save_jpg("tubes_depth", "roi", _overlay_tubes_roi(depth, rects, closest))
            if closest is None or closest > 3.0:
                yasmin.YASMIN_LOG_INFO("Tubes: no nearby depth, skip creep.")
                drone.move_velocity()
                return True
            if closest <= standoff:
                yasmin.YASMIN_LOG_INFO(
                    f"Tubes: closest={closest:.2f} m <= {standoff:.2f} m, stop."
                )
                drone.move_velocity()
                return True
            yasmin.YASMIN_LOG_INFO(
                f"Tubes: closest={closest:.2f} m, creep vx={_CREEP_VX:.2f}."
            )
            drone.move_velocity(vx=_CREEP_VX)

    def _fly_points(self, drone, points) -> str:
        for x, y, z in points:
            yasmin.YASMIN_LOG_INFO(f"Fly to x={x}; y={y}; z={z}...")
            drone.move_to(
                x=x,
                y=y,
                z=z,
                yaw=None,
                reference=MoveReference.BODY,
                precision=_MOVE_PRECISION,
            )
            if self.check_timeout():
                return TIMEOUT
        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(
            seconds=self.timeout
        ) or now - self.start_mission > Duration(seconds=self.mission_timeout)
