from rclpy.time import Time, Duration

import numpy as np
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference
from nectar.vision import DepthCam

from ...config import Config

_MOVE_PRECISION = 0.12
_CREEP_VX = 0.12


def _closest_forward_m(depth_cam) -> float | None:
    if depth_cam is None:
        return None
    try:
        frame = depth_cam.get_depth_frame()
    except Exception:
        return None
    if frame is None or getattr(frame, "size", 0) == 0:
        return None
    h, w = frame.shape[:2]
    roi = frame[int(h * 0.25) : int(h * 0.75), int(w * 0.30) : int(w * 0.70)]
    valid = roi[np.isfinite(roi) & (roi > 0.15) & (roi < 5.0)]
    if valid.size < 16:
        return None
    closest = float(np.percentile(valid, 15))
    band = valid[valid <= closest + 0.3]
    if band.size < 8:
        return closest
    return float(np.median(band))


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
        offset = config.obstacle_tubes_offset
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
        if depth_cam is None and handler is not None:
            if isinstance(getattr(handler, "camera", None), DepthCam):
                depth_cam = handler.camera

        if depth_cam is not None:
            if not self._creep_to_standoff(drone, depth_cam, standoff):
                return TIMEOUT

        # Left first at current altitude, then past the cluster, then partial
        # right (not lane center — that is tube_rear Y), then climb.
        left_y = start_y + offset
        end_y = start_y + config.obstacle_tubes_end_offset
        past_x = start_x + 5.75
        points = [
            (None, left_y, None),
            (past_x, left_y, None),
            (past_x, end_y, None),
            (None, None, alt),
        ]
        return self._fly_points(drone, points)

    def _creep_to_standoff(self, drone, depth_cam, standoff: float) -> bool:
        yasmin.YASMIN_LOG_INFO(
            f"Tubes: optional depth creep until closest ~{standoff:.2f} m."
        )
        start = self.node.get_clock().now()
        while True:
            if self.check_timeout():
                drone.move_velocity()
                return False
            if self.node.get_clock().now() - start > Duration(seconds=8.0):
                yasmin.YASMIN_LOG_WARN("Tubes: depth creep time cap, stop.")
                drone.move_velocity()
                return True
            closest = _closest_forward_m(depth_cam)
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
                reference=MoveReference.TAKEOFF,
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
