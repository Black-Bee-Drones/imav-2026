from datetime import datetime
import math
from pathlib import Path

import cv2

import yasmin
from yasmin import Blackboard, State
from yasmin_ros.basic_outcomes import ABORT, SUCCEED

from .constants import *
from . import constants
import manequim.core.constants as config

from nectar.control import (
    MavlinkDrone,
    MavrosDrone,
    MoveReference,
)
from nectar.ai import DetectionResult
from nectar.vision import ImageHandler


IMAGES_PATH = Path(__file__).resolve().parents[1] / 'images'


def save_photo(result: DetectionResult, state_name: str) -> None:
    if result.image is None:
        yasmin.YASMIN_LOG_WARN('Captured result has no image to save.')
        return

    IMAGES_PATH.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f')
    image_path = IMAGES_PATH / f'{state_name.lower()}-{timestamp}.png'
    annotated_path = IMAGES_PATH / f'{state_name.lower()}-{timestamp}-annotated.png'

    if not cv2.imwrite(str(image_path), result.image):
        yasmin.YASMIN_LOG_WARN(f'Failed to save captured image to {image_path}')

    annotated_image = getattr(result, 'annotated_image', None)
    if annotated_image is not None and not cv2.imwrite(str(annotated_path), annotated_image):
        yasmin.YASMIN_LOG_WARN(f'Failed to save annotated image to {annotated_path}')


class InitPosition(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.drone: MavrosDrone | MavlinkDrone | None = None

    def execute(self, blackboard: Blackboard):
        self.drone = blackboard["drone"]

        try:
            if config.SIM_MODE:
                yasmin.YASMIN_LOG_INFO("Simulation mode: skipping move to initial position.")
                return SUCCEED
            
            if constants.LATITUDE is None or constants.LONGITUDE is None:
                yasmin.YASMIN_LOG_ERROR("Initial GPS coordinates not configured.")
                return SUCCEED

            yasmin.YASMIN_LOG_INFO("Moving to initial position...")
            self.drone.move_to_gps(latitude=constants.LATITUDE, longitude=constants.LONGITUDE, precision=0.5)
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Move to initial position failed: {e}")
            return ABORT


class Ascend(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, SQUARE_SEARCH, MANEQUIM_FOUND])
        self.drone: MavrosDrone | MavlinkDrone | None = None

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Found")
            return ABORT

        self.drone = blackboard["drone"]

        if "camera" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Camera not available.")
            return ABORT
        camera: ImageHandler = blackboard["camera"]

        try:
            yasmin.YASMIN_LOG_INFO(f"Ascending to {ASCEND_HEIGHT}m for initial scan...")
            altitude = self.drone.get_altitude()
            if altitude is None:
                yasmin.YASMIN_LOG_ERROR("Unable to get current altitude.")
                return ABORT
            self.drone.move_to(x=0.0, y=0.0, z=(ASCEND_HEIGHT - altitude), reference=MoveReference.BODY)

            result: DetectionResult = camera.take_photo(timeout_sec=3.0)
            if result is None:
                yasmin.YASMIN_LOG_ERROR("Image not captured.")
                return ABORT
            save_photo(result, "ascend")

            detections = result.filter_by_class(config.DETECTOR_CLASS)
            if not detections:
                yasmin.YASMIN_LOG_INFO("Manequim not detected in Ascend State")
                return SQUARE_SEARCH

            best_conf = max(detections, key=lambda d: d.confidence)
            if best_conf.confidence < config.DETECTOR_CONFIDENCE_THRESHOLD:
                yasmin.YASMIN_LOG_INFO("Manequim not detected with enough confidence in Ascend State")
                return SQUARE_SEARCH

            return MANEQUIM_FOUND

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Ascend failed: {e}")
            return ABORT


class SearchNavigation(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, MANEQUIM_FOUND])
        self.drone: MavrosDrone | MavlinkDrone | None = None
        step = (2.0 * SEARCH_ALTITUDE * math.tan(config.CAMERA_HFOV / 2.0) - 1.0)
        self._waypoints = self._build_square_spiral(SEARCH_RADIUS, step)

        yasmin.YASMIN_LOG_INFO(
            f"SearchNavigation: {len(self._waypoints)} waypoints | "
            f"step={step:.2f}m | altitude={SEARCH_ALTITUDE}m | radius={SEARCH_RADIUS}m"
        )

    def _build_square_spiral(self, radius: float, step: float) -> list[tuple[float, float]]:
        waypoints: list[tuple[float, float]] = []
        x, y = 0.0, 0.0

        dirs = [(step, 0.0), (0.0, step), (-step, 0.0), (0.0, -step)]
        dir_idx = 0
        leg_len = 1
        legs_done = 0

        while True:
            dx, dy = dirs[dir_idx]

            for _ in range(leg_len):
                nx, ny = x + dx, y + dy
                if math.sqrt(nx ** 2 + ny ** 2) > radius:
                    return waypoints
                waypoints.append((nx, ny))
                x, y = nx, ny

            legs_done += 1
            dir_idx = (dir_idx + 1) % 4
            if legs_done % 2 == 0:
                leg_len += 1

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Found")
            return ABORT

        drone = blackboard["drone"]

        if "camera" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Camera not available.")
            return ABORT
        camera: ImageHandler = blackboard["camera"]

        try:
            yasmin.YASMIN_LOG_INFO(f"Descending to search altitude {SEARCH_ALTITUDE}m...")
            altitude = drone.get_altitude()
            if altitude is None:
                yasmin.YASMIN_LOG_ERROR("Unable to get current altitude.")
                return ABORT
            drone.move_to(
                x=0.0,
                y=0.0,
                z=(SEARCH_ALTITUDE - altitude),
                reference=MoveReference.BODY,
            )

            total = len(self._waypoints)
            prev_x, prev_y = 0.0, 0.0
            current_yaw = 0.0

            for i, (wx, wy) in enumerate(self._waypoints):
                map_dx = wx - prev_x
                map_dy = wy - prev_y
                target_yaw = math.degrees(math.atan2(map_dy, map_dx))
                yaw_delta = (target_yaw - current_yaw + 180.0) % 360.0 - 180.0

                yaw_rad = math.radians(current_yaw)
                body_x = map_dx * math.cos(yaw_rad) + map_dy * math.sin(yaw_rad)
                body_y = -map_dx * math.sin(yaw_rad) + map_dy * math.cos(yaw_rad)

                yasmin.YASMIN_LOG_INFO(
                    f"[{i + 1}/{total}] Moving to ({wx:.1f}, {wy:.1f}) @ "
                    f"{SEARCH_ALTITUDE}m | yaw={target_yaw:.1f}°"
                )
                drone.move_to(
                    x=body_x,
                    y=body_y,
                    z=0.0,
                    yaw=yaw_delta,
                    reference=MoveReference.BODY,
                )
                current_yaw = target_yaw
                prev_x, prev_y = wx, wy

                for _ in range(3):
                    result: DetectionResult = camera.take_photo(timeout_sec=3.0)
                    if result:
                        save_photo(result, "search")
                        detections = result.filter_by_class(config.DETECTOR_CLASS)
                        best = max(detections, key=lambda d: d.confidence) if detections else None
                        if best and best.confidence >= config.DETECTOR_CONFIDENCE_THRESHOLD:
                            yasmin.YASMIN_LOG_INFO(
                                f"Manequim detected at ({best.x:.1f}, {best.y:.1f}) "
                                f"with confidence {best.confidence:.2f}"
                            )
                            drone.move_velocity(
                                x=0.0,
                                y=0.0,
                                z=0.0,
                                reference=MoveReference.BODY,
                                duration=1.0,
                            )
                            return MANEQUIM_FOUND
                    else:
                        yasmin.YASMIN_LOG_WARN("Image not captured during search.")

            yasmin.YASMIN_LOG_INFO("Square search complete - manequim not found.")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"SearchNavigation failed: {e}")
            return ABORT