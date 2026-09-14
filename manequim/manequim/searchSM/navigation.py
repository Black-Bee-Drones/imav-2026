import math
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from searchSM.constants import *
from core.constants import *

from nectar.control import(
    DroneFactory,
    MavrosConfig,
    MavrosDrone,
    MavlinkDrone,
    PoseSource,
    MoveReference,
    RTLMethod,
)

from nectar.ai import DetectionResult

class InitPosition(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.drone: MavrosDrone | MavlinkDrone

    def execute(self, blackboard: Blackboard):
        self.drone = blackboard["drone"]

        try:
            yasmin.YASMIN_LOG_INFO("Moving to initial position...")
            self.drone.move_to_gps(latitude=LATITUDE, longitude=LONGITUDE, precision=0.5)
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Move to initial position failed: {e}")
            return ABORT


class Ascend(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "SQUARE_SEARCH", "FOUND_MANEQUIM"])

        self.drone: MavrosDrone | MavlinkDrone
        self.detector

    def execute(self, blackboard: Blackboard):

        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT

        if config.DRONE_TYPE == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')

        elif config.DRONE_TYPE == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')

        if "camera" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Camera not available.")
            return ABORT
        camera: ImageHandler = blackboard["camera"]

        try:
            yasmin.YASMIN_LOG_INFO(f"Ascending to {ASCEND_HEIGHT}m for initial scan...")
            self.drone.move_to(x=0.0, y=0.0, z=(ASCEND_HEIGHT - TAKEOFF_HEIGHT), frame=MoveReference.BODY)

            result: DetectionResult = camera.take_photo()

            if result is None:
                yasmin.YASMIN_LOG_ERROR("Image not captured.")
                return ABORT

            detections = result.filter_by_class(DETECTOR_CLASS)

            if not detections:
                yasmin.YASMIN_LOG_INFO("Manequim not detected in Ascend State")
                return SQUARE_SEARCH

            best_conf = max(detections, key=lambda d: d.confidence)

            if best_conf.confidence < DETECTED_CONFIDENCE_THRESHOLD:
                yasmin.YASMIN_LOG_INFO("Manequim not detected with enough confidence in Ascend State")
                return SQUARE_SEARCH

            return MANEQUIM_FOUND

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Ascend failed: {e}")
            return ABORT


class SearchNavigation(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "FOUND_MANEQUIM"])

        self.drone: MavrosDrone | MavlinkDrone

        step = (2.0 * SEARCH_ALTITUDE * math.tan(CAMERA_HFOV / 2.0) - 1.0) #Calculo de quantos metros a camera pega usando o FOV (margem de 1.0m)
        self._waypoints = self._build_square_spiral(SEARCH_RADIUS, step)

        yasmin.YASMIN_LOG_INFO(
            f"SearchNavigation: {len(self._waypoints)} waypoints | "
            f"step={step:.2f}m | altitude={SEARCH_ALTITUDE}m | radius={SEARCH_RADIUS}m"
        )

    def _build_square_spiral(radius: float, step: float) -> list[tuple[float, float]]:
        
        waypoints: list[tuple[float, float]] = []
        x, y = 0.0, 0.0

        dirs = [(step, 0.0), (0.0, step), (-step, 0.0), (0.0, -step)]
        dir_idx   = 0  
        leg_len   = 1   
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

        return waypoints

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT

        if config.DRONE_TYPE == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')

        elif config.DRONE_TYPE == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')

        if "camera" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Camera not available.")
            return ABORT
        camera: ImageHandler = blackboard["camera"]

        try:
            yasmin.YASMIN_LOG_INFO(
                f"Descending to search altitude {SEARCH_ALTITUDE}m..."
            )
            self.drone.move_to(
                x=0.0, y=0.0, z=SEARCH_ALTITUDE,
                frame=MoveReference.TAKEOFF,
            )

            total = len(self._waypoints)
            prev_x, prev_y = 0.0, 0.0

            for i, (wx, wy) in enumerate(self._waypoints):
                yaw = math.degrees(math.atan2(wy - prev_y, wx - prev_x))
                
                yasmin.YASMIN_LOG_INFO(
                    f"[{i+1}/{total}] Moving to ({wx:.1f}, {wy:.1f}) @ {SEARCH_ALTITUDE}m | yaw={yaw:.1f}°"
                )
                self.drone.move_to(
                    x=wx, y=wy, z=SEARCH_ALTITUDE,
                    yaw=yaw,
                    frame=MoveReference.TAKEOFF,
                )
                prev_x, prev_y = wx, wy

                result: DetectionResult = camera.take_photo()
                if result:
                    detections = result.filter_by_class(DETECTOR_CLASS)
                    if detections:
                        yasmin.YASMIN_LOG_INFO("Manequim detectado na espiral!")
                        return "FOUND_MANEQUIM"

            yasmin.YASMIN_LOG_INFO("Square search complete — manequim not found.")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"SearchNavigation failed: {e}")
            return ABORT

