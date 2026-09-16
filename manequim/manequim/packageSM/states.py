import math

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from .constants import *

from nectar.control import (
    MavrosDrone,
    MavlinkDrone,
    MoveReference,
    PIDController,
)
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult

from manequim.core.constants import *

class AlignState(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, LOST_PERSON, ALIGNMENT_FAILED])

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone not available for package alignment.")
            return ABORT

        if "camera" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Camera not available for package alignment.")
            return ABORT

        if "pid_cx" not in blackboard or "pid_cy" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("PID controllers not available for package alignment.")
            return ABORT

        drone: MavrosDrone | MavlinkDrone = blackboard["drone"]
        camera: ImageHandler = blackboard["camera"]
        pid_cx: PIDController = blackboard["pid_cx"]
        pid_cy: PIDController = blackboard["pid_cy"]
        
        photo_fail = 0
        lost_count = 0
        aligned_counter = 0

        try:
            while True:
                yasmin.YASMIN_LOG_INFO("Aligning package over target...")
                lost_count += 1
                
                for _ in range(3):
                    result: DetectionResult = camera.take_photo()
                    if result is None:
                        photo_fail += 1
                        if photo_fail >= PHOTO_FAIL_THRESHOLD:
                            yasmin.YASMIN_LOG_ERROR("Camera failed repeatedly while aligning package.")
                            return ALIGNMENT_FAILED
                        return LOST_PERSON

                detections = result.filter_by_class(DETECTOR_CLASS)
                if not detections:
                    lost_count += 1
                    if lost_count >= LOST_THRESHOLD:
                        yasmin.YASMIN_LOG_ERROR("Package lost after multiple attempts.")
                        return LOST_PERSON
                    yasmin.YASMIN_LOG_WARN("No package detected. Continuing...")
                    continue

                best = max(detections, key=lambda d: d.confidence)
                if best.confidence < 0.5:
                    yasmin.YASMIN_LOG_WARN("Target confidence below threshold during alignment.")
                    return LOST_PERSON
                
                target_x, target_y = best.center
                
                error_x_px = target_x - (camera.width // 2)
                error_y_px = target_y - (camera.height // 2)

                altitude = drone.get_altitude()
                
                error_x_m = self.ppm(error_x_px, altitude, CAMERA_HFOV, IMAGE_WIDTH)
                error_y_m = self.ppm(error_y_px, altitude, CAMERA_VFOV, IMAGE_HEIGHT)
                
                vx = pid_cx.update(error_x_m)
                vy = pid_cy.update(error_y_m)
                
                drone.move_velocity(x=vx, y=vy, z=0.0, frame=MoveReference.BODY, duration=1.0)

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Package alignment failed: {e}")
            return ALIGNMENT_FAILED
        
    def ppm(self, delta_pixel: int, altitude: float, fov_degrees: float, frame_px: int) -> float:
        angle_rad = math.radians(fov_degrees) / 2
        ratio = (math.tan(angle_rad) * altitude) / (frame_px // 2)
        return delta_pixel * ratio
    
class DescendState(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "ALIGN", ALIGNMENT_FAILED])

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone not available for descent.")
            return ABORT

        drone: MavrosDrone | MavlinkDrone = blackboard["drone"]
        phase = blackboard["descend_phase"] if "descend_phase" in blackboard else "approach"

        try:
            if phase == "approach":
                altitude = drone.get_altitude()
                if altitude is not None and altitude <= DROP_HEIGHT + 0.2:
                    yasmin.YASMIN_LOG_INFO("Already near drop height. Skipping approach descent.")
                else:
                    approach_altitude = max(DROP_HEIGHT + 0.25, 1.0)
                    yasmin.YASMIN_LOG_INFO(
                        f"Descending to approach altitude {approach_altitude}m before alignment..."
                    )
                    drone.move_to(x=0.0, y=0.0, z=approach_altitude, reference=MoveReference.BODY)

                blackboard["descend_phase"] = "final"
                yasmin.YASMIN_LOG_INFO("Initial descent complete. Ready to align.")
                return "ALIGN"

            yasmin.YASMIN_LOG_INFO(f"Descending to drop altitude {DROP_HEIGHT}m...")
            drone.move_to(x=0.0, y=0.0, z=DROP_HEIGHT, reference=MoveReference.BODY)
            blackboard["descend_phase"] = "approach"
            yasmin.YASMIN_LOG_INFO("Final descent complete. Ready to drop.")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Package descent failed: {e}")
            return ABORT


class ReestablishState(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, LOST_PERSON])

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone not available for re-establishing package alignment.")
            return ABORT

        if "camera" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Camera not available for re-establishing package alignment.")
            return LOST_PERSON

        yasmin.YASMIN_LOG_INFO("Re-establishing package alignment.")
        drone: MavrosDrone | MavlinkDrone = blackboard["drone"]
        camera: ImageHandler = blackboard["camera"]

        try:
            current_altitude = drone.get_altitude()
            target_altitude = REESTABILISH_ALTITUDE_INCREMENT

            if current_altitude is None or current_altitude < target_altitude:
                yasmin.YASMIN_LOG_INFO(f"Rising to {target_altitude}m to re-acquire target.")
                drone.move_to(x=0.0, y=0.0, z=target_altitude, reference=MoveReference.BODY)

            search_steps = [1.0, -1.0, 1.4, -1.4]
            for step in search_steps:
                yasmin.YASMIN_LOG_INFO(f"Re-establish search step: move x={step:.2f}m.")
                drone.move_to(x=step, y=0.0, z=0.0, reference=MoveReference.BODY)

                for _ in range(3):
                    result = camera.take_photo(timeout_sec=3.0)
                    if result is not None:
                        detections = result.filter_by_class(DETECTOR_CLASS)
                        if detections:
                            yasmin.YASMIN_LOG_INFO("Target found during re-establish search.")
                            return SUCCEED

            yasmin.YASMIN_LOG_WARN("Target still not found during re-establish search.")
            return LOST_PERSON

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Re-establish failed: {e}")
            return ABORT


class DropState(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, DROP_RETRY])

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone | MavrosDrone | None = blackboard.get("drone")
        if drone is None:
            yasmin.YASMIN_LOG_ERROR("Drone not available for dropping package.")
            return ABORT

        try:
            altitude = drone.get_altitude()
            if altitude is None:
                yasmin.YASMIN_LOG_ERROR("Unable to retrieve drone altitude for drop.")
                return ABORT
            yasmin.YASMIN_LOG_INFO(f"Dropping package at {altitude}m...")
            drone.move_velocity(x=0.0, y=0.0, z=0.0, reference=MoveReference.BODY, duration=1.0)

            retries = DROP_MAX_RETRIES
            for attempt in range(retries):
                yasmin.YASMIN_LOG_INFO(f"Attempting to drop the package (Attempt {attempt + 1}/{retries})...")
                if drone.do_servo(aux_out=SERVO_CHANNEL, value=SERVO_OPEN_PWM):
                    yasmin.YASMIN_LOG_INFO("Package dropped successfully.")
                    return SUCCEED

                yasmin.YASMIN_LOG_WARN("Package drop failed. Retrying...")
                drone.delay(RETRY_DELAY)

            yasmin.YASMIN_LOG_ERROR("Failed to drop the package after all attempts.")
            return DROP_RETRY

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN("Package drop interrupted by user.")
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Package drop failed: {e}")
            return ABORT
        
