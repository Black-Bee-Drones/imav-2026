import time

from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference
from nectar.control.pid import PIDController
from nectar.vision import ImageHandler

from ...config import Config


class GoToWindow(State):
    """
    1. Fly to (inspect_start_x, inspect_start_y) at a safe altitude.
        2. Descend to inspect_start_z at a fixed vertical velocity while
             centering on the ArUco marker with PID controllers.
    3. Yaw 180 degrees.
    4. Fine-align on the marker with a closed-loop PID pass on x/y.
    5. SUCCEED.
    """

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        config.inspect_start_time = self.node.get_clock().now()
        blackboard.set('inspect_start_time', config.inspect_start_time)

        self.inspect_skip = config.inspect_skip
        if self.inspect_skip:
            return CANCEL

        self.timeout: int = config.timeout
        self.mission_timeout: int = config.inspect_timeout

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = config.inspect_start_time

        drone: MavlinkDrone = blackboard.get('drone')
        handler: ImageHandler = blackboard.get('image_handler_down')
        handler.image_processing_callback = blackboard.get('callback_inspect_aruco')

        img, _, _, _ = handler.take_photo()
        if img is not None:
            height, width = img.shape[:2]
        else:
            height = width = 0.0
            yasmin.YASMIN_LOG_ERROR(f"Failed to take photo. Img height: {height} width: {width}")

        self.pid_x = PIDController(
            kp=config.obstacle_xy_kp,
            ki=config.obstacle_xy_ki,
            kd=config.obstacle_xy_kd,
            setpoint=0.0,
            output_limits=(config.controller_xy_output_min,
                           config.controller_xy_output_max),
            integral_limits=(config.controller_xy_integral_min,
                             config.controller_xy_integral_max),
        )
        self.pid_y = PIDController(
            kp=config.obstacle_xy_kp,
            ki=config.obstacle_xy_ki,
            kd=config.obstacle_xy_kd,
            setpoint=0.0,
            output_limits=(config.controller_xy_output_min,
                           config.controller_xy_output_max),
            integral_limits=(config.controller_xy_integral_min,
                             config.controller_xy_integral_max),
        )
        self.pid_yaw = PIDController(
            kp=config.controller_yaw_kp,
            ki=config.controller_yaw_ki,
            kd=config.controller_yaw_kd,
            setpoint=0.0,
            output_limits=(config.controller_yaw_output_min,
                           config.controller_yaw_output_max),
            integral_limits=(config.controller_yaw_integral_min,
                             config.controller_yaw_integral_max),
        )

        start_x: float = config.inspect_start_x
        start_y: float = config.inspect_start_y
        safe_alt: float = config.safe_alt
        self.inspect_z: float = config.inspect_start_z

        # --- 1. Go to the window approach point at a safe altitude ---
        yasmin.YASMIN_LOG_INFO(
            f'Fly to x={start_x}; y={start_y}; z={safe_alt}...')
        drone.move_to(
            x=start_x,
            y=start_y,
            z=safe_alt,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            return TIMEOUT

        # --- 2. Descend to inspection altitude while coarsely centering on
        #        the ArUco marker. ---

        yasmin.YASMIN_LOG_INFO(
            f'Descending to z={self.inspect_z} while centering on ArUco...'
        )
        self._coarse_center(drone, handler, config)

        if self.check_timeout():
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Yawing 180 degrees...')
        drone.move_to(
            x=start_x,
            y=start_y,
            z=self.inspect_z,
            yaw=180,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(
            'Starting fine PID alignment on ArUco marker...')
        if not self._pid_center_align(drone, handler, config):
            yasmin.YASMIN_LOG_WARN('Lost ArUco marker during fine alignment.')

        if self.check_timeout():
            return TIMEOUT

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()
        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)

    def _coarse_center(self, drone: MavlinkDrone, handler: ImageHandler, config: Config) -> bool:

        coarse_threshold = config.obstacle_gate_aligned_threshold * 2
        lost_count = 0

        while drone.get_altitude() > self.inspect_z:
            if self.check_timeout():
                drone.move_velocity()
                return False

            _, marker_id, translation, _ = handler.take_photo()
            if marker_id is None or translation is None:
                yasmin.YASMIN_LOG_INFO(
                    f'Lost detection {lost_count}/{config.obstacle_gate_lost_tolerance}'
                )
                lost_count += 1
                if lost_count >= config.obstacle_gate_lost_tolerance:
                    drone.move_velocity()
                    return False
                drone.move_velocity(vz=config.inspect_descent_speed)
                time.sleep(0.05)
                continue

            lost_count = 0
            error_y, error_x, _ = translation
            output_x = -self.pid_x.update(error_x)
            output_y = -self.pid_y.update(error_y)

            yasmin.YASMIN_LOG_INFO(
                f"error x={error_x} y={error_y:.0f}px"
                f"| vel x={output_x:.2f} y={output_y:.2f} m/s "
            )

            drone.move_velocity(
                vx=output_x,
                vy=output_y,
                vz=config.inspect_descent_speed,
                vyaw=0.0,
            )
            if abs(error_x) < coarse_threshold and abs(error_y) < coarse_threshold:
                yasmin.YASMIN_LOG_INFO(
                    'Coarse ArUco centering reached target.')
            time.sleep(0.05)

        drone.move_velocity()
        return True

    def _pid_center_align(self, drone: MavlinkDrone, handler: ImageHandler, config: Config) -> bool:
        aligned_count = 0
        lost_count = 0
        while aligned_count < config.obstacle_gate_aligned_tolerance:
            if self.check_timeout():
                return False

            _, marker_id, translation, yaw = handler.take_photo()
            if marker_id is None or translation is None or yaw is None:
                lost_count += 1
                if lost_count >= config.obstacle_gate_lost_tolerance:
                    return False
                time.sleep(0.05)
                continue

            lost_count = 0
            error_y, error_x, _ = translation
            output_x = -self.pid_x.update(error_x)
            output_y = -self.pid_y.update(error_y)
            output_yaw = self.pid_yaw.update(yaw)

            yasmin.YASMIN_LOG_INFO(
                f"error x={error_x} y={error_y:.0f}px yaw={yaw} deg"
                f"| vel x={output_x:.2f} y={output_y:.2f} m/s yaw={output_yaw}"
            )

            drone.move_velocity(vx=output_x, vy=output_y, vz=0.0, vyaw=output_yaw)

            if (abs(error_x) < config.obstacle_gate_aligned_threshold and
                    abs(error_y) < config.obstacle_gate_aligned_threshold):
                aligned_count += 1
            else:
                aligned_count = 0
            time.sleep(0.05)

        return True
