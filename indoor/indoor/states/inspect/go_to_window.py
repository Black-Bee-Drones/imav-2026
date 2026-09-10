import time

from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference
from nectar.control.pid import PIDController
from nectar.vision import Aruco, ImageHandler

from ...config import Config


class GoToWindow(State):
    """
    1. Fly to (inspect_start_x, inspect_start_y) at a safe altitude.
    2. Descend to inspect_start_z while roughly centering on the ArUco
       marker (open-loop, no PIDController).
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

        self.aruco: Aruco = blackboard.get('aruco')

        start_x: float = config.inspect_start_x
        start_y: float = config.inspect_start_y
        safe_alt: float = config.safe_alt
        inspect_z: float = config.inspect_start_z

        # --- 1. Go to the window approach point at a safe altitude ---
        yasmin.YASMIN_LOG_INFO(f'Fly to x={start_x}; y={start_y}; z={safe_alt}...')
        drone.move_to(
            x=start_x,
            y=start_y,
            z=safe_alt,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            return TIMEOUT

        # --- 2. Descend to inspection altitude, coarsely centering on the
        #        ArUco marker on the way. No PIDController here - just a
        #        fixed-gain proportional nudge to roughly line things up
        #        before we turn around. ---

        self._coarse_center(drone, handler, config)

        yasmin.YASMIN_LOG_INFO(f'Descending to z={inspect_z} while centering on ArUco...')
        drone.move_to(
            x=start_x,
            y=start_y,
            z=inspect_z,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            return TIMEOUT

        # --- 3. Yaw 180 degrees to face the window ---
        yasmin.YASMIN_LOG_INFO('Yawing 180 degrees...')
        drone.move_to(
            x=start_x,
            y=start_y,
            z=inspect_z,
            yaw=180,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            return TIMEOUT

        # --- 4. Fine alignment on the marker, closed-loop with PID on x/y ---
        yasmin.YASMIN_LOG_INFO('Starting fine PID alignment on ArUco marker...')
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
        """
        Coarse centering pass: same PID-on-pixel-error approach as
        _pid_center_align, but with a much looser aligned_threshold so it
        just gets the marker roughly in frame quickly. The fine alignment
        pass (step 4) does the precise work.

        Returns True once loosely centered, False if the marker was lost
        for `obstacle_gate_lost_tolerance` consecutive reads (or the
        mission/state timeout hit).
        """
        img = handler.take_photo()

        pid_y = PIDController(
            kp=config.obstacle_xy_kp,
            ki=config.obstacle_xy_ki,
            kd=config.obstacle_xy_kd,
            setpoint=img.shape[1] / 2,
            output_limits=(config.controller_xy_output_min, config.controller_xy_output_max),
            integral_limits=(config.controller_xy_integral_min, config.controller_xy_integral_max),
        )
        pid_z = PIDController(
            kp=config.obstacle_xy_kp,
            ki=config.obstacle_xy_ki,
            kd=config.obstacle_xy_kd,
            setpoint=img.shape[0] / 2,
            output_limits=(config.controller_xy_output_min, config.controller_xy_output_max),
            integral_limits=(config.controller_xy_integral_min, config.controller_xy_integral_max),
        )

        # Very loose acceptance window - we just want the marker in frame,
        # not centered. ASSUMPTION: 5x the fine-alignment pixel threshold.
        coarse_threshold = config.obstacle_gate_aligned_threshold * 5

        lost_count = 0

        while True:
            if self.check_timeout():
                return False

            img = handler.take_photo()
            bbox, marker_id = self.aruco.detect(img)

            if marker_id is None:
                yasmin.YASMIN_LOG_INFO(
                    f'Lost detection {lost_count}/{config.obstacle_gate_lost_tolerance}'
                )
                lost_count += 1
                if lost_count >= config.obstacle_gate_lost_tolerance:
                    return False
                time.sleep(0.05)
                continue

            lost_count = 0

            center_x = float(bbox[0][0][:, 0].mean())
            center_y = float(bbox[0][0][:, 1].mean())

            output_y = pid_y.update(center_x)
            output_z = pid_z.update(center_y)

            error_y = pid_y._last_error
            error_z = pid_z._last_error

            yasmin.YASMIN_LOG_INFO(
                f'Coarse centering | Error: y={error_y:.0f}; z={error_z:.0f}. '
                f'| Output: y={output_y:.2f}; z={output_z:.2f}.'
            )

            drone.move_velocity(0.0, vy=output_y, vz=output_z, vyaw=0.0)

            if abs(error_y) < coarse_threshold and abs(error_z) < coarse_threshold:
                return True

            time.sleep(0.05)

    def _pid_center_align(self, drone: MavlinkDrone, handler: ImageHandler, config: Config) -> bool:
        """
        Closed-loop fine alignment on the ArUco marker: PIDController on
        image-x, image-y, and yaw so the drone settles smoothly on the
        marker instead of hunting/oscillating like the coarse pass might.
        x/y setpoints are the camera frame's center (same pattern as the
        Window state); the yaw setpoint targets the marker reading squared-
        on to the camera.
        """
        img = handler.take_photo()

        pid_y = PIDController(
            kp=config.controller_xy_kp,
            ki=config.controller_xy_ki,
            kd=config.controller_xy_kd,
            setpoint=img.shape[1] / 2,
            output_limits=(config.controller_xy_output_min, config.controller_xy_output_max),
            integral_limits=(config.controller_xy_integral_min, config.controller_xy_integral_max),
        )
        pid_z = PIDController(
            kp=config.controller_xy_kp,
            ki=config.controller_xy_ki,
            kd=config.controller_xy_kd,
            setpoint=img.shape[0] / 2,
            output_limits=(config.controller_xy_output_min, config.controller_xy_output_max),
            integral_limits=(config.controller_xy_integral_min, config.controller_xy_integral_max),
        )
        pid_yaw = PIDController(
            kp=config.controller_yaw_kp,
            ki=config.controller_yaw_ki,
            kd=config.controller_yaw_kd,
            setpoint=180.0,  # ASSUMPTION: marker reads 180 deg when squared-on to the camera
            output_limits=(config.controller_yaw_output_min, config.controller_yaw_output_max),
            integral_limits=(config.controller_yaw_integral_min, config.controller_yaw_integral_max),
        )

        aligned_count = 0
        lost_count = 0

        while aligned_count < config.obstacle_gate_aligned_tolerance:
            if self.check_timeout():
                return False

            img = handler.take_photo()
            bbox, marker_id = self.aruco.detect(img)

            if marker_id is None:
                lost_count += 1
                if lost_count >= config.obstacle_gate_lost_tolerance:
                    return False
                time.sleep(0.05)
                continue

            lost_count = 0

            center_x = float(bbox[0][0][:, 0].mean())
            center_y = float(bbox[0][0][:, 1].mean())
            yaw_deg = self.aruco.calculateYawFromCorners(bbox)

            output_y = pid_y.update(center_x)
            output_z = pid_z.update(center_y)
            output_yaw = pid_yaw.update(yaw_deg)   # rad/s

            error_y = pid_y._last_error
            error_z = pid_z._last_error

            yasmin.YASMIN_LOG_INFO(
                f'Aligning: ({aligned_count}/{config.obstacle_gate_aligned_tolerance}) '
                f'| Error: y={error_y:.0f}; z={error_z:.0f}; yaw={yaw_deg:.1f}. '
                f'| Output: y={output_y:.2f}; z={output_z:.2f}; vyaw={output_yaw:.3f}.'
            )

            drone.move_velocity(0.0, vy=output_y, vz=output_z, vyaw=output_yaw)

            if abs(error_y) < config.obstacle_gate_aligned_threshold and abs(error_z) < config.obstacle_gate_aligned_threshold:
                aligned_count += 1
            else:
                aligned_count = 0

            time.sleep(0.05)

        return True