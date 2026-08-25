import numpy as np

from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT

from nectar.control import MavlinkDrone, PIDController
from nectar.vision import ImageHandler, Aruco

from indoor import Config

import time


class CenterMoving(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, FAIL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        pid_x: PIDController = blackboard.get('pid_x')
        pid_y: PIDController = blackboard.get('pid_y')
        pid_z: PIDController = blackboard.get('pid_z')
        pid_yaw: PIDController = blackboard.get('pid_yaw')

        image_handler_down: ImageHandler = blackboard.get('image_handler_down')
        image_handler_down.image_processing_callback = blackboard.get(
            'callback_aruco')

        # Overall mission-timeout reference (set once, outside this state)
        # vs. this state's own start time (used for the per-state timeout).
        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        pid_x.reset()
        pid_y.reset()
        pid_z.reset()
        pid_yaw.reset()

        aruco = Aruco(marker_dict=5, tag_size=1.0)

        is_yaw_aligned = False

        yasmin.YASMIN_LOG_INFO(
            'Center state started: beginning yaw alignment phase.')
        if self.check_timeout(config):
            yasmin.YASMIN_LOG_ERROR(
                'Timeout before yaw alignment could start.')
            return TIMEOUT

        # --- Phase 1: yaw alignment -----------------------------------
        # Rotate in place until the marker's yaw error is within threshold.
        lost_count = 0
        while not is_yaw_aligned:
            loop_start = self.node.get_clock().now()
            frame = image_handler_down.take_photo()
            marker_id, translation, yaw = aruco.pose_estimate(frame, draw=True)

            if marker_id is not None:
                lost_count = 0
                yaw_error = yaw

                if abs(yaw_error) <= config.yaw_threshold:
                    yasmin.YASMIN_LOG_INFO(
                        'Yaw aligned successfully; proceeding to centering phase.')
                    drone.move_velocity()
                    is_yaw_aligned = True
                    continue

                yaw_output = pid_yaw.update(yaw_error)

                yasmin.YASMIN_LOG_INFO(
                    f'Aligning yaw: yaw_error={yaw_error:.3f}; yaw_output={yaw_output:.3f}.'
                )

                drone.move_velocity(
                    vx=0.0,
                    vy=0.0,
                    vz=0.0,
                    vyaw=yaw_output
                )
            else:
                yasmin.YASMIN_LOG_ERROR(
                    f'Marker not detected during yaw alignment ({lost_count}/{config.lost_tolerance}).'
                )
                lost_count += 1

                if config.lost_tolerance <= lost_count:
                    yasmin.YASMIN_LOG_ERROR(
                        'Lost-detection tolerance exceeded during yaw alignment. Aborting.')
                    drone.move_velocity()
                    return FAIL

            if self.check_timeout(config):
                yasmin.YASMIN_LOG_ERROR('Timeout during yaw alignment.')
                drone.move_velocity()
                return TIMEOUT

            self.node.get_clock().sleep_until(loop_start + Duration(seconds=1 / 30))

        else:
            # Moving base: the marker oscillates back and forth, so we first
            # estimate its motion (velocity and turning points) before
            # syncing our descent to the moment it recrosses the center.
            yasmin.YASMIN_LOG_INFO(
                'Moving base detected: estimating marker motion before centering.')

            sample_time_1 = self.node.get_clock().now()
            frame = image_handler_down.take_photo()
            marker_id, translation_1, yaw = aruco.pose_estimate(frame)

            time.sleep(0.1)

            sample_time_2 = self.node.get_clock().now()
            frame = image_handler_down.take_photo()
            marker_id, translation_2, yaw = aruco.pose_estimate(frame)

            # Direction the marker's x-position is currently trending toward.
            slope_sign = 1 if translation_2[0] - translation_1[0] < 0 else -1

            estimated_speeds = []
            turning_point_positions = []

            yasmin.YASMIN_LOG_INFO(
                f'Beginning motion estimation over {config.estimation_cycles * 2} half-cycles.'
            )

            for cycle in range(config.estimation_cycles * 2):

                x_samples = np.array([translation_1[0], translation_2[0]])
                t_samples = np.array([sample_time_1, sample_time_2])

                # Keep sampling until the marker's x-position reverses
                # direction (i.e. we've reached a turning point).
                while slope_sign * (x_samples[-1] - x_samples[-2]) < 0:
                    loop_start = self.node.get_clock().now()
                    frame = image_handler_down.take_photo()
                    marker_id, translation, yaw = aruco.pose_estimate(frame)

                    x_samples = np.append(x_samples, translation[0])
                    t_samples = t_samples.append(t_samples, loop_start)

                slope, intercept = np.polyfit(t_samples, x_samples, 1)

                estimated_speeds.append(abs(slope))
                turning_point_positions.append(x_samples[-1])

                yasmin.YASMIN_LOG_INFO(
                    f'Motion estimation cycle {cycle + 1}/{config.estimation_cycles * 2}: '
                    f'speed={abs(slope):.3f}; turning_point={x_samples[-1]:.1f}.'
                )

                slope_sign *= -1

            marker_center = np.mean(turning_point_positions, axis=0)
            avg_speed = np.mean(estimated_speeds)

            yasmin.YASMIN_LOG_INFO(
                f'Motion estimation complete: marker_center={marker_center:.1f}; avg_speed={avg_speed:.3f}.'
            )

            # Move over the estimated center point of the marker's path.
            while True:
                frame_h, frame_w = frame.shape[:2]

                error_x = (marker_center[1] - (frame_w / 2))
                error_y = (marker_center[0] - (frame_h / 2))

                if (error_x**2 + error_y**2) <= config.center_threshold**2 and drone.get_altitude() < config.land_altitude:
                    yasmin.YASMIN_LOG_INFO(
                        'Centered over estimated marker path and below landing altitude!')
                    drone.move_velocity()
                    break

                output_x = pid_x.update(error_x)
                output_y = pid_y.update(error_y)

                is_near_center = (error_x**2 + error_y**2) <= 4 * \
                    config.center_threshold**2

                yasmin.YASMIN_LOG_INFO(
                    f'Centering over path: error_x={error_x:.0f}; error_y={error_y:.0f}; '
                    f'output_x={output_x:.1f}; output_y={output_y:.1f}; descending={is_near_center}.'
                )
                drone.move_velocity(
                    vx=output_x,
                    vy=output_y,
                    vz=config.land_speed if is_near_center else 0,
                    vyaw=0,
                )

            # Now wait for the moving marker to pass back through center and
            # sync the final drop to that moment.
            yasmin.YASMIN_LOG_INFO(
                'Waiting for marker to re-cross center to sync drop.')
            previous_x = None

            while True:
                loop_start = self.node.get_clock().now()

                frame = image_handler_down.take_photo()
                marker_id, translation, yaw = aruco.pose_estimate(
                    frame, draw=True)

                if marker_id is not None:
                    current_x = translation[0]

                    if previous_x is not None:
                        moving_towards_center = abs(
                            current_x) < abs(previous_x)
                        time_to_center = abs(current_x) / avg_speed

                        # Only drop if we're both moving toward center and
                        # will arrive there before we finish descending.
                        time_to_land = translation[2] / config.land_speed
                        if moving_towards_center and time_to_center <= time_to_land:
                            yasmin.YASMIN_LOG_INFO(
                                f'Target synced! Dropping. time_to_center={time_to_center:.2f}s '
                                f'(time_to_land={time_to_land:.2f}s).'
                            )

                            drone.move_velocity(
                                vx=0.0,
                                vy=0.0,
                                vz=config.land_speed,
                                vyaw=0.0
                            )
                            return SUCCEED

                    previous_x = current_x

                else:
                    yasmin.YASMIN_LOG_ERROR(
                        'Marker not detected while waiting for center sync; holding position.')
                    drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0)

                if self.check_timeout(config):
                    yasmin.YASMIN_LOG_ERROR('Timeout waiting for target sync.')
                    drone.move_velocity()
                    return TIMEOUT

                self.node.get_clock().sleep_until(loop_start + Duration(seconds=1 / 30))

    def check_timeout(self):
        """
        Returns True if either the overall mission timeout or this state's
        own per-state timeout has been exceeded.
        """
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=config.timeout) or \
            now - \
            self.start_state > Duration(seconds=config.timeout_per_state)
