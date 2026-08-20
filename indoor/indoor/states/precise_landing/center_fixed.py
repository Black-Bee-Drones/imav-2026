from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT

from nectar.control import MavlinkDrone, PIDController
from nectar.vision import ImageHandler

from indoor import Config


class CenterFixed(State):
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

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        pid_x.reset()
        pid_y.reset()
        pid_z.reset()
        pid_yaw.reset()

        yasmin.YASMIN_LOG_INFO(
            'Center state started: beginning yaw alignment phase.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR(
                'Timeout before yaw alignment could start.')
            return TIMEOUT

        lost_count = 0
        while True:
            now = self.node.get_clock().now()
            image, marker_id, translation, yaw = image_handler_down.take_photo()

            if marker_id is not None:
                lost_count = 0

                error_x, error_y, _ = translation
                error_z = drone.get_altitude() - config.land_altitude
                error_yaw = yaw

                if (error_x**2 + error_y**2) <= config.center_threshold_xy**2 and \
                        abs(drone.get_altitude()) <= config.center_threshold_z and \
                        abs(error_yaw) <= config.center_threshold_yaw:
                    drone.move_velocity()
                    yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
                    return SUCCEED

                output_x = pid_x.update(error_x)
                output_y = pid_y.update(error_y)
                output_z = pid_z.update(error_z)
                output_yaw = pid_yaw.update(error_yaw)

                yasmin.YASMIN_LOG_INFO(
                    'Error:',
                    f'x={error_x:.0f};',
                    f'y={error_y:.0f};',
                    f'z={error_z:.0f};',
                    f'yaw={error_yaw:.0f}.'
                )

                yasmin.YASMIN_LOG_INFO(
                    'Output:',
                    f'x={output_x:.1f};',
                    f'y={output_y:.1f};',
                    f'z={output_z:.1f};',
                    f'yaw={output_yaw:.1f}.'
                )

                drone.move_velocity(
                    x=output_x,
                    y=output_y,
                    z=output_z,
                    yaw=output_yaw,
                )

            else:
                yasmin.YASMIN_LOG_ERROR(
                    f'Lost detection ({lost_count}/{config.lost_tolerance}).')
                lost_count += 1

                if config.lost_tolerance <= lost_count:
                    yasmin.YASMIN_LOG_ERROR('Lost detection exceeded.')
                    drone.move_velocity()
                    return FAIL

            if self.check_timeout():
                yasmin.YASMIN_LOG_ERROR('Timeout during yaw alignment.')
                drone.move_velocity()
                return TIMEOUT

            self.node.get_clock().sleep_until(now + Duration(seconds=1 / 30))

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=config.timeout) or \
            now - \
            self.start_state > Duration(seconds=config.timeout_per_state)
