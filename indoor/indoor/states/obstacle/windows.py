from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone, PIDController, MoveReference
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult


class Windows(State):
    def __init__(self, color_window: str = 'blue'):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.color_window = color_window.strip().lower()

        self.node = YasminNode.get_instance()

        self.timeout: int | float = self.node.get_parameter('timeout').value
        self.timeout_per_state: int | float = self.node.get_parameter('timeout_per_state').value

        self.window_threshold: int | float = self.node.get_parameter('window_threshold').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        pid_y: PIDController = blackboard.get('pid_y')
        pid_z: PIDController = blackboard.get('pid_z')

        image_handler_front: ImageHandler = blackboard.get('image_handler_front')
        image_handler_front.image_processing_callback = blackboard.get('callback_detector_gate')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        pid_y.reset()
        pid_z.reset()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Correcting drone altitude...')
        drone.move_to(
            x = None,
            y = 0,
            z = 1.2,
            yaw = 0,
            reference = MoveReference.TAKEOFF,
            precision = 0.05,
        )

        self.node.get_clock().sleep_for(Duration(seconds=1))
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        lost = 0
        while True:
            now = self.node.get_clock().now()

            result: DetectionResult = image_handler_front.take_photo()

            window = result.filter_by_class(['blue_window' if self.color_window == 'blue' else 'red_window'])

            if window:
                lost = 0

                h, w = result.image.shape[:2]

                center = window[0].center

                error_y = (center[0] - (h / 2))
                error_z = (center[1] - (w / 2))

                if (error_y**2 + error_z**2) <= self.window_threshold**2:
                    yasmin.YASMIN_LOG_INFO('successful alignment!')
                    drone.move_velocity()
                    break

                output_y = pid_y.update(error_y)
                output_z = pid_z.update(error_z)

                yasmin.YASMIN_LOG_INFO(f'Centering: error_y={error_y:.0f}; error_z={error_z:.0f}; output_y={output_y:.1f}; output_z={output_z:.1f}.')
                drone.move_velocity(
                    vx = 0,
                    vy = output_y,
                    vz = output_z,
                    vyaw = 0,
                )
            else:
                yasmin.YASMIN_LOG_ERROR(f'Lost detection {lost}.')
                lost += 1

            if self.check_timeout():
                yasmin.YASMIN_LOG_ERROR('Timeout.')
                return TIMEOUT

            self.node.get_clock().sleep_until(now + Duration(seconds=1/30))

        self.node.get_clock().sleep_for(Duration(seconds=1))
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly the drone through the window.')
        drone.move_to(
            x = 1.25 + 0.2,
            y = 0,
            z = 0,
            yaw = 0,
        )

        self.node.get_clock().sleep_for(Duration(seconds=1))
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_state > Duration(seconds=self.timeout_per_state)
