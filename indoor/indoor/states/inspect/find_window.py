from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult


class FindWindow(State):
    def __init__(self, color_window: str = 'blue'):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.color_window = color_window.strip().lower()

        self.node = YasminNode.get_instance()

        self.timeout: int | float = self.node.get_parameter('timeout').value
        self.timeout_per_state: int | float = self.node.get_parameter('timeout_per_state').value

        self.find_tolerance: int | float = self.node.get_parameter('find_tolerance').value
        self.back_speed: int | float = self.node.get_parameter('back_speed').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        image_handler_front: ImageHandler = blackboard.get('image_handler_front')
        image_handler_front.image_processing_callback = blackboard.get('callback_detector_gate')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        find = 0
        while True:
            now = self.node.get_clock().now()

            result: DetectionResult = image_handler_front.take_photo()

            window = result.filter_by_class(['blue_window' if self.color_window == 'blue' else 'red_window'])

            if window:
                find += 1

                if self.find_tolerance <= find:
                    yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
                    return SUCCEED
            else:
                find = 0

            yasmin.YASMIN_LOG_INFO(f'Go Back ({find}/{self.find_tolerance})...')
            drone.move_velocity(
                vx = self.back_speed,
                vy = 0,
                vz = 0,
                vyaw = 0,
            )

            if self.check_timeout():
                yasmin.YASMIN_LOG_ERROR('Timeout.')
                return TIMEOUT

            self.node.get_clock().sleep_until(now + Duration(seconds=1/30))

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_state > Duration(seconds=self.timeout_per_state)
