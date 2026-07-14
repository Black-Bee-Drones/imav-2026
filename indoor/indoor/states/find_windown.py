from rclpy.time import Time, Duration
from rclpy.parameter import Parameter

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult


class FindWindown(State):
    def __init__(self, color_window: str = 'blue'):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.color_window = color_window.strip().lower()

        self.node = YasminNode.get_instance()

        self.node.declare_parameter('find_tolerance', Parameter.Type.INTEGER)
        self.node.declare_parameter('back_speed', Parameter.Type.DOUBLE)

        self.find_tolerance = self.node.get_parameter('find_tolerance').value
        self.back_speed = self.node.get_parameter('back_speed').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        image_handler: ImageHandler = blackboard.get('image_handler')

        self.start_time: Time = blackboard['start_time']
        self.timeout = blackboard['timeout']
        self.timeout_per_state = blackboard['timeout_per_state']
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        find = 0
        while True:
            now = self.node.get_clock().now()

            result: DetectionResult = image_handler.take_photo()

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
