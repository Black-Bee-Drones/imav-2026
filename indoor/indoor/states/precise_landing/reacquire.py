from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, CANCEL

from nectar.control import MavrosDrone


class Reacquire(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT, CANCEL])

        self.node = YasminNode.get_instance()

        self.timeout: int | float = self.node.get_parameter('timeout').value
        self.timeout_per_state: int | float = self.node.get_parameter('timeout_per_state').value

        self.reacquire_step: int | float = self.node.get_parameter('reacquire_step').value
        self.max_altitude: int | float = self.node.get_parameter('max_altitude').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard['drone']

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        if self.reacquire_step + drone.get_altitude() >= self.max_altitude:
            yasmin.YASMIN_LOG_ERROR('Altitude limit.')
            return CANCEL

        yasmin.YASMIN_LOG_INFO(f'Up {self.reacquire_step} m...')
        drone.move_to(
            x = 0,
            y = 0,
            z = self.reacquire_step,
            yaw = 0,
        )

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_state > Duration(seconds=self.timeout_per_state)
