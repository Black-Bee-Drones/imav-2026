from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone, MoveReference


class RedBar(State):
    def __init__(self, step: int = 3):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.step = step

        self.node = YasminNode.get_instance()

        self.timeout: int | float = self.node.get_parameter('timeout').value
        self.timeout_per_state: int | float = self.node.get_parameter('timeout_per_state').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        altitude = 0.5 + [1.2, 1.6, 1.98][self.step-1]

        yasmin.YASMIN_LOG_INFO(f'Correcting drone altitude by z={altitude:.1f} m...')
        drone.move_to(
            x = 1.25,
            y = 0,
            z = altitude,
            yaw = 0,
            reference = MoveReference.TAKEOFF,
            precision = 0.05,
        )

        self.node.get_clock().sleep_for(Duration(seconds=1))
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly over the red bar...')
        drone.move_to(
            x = 2.25,
            y = 0,
            z = altitude,
            yaw = 0,
            reference = MoveReference.TAKEOFF,
            precision = 0.05,
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
