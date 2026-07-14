from rclpy.time import Time, Duration
from rclpy.parameter import Parameter

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone


class GoOut(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()

        self.node.declare_parameter('overall_max', Parameter.Type.DOUBLE)
        self.node.declare_parameter('overall_max_per_state', Parameter.Type.BOUBLE)

        self.overall_max = self.node.get_parameter('overall_max').value
        self.overall_max_per_state = self.node.get_parameter('overall_max_per_state').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        self.start_time: Time = blackboard['start_time']
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly the drone through the window.')
        drone.move_to(
            x = -1.25 - 0.2,
            y = 0,
            z = 0,
            yaw = 0,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.overall_max) or \
            now - self.start_state > Duration(seconds=self.overall_max_per_state)
