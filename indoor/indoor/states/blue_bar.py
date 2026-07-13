from rclpy.time import Time, Duration
from rclpy.parameter import Parameter

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone, MoveReference


class BlueBar(State):
    def __init__(self, step=3):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.step = step

        self.node = YasminNode.get_instance()

        self.node.declare_parameter('px_threshold', Parameter.Type.DOUBLE)

        self.px_threshold = self.node.get_parameter('px_threshold').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        self.start_time: Time = blackboard['start_time']
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        z = [.4, .8, 1.2][self.step-1] / 2

        yasmin.YASMIN_LOG_INFO(f'Correcting drone altitude by z={z:.1f} m...')
        drone.move_to(
            x = None,
            y = 0,
            z = z,
            yaw = 0,
            reference = MoveReference.TAKEOFF,
            precision = 0.05,
        )

        self.node.get_clock().sleep_for(Duration(seconds=1))
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly under the blue bar...')
        drone.move_to(
            x = 1 + 0.2,
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

        return now - self.start_time > Duration(seconds=self.overall_max) or \
            now - self.start_state > Duration(seconds=self.overall_max_per_state)
