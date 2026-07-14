from rclpy.parameter import Parameter

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone


class Reacquire(State):
    def __init__(self, step: int = 0.5):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.step = step

        self.node = YasminNode.get_instance()

        self.node.declare_parameter('max_altitude', Parameter.Type.DOUBLE)

        self.max_altitude = self.node.get_parameter('max_altitude').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard['drone']

        yasmin.YASMIN_LOG_INFO(f'Reacquire - Altitude step: {self.step} m...')

        if self.step + drone.get_altitude() >= self.max_altitude:
            yasmin.YASMIN_LOG_ERROR('Altitude limit.')
            return ABORT

        drone.move_to(
            x = 0,
            y = 0,
            z = self.step,
            yaw = 0,
        )

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED
