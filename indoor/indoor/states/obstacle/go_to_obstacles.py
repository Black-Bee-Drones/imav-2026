from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavrosDrone, MoveReference


class GoToObstacles(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_output_key('obstacle_start_time')

        self.add_input_key('obstacle_skip')

        self.add_input_key('timeout')
        self.add_input_key('obstacle_timeout')

        self.add_input_key('start_time')
        self.add_input_key('obstacle_start_time')

        self.add_input_key('drone')

        self.add_input_key('obstacle_start_x')
        self.add_input_key('obstacle_start_y')

        self.add_input_key('obstacle_gate_alt')

    def execute(self, blackboard: Blackboard):
        blackboard.set('obstacle_start_time', self.node.get_clock().now())
        self.obstacle_skip = blackboard.get('obstacle_skip')
        if self.obstacle_skip:
            return CANCEL

        drone: MavrosDrone = blackboard.get('drone')

        self.timeout: int = blackboard.get('timeout')
        self.mission_timeout: int = blackboard.get('obstacle_timeout')

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = blackboard.get('obstacle_start_time')

        start_x: float = blackboard.get('obstacle_start_x')
        start_y: float = blackboard.get('obstacle_start_y')
        gate_alt: float = blackboard.get('obstacle_gate_alt')

        points = [
            (None, None, gate_alt),
            (start_x, None, gate_alt),
            (start_x, start_y, gate_alt),
        ]

        if self.check_timeout():
            return TIMEOUT

        for x, y, z in points:
            yasmin.YASMIN_LOG_INFO(f'Fly to x={x}; y={y}; z={z}...')
            drone.move_to(
                x=x,
                y=y,
                z=z,
                yaw=0,
                reference=MoveReference.TAKEOFF,
            )

            if self.check_timeout():
                return TIMEOUT

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
