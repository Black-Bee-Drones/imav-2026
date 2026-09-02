from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavrosDrone, MoveReference

from ...config import Config


class GoToObstacles(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        config.obstacle_start_time = self.node.get_clock().now()
        blackboard.set('obstacle_start_time', config.obstacle_start_time)
        self.obstacle_skip = config.obstacle_skip
        if self.obstacle_skip:
            return CANCEL

        drone: MavrosDrone = blackboard.get('drone')

        self.start_time: Time = blackboard.get('start_time')
        self.timeout: int = config.timeout

        self.start_mission: Time = config.obstacle_start_time
        self.mission_timeout: int = config.obstacle_timeout

        start_x: float = config.obstacle_start_x
        start_y: float = config.obstacle_start_y
        gate_alt: float = config.obstacle_gate_alt

        points = [
            (None, None, gate_alt),
            (start_x - 0.75, None, gate_alt), # safe distance
            (start_x - 0.75, start_y, gate_alt), 
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
