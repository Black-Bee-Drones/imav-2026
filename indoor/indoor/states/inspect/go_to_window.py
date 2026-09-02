from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from ...config import Config


class GoToWindow(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        config.inspect_start_time = self.node.get_clock().now()
        blackboard.set('inspect_start_time', config.inspect_start_time)
        self.obstacle_skip = config.inspect_skip
        if self.obstacle_skip:
            return CANCEL

        self.timeout: int = config.timeout
        self.mission_timeout: int = config.inspect_timeout

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = config.inspect_start_time

        drone: MavlinkDrone = blackboard.get('drone')

        start_x: float = config.inspect_start_x
        start_y: float = config.inspect_start_y
        alt: float = config.inspect_start_z

        points = [
            (start_x - 1.5, start_y, None, None),
            (start_x - 1.5, start_y, alt, None),
            (start_x - 1.5, start_y, alt, 180),
            (start_x, start_y, alt, None)
        ]

        if self.check_timeout():
            return TIMEOUT

        for x, y, z, yaw in points:
            yasmin.YASMIN_LOG_INFO(f'Fly to x={x}; y={y}; z={z}...')
            drone.move_to(
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                reference=MoveReference.TAKEOFF,
            )

            if self.check_timeout():
                return TIMEOUT

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
