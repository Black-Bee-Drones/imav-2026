from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from ...config import Config


class Tubes(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()


    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        drone: MavlinkDrone = blackboard.get('drone')
        self.start_time: Time = blackboard.get('start_time')
        self.timeout: int = config.timeout
        self.start_mission: Time = config.obstacle_start_time
        self.mission_timeout: int = config.obstacle_timeout
        start_x = config.obstacle_start_x
        start_y = config.obstacle_start_y

        alt = config.obstacle_tubes_alt
        offset = config.obstacle_tubes_offset
        safe_alt = config.safe_alt

        if config.obstacle_tubes_skip:
            points = [
                (start_x+4.75, start_y, safe_alt),
                (start_x+6.25, start_y, safe_alt),
            ]
        else:
            points = [
                (start_x+4.25, start_y, alt),
                (start_x+4.25, start_y+offset, alt),
                (start_x+5.75, start_y+offset, alt),
                (start_x+5.75, start_y, alt),
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
