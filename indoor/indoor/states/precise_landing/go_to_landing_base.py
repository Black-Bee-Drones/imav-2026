from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from ...config import Config


class GoToLandingBase(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        config.precise_start_time = self.node.get_clock().now()
        blackboard.set('precise_start_time', config.precise_start_time)
        if config.precise_skip:
            return CANCEL

        drone: MavlinkDrone = blackboard.get('drone')

        safe_alt: float = config.safe_alt
        self.start_time: Time = blackboard.get('start_time')
        self.timeout: int = config.timeout

        self.start_mission: Time = config.precise_start_time
        self.mission_timeout: int = config.precise_timeout

        fixed: bool = config.precise_fixed
        if fixed:
            yasmin.YASMIN_LOG_INFO(f'Fly to a "fixed" landing base...')
            base_x: float = config.precise_fixed_x
            base_y: float = config.precise_fixed_y
        else:
            yasmin.YASMIN_LOG_INFO(f'Fly to a "mobile" landing base...')
            base_x: float = config.precise_mobile_x
            base_y: float = config.precise_mobile_y

        points = [
            (None, None, safe_alt),
            (base_x, base_y, safe_alt),
        ]

        for x, y, z in points:
            if self.check_timeout():
                return TIMEOUT

            yasmin.YASMIN_LOG_INFO(f'Fly to x={x}; y={y}; z={z}...')
            drone.move_to(
                x=x,
                y=y,
                z=z,
                yaw=0,
                reference=MoveReference.TAKEOFF,
            )

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
