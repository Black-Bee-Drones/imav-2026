from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import CANCEL, SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from ...config import Config


class GoToBox(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT, CANCEL])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')
        if config.droping_skip:
            return CANCEL

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout(config):
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Flying to a safe altitude...')
        drone.move_to(
            x=None,
            y=None,
            z=config.safe_altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout(config):
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f'fly to y={config.box_y}...')
        drone.move_to(
            x=None,
            y=config.box_y,
            z=config.safe_altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout(config):
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f'fly to x={config.box_x}...')
        drone.move_to(
            x=config.box_x,
            y=config.box_y,
            z=config.safe_altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout(config):
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
        return SUCCEED

    def check_timeout(self, config):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=config.timeout) or \
            now - \
            self.start_state > Duration(seconds=config.timeout_per_state)
