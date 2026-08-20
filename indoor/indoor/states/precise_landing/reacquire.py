from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, CANCEL

from nectar.control import MavlinkDrone

from indoor import Config


class Reacquire(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT, CANCEL])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        if config.reacquire_step + drone.get_altitude() >= config.max_altitude:
            yasmin.YASMIN_LOG_ERROR('Altitude limit.')
            return CANCEL

        yasmin.YASMIN_LOG_INFO(f'Up {config.reacquire_step} m...')
        drone.move_to(
            x=0,
            y=0,
            z=config.reacquire_step,
            yaw=0,
        )

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=config.timeout) or \
            now - \
            self.start_state > Duration(seconds=config.timeout_per_state)
