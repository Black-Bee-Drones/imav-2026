from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone

from ...config import Config

class PassBlue(State):
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

        pass_x = config.obstacle_bar_pass_x

        if self.check_timeout():
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(
            f'Pass blue: +{pass_x:.1f} m body-forward.'
        )
        drone.move_to(x=pass_x, y=0.0, z=0.0, yaw=0.0, precision=0.12)
        if self.check_timeout():
            return TIMEOUT

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
