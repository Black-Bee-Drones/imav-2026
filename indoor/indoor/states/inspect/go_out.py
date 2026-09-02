from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from ...config import Config


class GoOut(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()


    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        self.timeout: int = config.timeout
        self.mission_timeout: int = config.inspect_timeout

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = config.inspect_start_time

        drone: MavlinkDrone = blackboard.get('drone')

        if self.check_timeout():
            blackboard.set('obstacle_skip', True)
            blackboard.set('inpect_skip', True)
            blackboard.set('dropping_skip', True)
            blackboard.set('precise_skip', True)

            blackboard.set('rtl', False)
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly through the window.')
        drone.move_to(
            x=config.inspect_go_out_x,
            y=0,
            z=0,
            yaw=0,
            reference=MoveReference.BODY
        )

        if self.check_timeout():
            return TIMEOUT

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
