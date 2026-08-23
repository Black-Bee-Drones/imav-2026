from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference


class GoOut(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_output_key('obstacle_skip')
        self.add_output_key('inpect_skip')
        self.add_output_key('dropping_skip')
        self.add_output_key('precise_skip')

        self.add_output_key('rtl')

        self.add_input_key('timeout')
        self.add_input_key('inspect_timeout')

        self.add_input_key('start_time')
        self.add_input_key('inspect_start_time')

        self.add_input_key('drone')

        self.add_input_key('inspect_go_out_x')

    def execute(self, blackboard: Blackboard):
        self.timeout: int = blackboard.get('timeout')
        self.mission_timeout: int = blackboard.get('inspect_timeout')

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = blackboard.get('inspect_start_time')

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
            x=blackboard.get('inspect_go_out_x'),
            y=0,
            z=0,
            yaw=0,
            reference=MoveReference.BODY
        )

        if self.check_timeout():
            return TIMEOUT

        return SUCCEED

    def check_timeout(self, config):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
