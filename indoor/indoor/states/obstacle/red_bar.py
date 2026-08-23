from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference


class RedBar(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_input_key('timeout')
        self.add_input_key('obstacle_timeout')

        self.add_input_key('start_time')
        self.add_input_key('obstacle_start_time')

        self.add_input_key('drone')

        self.add_input_key('safe_alt')

        self.add_input_key('obstacle_start_x')
        self.add_input_key('obstacle_start_y')

        self.add_input_key('obstacle_red_step_alt_1')
        self.add_input_key('obstacle_red_step_alt_2')
        self.add_input_key('obstacle_red_step_alt_3')

        self.add_input_key('obstacle_red')

    def execute(self, blackboard: Blackboard):
        self.timeout: int = blackboard.get('timeout')
        self.mission_timeout: int = blackboard.get('obstacle_timeout')

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = blackboard.get('obstacle_start_time')

        drone: MavlinkDrone = blackboard.get('drone')

        start_x = blackboard.get('obstacle_start_x')
        start_y = blackboard.get('obstacle_start_y')

        match blackboard.get('obstacle_red'):
            case 1:
                alt = blackboard.get('obstacle_red_step_alt_1')
            case 2:
                alt = blackboard.get('obstacle_red_step_alt_2')
            case 3:
                alt = blackboard.get('obstacle_red_step_alt_3')
            case _:
                alt = blackboard.get('safe_alt')

        points = [
            (start_x+1.25, start_y, alt),
            (start_x+2.25, start_y, alt),
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

    def check_timeout(self, config):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
