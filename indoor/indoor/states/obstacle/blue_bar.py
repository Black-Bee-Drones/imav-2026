from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference


class BlueBar(State):
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

        self.add_input_key('obstacle_blue_step_alt_1')
        self.add_input_key('obstacle_blue_step_alt_2')
        self.add_input_key('obstacle_blue_step_alt_3')

        self.add_input_key('obstacle_blue_1')
        self.add_input_key('obstacle_blue_2')

    def execute(self, blackboard: Blackboard):
        self.timeout: int = blackboard.get('timeout')
        self.mission_timeout: int = blackboard.get('obstacle_timeout')

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = blackboard.get('obstacle_start_time')

        drone: MavlinkDrone = blackboard.get('drone')

        start_x: float = blackboard.get('obstacle_start_x')
        start_y: float = blackboard.get('obstacle_start_y')

        match blackboard.get('obstacle_blue_1'):
            case 1:
                alt_1 = blackboard.get('obstacle_blue_step_alt_1')
            case 2:
                alt_1 = blackboard.get('obstacle_blue_step_alt_2')
            case 3:
                alt_1 = blackboard.get('obstacle_blue_step_alt_3')
            case _:
                alt_1: float = blackboard.get('safe_alt')

        match blackboard.get('obstacle_blue_2'):
            case 1:
                alt_2 = blackboard.get('obstacle_blue_step_alt_1')
            case 2:
                alt_2 = blackboard.get('obstacle_blue_step_alt_2')
            case 3:
                alt_2 = blackboard.get('obstacle_blue_step_alt_3')
            case _:
                alt_2: float = blackboard.get('safe_alt')

        points = [
            (start_x+2.25, start_y, alt_1),
            (start_x+3.25, start_y, alt_1),
            (start_x+3.25, start_y, alt_2),
            (start_x+4.25, start_y, alt_2),
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
