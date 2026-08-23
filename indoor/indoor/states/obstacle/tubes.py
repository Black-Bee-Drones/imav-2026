from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference


class Tubes(State):
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

        self.add_input_key('obstacle_tubes_skip')

        self.add_input_key('obstacle_tubes_alt')
        self.add_input_key('obstacle_tubes_offset')

    def execute(self, blackboard: Blackboard):
        self.timeout: int = blackboard.get('timeout')
        self.mission_timeout: int = blackboard.get('obstacle_timeout')

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = blackboard.get('obstacle_start_time')

        drone: MavlinkDrone = blackboard.get('drone')

        safe_alt = blackboard.get('safe_alt')

        start_x = blackboard.get('obstacle_start_x')
        start_y = blackboard.get('obstacle_start_y')

        alt = blackboard.get('obstacle_tubes_alt')
        offset = blackboard.get('obstacle_tubes_offset')

        if blackboard.get('obstacle_tubes_skip'):
            points = [
                (start_x+4.75, start_y, safe_alt),
                (start_x+6.25, start_y, safe_alt),
            ]
        else:
            points = [
                (start_x+4.75, start_y, alt),
                (start_x+5.75, start_y, alt),
                (start_x+5.75, start_y+offset, alt),
                (start_x+6.25, start_y+offset, alt),
                (start_x+6.25, start_y, alt),
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
