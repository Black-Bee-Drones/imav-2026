from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference


class GoToWindow(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_output_key('inspect_start_time')

        self.add_input_key('inspect_skip')

        self.add_input_key('timeout')
        self.add_input_key('inspect_timeout')

        self.add_input_key('start_time')
        self.add_input_key('inspect_start_time')

        self.add_input_key('drone')

        self.add_input_key('inspect_start_x')
        self.add_input_key('inspect_start_y')
        self.add_input_key('inspect_start_z')

    def execute(self, blackboard: Blackboard):
        blackboard.set('inspect_start_time', self.node.get_clock().now())
        self.obstacle_skip = blackboard.get('inspect_skip')
        if self.obstacle_skip:
            return CANCEL

        self.timeout: int = blackboard.get('timeout')
        self.mission_timeout: int = blackboard.get('inspect_timeout')

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = blackboard.get('inspect_start_time')

        drone: MavlinkDrone = blackboard.get('drone')

        start_x: float = blackboard.get('inspect_start_x')
        start_y: float = blackboard.get('inspect_start_y')
        alt: float = blackboard.get('inspect_start_z')

        points = [
            (start_x - 1.5, start_y, None, None),
            (start_x - 1.5, start_y, alt, None),
            (start_x - 1.5, start_y, alt, 180),
            (start_x, start_y, alt, None)
        ]

        if self.check_timeout():
            return TIMEOUT

        for x, y, z, yaw in points:
            yasmin.YASMIN_LOG_INFO(f'Fly to x={x}; y={y}; z={z}...')
            drone.move_to(
                x=x,
                y=y,
                z=z,
                yaw=yaw,
                reference=MoveReference.TAKEOFF,
            )

            if self.check_timeout():
                return TIMEOUT

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
