from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference


class GoToLandingBase(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_output_key('precise_start_time')

        self.add_input_key('drone')

        self.add_input_key('safe_alt')
        self.add_input_key('start_time')
        self.add_input_key('timeout')

        self.add_input_key('precise_skip')
        self.add_input_key('precise_start_time')
        self.add_input_key('precise_timeout')

        self.add_input_key('precise_fixed')
        self.add_input_key('precise_fixed_x')
        self.add_input_key('precise_fixed_y')
        self.add_input_key('precise_mobile_x')
        self.add_input_key('precise_mobile_y')

    def execute(self, blackboard: Blackboard):
        blackboard.set('precise_start_time', self.node.get_clock().now())
        if blackboard.get('precise_skip'):
            return CANCEL

        drone: MavlinkDrone = blackboard.get('drone')

        safe_alt: float = blackboard.get('safe_alt')
        self.start_time: Time = blackboard.get('start_time')
        self.timeout: int = blackboard.get('timeout')

        self.start_mission: Time = blackboard.get('precise_start_time')
        self.mission_timeout: int = blackboard.get('precise_timeout')

        fixed: bool = blackboard.get('precise_fixed')
        if fixed:
            yasmin.YASMIN_LOG_INFO(f'Fly to a "fixed" landing base...')
            base_x: float = blackboard.get('precise_fixed_x')
            base_y: float = blackboard.get('precise_fixed_y')
        else:
            yasmin.YASMIN_LOG_INFO(f'Fly to a "mobile" landing base...')
            base_x: float = blackboard.get('precise_mobile_x')
            base_y: float = blackboard.get('precise_mobile_y')

        points = [
            (None, None, safe_alt),
            (None, base_y, safe_alt),
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
