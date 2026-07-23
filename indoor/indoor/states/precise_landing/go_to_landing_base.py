from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone, MoveReference

from indoor import Config


class GoToLandingBase(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.config = config

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(
            f'Fly to a {"fixed" if self.config.fixed_base else "mobile"} landing base...')

        yasmin.YASMIN_LOG_INFO('Flying to a safe altitude...')
        drone.move_to(
            x=None,
            y=None,
            z=self.config.safe_altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(
            f'fly to y={self.config.base_y} from the {"fixed" if self.config.fixed_base else "mobile"} base...')
        drone.move_to(
            x=None,
            y=self.config.base_y,
            z=self.config.safe_altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(
            f'fly to x={self.config.base_x} from the {"fixed" if self.config.fixed_base else "mobile"} base...')
        drone.move_to(
            x=self.config.base_x,
            y=self.config.base_y,
            z=self.config.safe_altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.config.timeout) or \
            now - \
            self.start_state > Duration(seconds=self.config.timeout_per_state)
