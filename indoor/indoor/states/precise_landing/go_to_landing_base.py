from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from indoor import Config


class GoToLandingBase(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

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

        yasmin.YASMIN_LOG_INFO(
            f'Fly to a {"fixed" if config.fixed_base else "mobile"} landing base...')

        yasmin.YASMIN_LOG_INFO('Flying to a safe altitude...')
        drone.move_to(
            x=None,
            y=None,
            z=config.safe_altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(
            f'fly to y={config.base_y} from the {"fixed" if config.fixed_base else "mobile"} base...')
        drone.move_to(
            x=None,
            y=config.base_y,
            z=config.safe_altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(
            f'fly to x={config.base_x} from the {"fixed" if config.fixed_base else "mobile"} base...')
        drone.move_to(
            x=config.base_x,
            y=config.base_y,
            z=config.safe_altitude,
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

        return now - self.start_time > Duration(seconds=config.timeout) or \
            now - \
            self.start_state > Duration(seconds=config.timeout_per_state)
