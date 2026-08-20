from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from indoor import Config


class BlueBar(State):
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

        altitude_1 = config.blue_alt[config.blue_step_1 -
                                          1] if config.blue_step_1 else config.safe_altitude
        altitude_2 = config.blue_alt[config.blue_step_2 -
                                          1] if config.blue_step_2 else config.safe_altitude

        yasmin.YASMIN_LOG_INFO(
            f'Step 1 - Correcting drone altitude by z={altitude_1:.1f} m...')
        drone.move_to(
            x=config.start_obstacle_x + 2.25,
            y=config.start_obstacle_y,
            z=altitude_1,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=config.precision,
            method=config.navigation_method,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Step 1 - Fly under the first blue bar...')
        drone.move_to(
            x=config.start_obstacle_x + 3.25,
            y=config.start_obstacle_y,
            z=altitude_1,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=config.precision,
            method=config.navigation_method,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(
            f'Step 2 - Correcting drone altitude by z={altitude_2:.1f} m...')
        drone.move_to(
            x=config.start_obstacle_x + 3.25,
            y=config.start_obstacle_y,
            z=altitude_2,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=config.precision,
            method=config.navigation_method,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Step 2 - Fly under the second blue bar...')
        drone.move_to(
            x=config.start_obstacle_x + 4.25,
            y=config.start_obstacle_y,
            z=altitude_2,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=config.precision,
            method=config.navigation_method,
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
