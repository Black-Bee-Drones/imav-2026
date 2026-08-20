from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from indoor import Config


class RedBar(State):
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

        altitude = config.red_alt[config.red_step - 1] if config.red_step else config.safe_altitude

        yasmin.YASMIN_LOG_INFO(
            f'Correcting drone altitude by z={altitude:.1f} m...')
        drone.move_to(
            x=config.start_obstacle_x + 1.25,
            y=config.start_obstacle_y,
            z=altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=config.precision,
        )

        self.node.get_clock().sleep_for(Duration(seconds=1))
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly over the red bar...')
        drone.move_to(
            x=config.start_obstacle_x + 2.25,
            y=config.start_obstacle_y,
            z=altitude,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=config.precision,
        )

        self.node.get_clock().sleep_for(Duration(seconds=1))
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
