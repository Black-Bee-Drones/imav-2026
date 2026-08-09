from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from indoor import Config


class Tubes(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.config = config

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        if self.config.tubes_skip:
            yasmin.YASMIN_LOG_INFO('Up to skip of the tubes...')
            drone.move_to(
                x=self.config.start_obstacle_x + 4.75,
                y=self.config.start_obstacle_y,
                z=self.config.safe_altitude,
                yaw=0,
                reference=MoveReference.TAKEOFF,
                precision=self.config.precision,
                method=self.config.navigation_method,
            )
            yasmin.YASMIN_LOG_INFO('Fly to skip of the tubes...')
            drone.move_to(
                x=self.config.start_obstacle_x + 6.25,
                y=self.config.start_obstacle_y,
                z=self.config.safe_altitude,
                yaw=0,
                reference=MoveReference.TAKEOFF,
                precision=self.config.precision,
                method=self.config.navigation_method,
            )
            yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
            return SUCCEED

        yasmin.YASMIN_LOG_INFO('Correcting drone altitude and move closer...')
        drone.move_to(
            x=self.config.start_obstacle_x + 4.75,
            y=self.config.start_obstacle_y,
            z=self.config.tubes_alt,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.precision,
            method=self.config.navigation_method,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly through the tube...')
        drone.move_to(
            x=self.config.start_obstacle_x + 5.75,
            y=self.config.start_obstacle_y,
            z=self.config.tubes_alt,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.precision,
            method=self.config.navigation_method,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly to the side of the tube...')
        drone.move_to(
            x=self.config.start_obstacle_x + 5.75,
            y=self.config.start_obstacle_y + self.config.tubes_offset,
            z=self.config.tubes_alt,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.precision,
            method=self.config.navigation_method,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly to the past the tube...')
        drone.move_to(
            x=self.config.start_obstacle_x + 6.25,
            y=self.config.start_obstacle_y + self.config.tubes_offset,
            z=0,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.precision,
            method=self.config.navigation_method,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly to the front of the tube...')
        drone.move_to(
            x=self.config.start_obstacle_x + 6.25,
            y=self.config.start_obstacle_y,
            z=0,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.precision,
            method=self.config.navigation_method,
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
