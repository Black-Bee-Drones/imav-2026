from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone, MoveReference


class GoToWindown(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()

        self.timeout: int | float = self.node.get_parameter('timeout').value
        self.timeout_per_state: int | float = self.node.get_parameter('timeout_per_state').value

        self.safe_altitude: int | float = self.node.get_parameter('safe_altitude').value
        self.room_x: int | float = self.node.get_parameter('room_x').value
        self.room_y: int | float = self.node.get_parameter('room_y').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Flying to a safe altitude...')
        drone.move_to(
            x = None,
            y = None,
            z = self.safe_altitude,
            yaw = 0,
            reference = MoveReference.TAKEOFF,  
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f'fly to y={self.room_y}...')
        drone.move_to(
            x = None,
            y = self.room_y,
            z = self.safe_altitude,
            yaw = 0,
            reference = MoveReference.TAKEOFF,  
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f'fly to x={self.room_x}...')
        drone.move_to(
            x = self.room_x,
            y = self.room_y,
            z = self.safe_altitude,
            yaw = 0,
            reference = MoveReference.TAKEOFF,  
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f'fly up to window height...')
        drone.move_to(
            x = self.room_x,
            y = self.room_y,
            z = 1.2,
            yaw = 0,
            reference = MoveReference.TAKEOFF,  
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_state > Duration(seconds=self.timeout_per_state)
