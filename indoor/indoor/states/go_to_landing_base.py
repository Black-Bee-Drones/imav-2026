from rclpy.time import Time, Duration
from rclpy.parameter import Parameter

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone, MoveReference


class GoToLandingBase(State):
    def __init__(self, fixed_base: bool = True):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.fixed_base = fixed_base

        self.node = YasminNode.get_instance()

        self.node.declare_parameter('safe_altitude', Parameter.Type.DOUBLE)

        self.safe_altitude = self.node.get_parameter('safe_altitude').value

        if fixed_base:
            self.node.declare_parameter('fixed_base_x', Parameter.Type.DOUBLE)
            self.node.declare_parameter('fixed_base_y', Parameter.Type.DOUBLE)

            self.base_x = self.node.get_parameter('fixed_base_x').value
            self.base_y = self.node.get_parameter('fixed_base_y').value
        else:
            self.node.declare_parameter('mobile_base_x', Parameter.Type.DOUBLE)
            self.node.declare_parameter('mobile_base_y', Parameter.Type.DOUBLE)

            self.base_x = self.node.get_parameter('mobile_base_x').value
            self.base_y = self.node.get_parameter('mobile_base_y').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        self.start_time: Time = blackboard['start_time']
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f'Fly to a {"fixed" if self.fixed_base else "mobile"} landing base...')

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

        yasmin.YASMIN_LOG_INFO(f'fly to y={self.base_y} from the {"fixed" if self.fixed_base else "mobile"} base...')
        drone.move_to(
            x = None,
            y = self.base_y,
            z = self.safe_altitude,
            yaw = 0,
            reference = MoveReference.TAKEOFF,  
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f'fly to x={self.base_x} from the {"fixed" if self.fixed_base else "mobile"} base...')
        drone.move_to(
            x = self.base_x,
            y = self.base_y,
            z = self.safe_altitude,
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

        return now - self.start_time > Duration(seconds=self.overall_max) or \
            now - self.start_state > Duration(seconds=self.overall_max_per_state)
