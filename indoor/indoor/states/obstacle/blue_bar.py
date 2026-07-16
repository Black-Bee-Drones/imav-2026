from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone, MoveReference


class BlueBar(State):
    def __init__(self, step1: int | None = 3, step2: int | None = 3):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.step1 = step1
        self.step2 = step2

        self.node = YasminNode.get_instance()

        self.timeout: int | float = self.node.get_parameter('timeout').value
        self.timeout_per_state: int | float = self.node.get_parameter('timeout_per_state').value

        self.safe_altitude: int | float = self.node.get_parameter('safe_altitude').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        altitude1 = [.4, .8, 1.2][self.step-1] / 2 if self.step1 else self.safe_altitude
        altitude2 = [.4, .8, 1.2][self.step-1] / 2 if self.step1 else self.safe_altitude

        yasmin.YASMIN_LOG_INFO(f'Step 1 - Correcting drone altitude by z={altitude1:.1f} m...')
        drone.move_to(
            x = 2.25,
            y = 0,
            z = altitude1,
            yaw = 0,
            reference = MoveReference.TAKEOFF,
            precision = 0.05,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Step 1 - Fly under the first blue bar...')
        drone.move_to(
            x = 3.25,
            y = 0,
            z = altitude1,
            yaw = 0,
            reference = MoveReference.TAKEOFF,
            precision = 0.5
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f'Step 2 - Correcting drone altitude by z={altitude2:.1f} m...')
        drone.move_to(
            x = 3.25,
            y = 0,
            z = altitude2,
            yaw = 0,
            reference = MoveReference.TAKEOFF,
            precision = 0.05,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Step 2 - Fly under the second blue bar...')
        drone.move_to(
            x = 4.25,
            y = 0,
            z = altitude2,
            yaw = 0,
            reference = MoveReference.TAKEOFF,
            precision = 0.5
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
