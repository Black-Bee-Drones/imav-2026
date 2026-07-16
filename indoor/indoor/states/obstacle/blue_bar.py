from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone, MoveReference


class BlueBar(State):
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

        altitude1 = [.4, .8, 1.2][self.config.bleu_step_-1] / 2 if self.config.bleu_step_1 else self.config.safe_altitude
        altitude2 = [.4, .8, 1.2][self.config.bleu_step_-1] / 2 if self.config.bleu_step_1 else self.config.safe_altitude

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

        return now - self.start_time > Duration(seconds=self.config.timeout) or \
            now - self.start_state > Duration(seconds=self.config.timeout_per_state)
