from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from indoor import Config


class BlueBar(State):
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

<<<<<<< HEAD
        altitude_1 = self.config.blue_alt[self.config.blue_step_1 -
                                          1] if self.config.blue_step_1 else self.config.safe_altitude
        altitude_2 = self.config.blue_alt[self.config.blue_step_2 -
                                          1] if self.config.blue_step_2 else self.config.safe_altitude
=======
        altitude1 = ([0.4, 0.8, 1.2][self.config.blue_step1 - 1] / 2) if self.config.blue_step1 else self.config.safe_altitude
        altitude2 = ([0.4, 0.8, 1.2][self.config.blue_step2 - 1] / 2) if self.config.blue_step2 else self.config.safe_altitude
>>>>>>> ea2beb4 (fix: correct n logic avigation on obstacle sm)

        yasmin.YASMIN_LOG_INFO(
            f'Step 1 - Correcting drone altitude by z={altitude_1:.1f} m...')
        drone.move_to(
<<<<<<< HEAD
            x=self.config.start_obstacle_x + 2.25,
            y=self.config.start_obstacle_y,
            z=altitude_1,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.precision,
            method=self.config.navigation_method,
=======
            x=2.25,
            y=None,
            z=altitude1,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.move_precision,
>>>>>>> ea2beb4 (fix: correct n logic avigation on obstacle sm)
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Step 1 - Fly under the first blue bar...')
        drone.move_to(
<<<<<<< HEAD
            x=self.config.start_obstacle_x + 3.25,
            y=self.config.start_obstacle_y,
            z=altitude_1,
=======
            x=3.25,
            y=None,
            z=altitude1,
>>>>>>> ea2beb4 (fix: correct n logic avigation on obstacle sm)
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.precision,
            method=self.config.navigation_method,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(
            f'Step 2 - Correcting drone altitude by z={altitude_2:.1f} m...')
        drone.move_to(
<<<<<<< HEAD
            x=self.config.start_obstacle_x + 3.25,
            y=self.config.start_obstacle_y,
            z=altitude_2,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.precision,
            method=self.config.navigation_method,
=======
            x=3.25,
            y=None,
            z=altitude2,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=self.config.move_precision,
>>>>>>> ea2beb4 (fix: correct n logic avigation on obstacle sm)
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Step 2 - Fly under the second blue bar...')
        drone.move_to(
<<<<<<< HEAD
            x=self.config.start_obstacle_x + 4.25,
            y=self.config.start_obstacle_y,
            z=altitude_2,
=======
            x=4.25,
            y=None,
            z=altitude2,
>>>>>>> ea2beb4 (fix: correct n logic avigation on obstacle sm)
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
