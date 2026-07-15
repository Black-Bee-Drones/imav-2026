import numpy as np

from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT

from nectar.control import MavrosDrone, PIDController
from nectar.vision import ImageHandler


class Center(State):
    def __init__(self, color_window: str = 'blue'):
        super().__init__(outcomes=[SUCCEED, FAIL, TIMEOUT])

        self.color_window = color_window.strip().lower()

        self.node = YasminNode.get_instance()

        self.timeout: int | float = self.node.get_parameter('timeout').value
        self.timeout_per_state: int | float = self.node.get_parameter('timeout_per_state').value

        self.lost_tolerance: int | float = self.node.get_parameter('lost_tolerance').value
        self.land_altitude: int | float = self.node.get_parameter('land_altitude').value
        self.land_speed: int | float = self.node.get_parameter('land_altitude').value
        self.center_threshold: int | float = self.node.get_parameter('center_threshold').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        pid_x: PIDController = blackboard.get('pid_x')
        pid_y: PIDController = blackboard.get('pid_y')

        image_handler_down: ImageHandler = blackboard.get('image_handler_down')
        image_handler_down.image_processing_callback = blackboard.get('callback_aruco')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        pid_x.reset()
        pid_y.reset()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        lost = 0
        while True:
            now = self.node.get_clock().now()

            image, bbox, id = image_handler_down.take_photo()

            if isinstance(id, int):
                lost = 0

                h, w = image.shape[:2]

                center = np.mean(bbox[0][0], axis=0)

                error_x = (center[1] - (w / 2))
                error_y = (center[0] - (h / 2))

                if (error_x**2 + error_y**2) <= self.px_threshold**2 and drone.get_altitude() < self.land_altitude:
                    yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
                    drone.move_velocity()
                    return SUCCEED

                output_x = pid_x.update(error_x)
                output_y = pid_y.update(error_y)

                yasmin.YASMIN_LOG_INFO(f'Centering: error_x={error_x:.0f}; error_y={error_y:.0f}; output_x={output_x:.1f}; output_y={output_y:.1f}.')
                drone.move_velocity(
                    vx = output_x,
                    vy = output_y,
                    vz = self.land_speed if ((error_x**2 + error_y**2) <= 4*self.px_threshold**2) else 0,
                    vyaw = 0,
                )
            else:
                yasmin.YASMIN_LOG_ERROR(f'Lost detection ({lost}/{self.lost_tolerance}).')
                lost += 1

                if self.lost_tolerance <= lost:
                    yasmin.YASMIN_LOG_ERROR('Lost detection exceeded.')
                    drone.move_velocity()
                    return FAIL

            if self.check_timeout():
                yasmin.YASMIN_LOG_ERROR('Timeout.')
                drone.move_velocity()
                return TIMEOUT

            self.node.get_clock().sleep_until(now + Duration(seconds=1/30))

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_state > Duration(seconds=self.timeout_per_state)
