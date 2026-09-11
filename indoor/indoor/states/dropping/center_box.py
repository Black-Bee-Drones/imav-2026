from rclpy.time import Time, Duration

import numpy as np
import math

import yasmin
from yasmin import State, Blackboard, YASMIN_LOG_INFO
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT

from nectar.control import MavlinkDrone, MavrosDrone, PIDController
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult

from ...config import Config

Point2D = tuple[float, float]       # (x, y)
Pose2D = tuple[float, float, float] # (x, y, yaw)

class CenterBox(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, FAIL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone | MavrosDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        pid_x: PIDController = blackboard.get('pid_x')
        pid_y: PIDController = blackboard.get('pid_y')

        image_handler_down: ImageHandler = blackboard.get('image_handler_down')
        image_handler_down.image_processing_callback = blackboard.get('callback_box')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        pid_x.reset()
        pid_y.reset()

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout(config):
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        lost = 0
        while True:
            now = self.node.get_clock().now()

            result = image_handler_down.take_photo()

            if result:
                lost = 0

                h, w = result.image.shape[:2]

                center = result[0].center

                error_x = (center[1] - (w / 2))
                error_y = (center[0] - (h / 2))

                if (error_x**2 + error_y**2) <= config.center_threshold**2 and drone.get_altitude() < config.land_altitude:
                    yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
                    drone.move_velocity()
                    return SUCCEED

                output_x = pid_x.update(error_x)
                output_y = pid_y.update(error_y)

                yasmin.YASMIN_LOG_INFO(
                    f'Centering: error_x={error_x:.0f}; error_y={error_y:.0f}; output_x={output_x:.1f}; output_y={output_y:.1f}.')
                drone.move_velocity(
                    vx=output_x,
                    vy=output_y,
                    vz=config.land_speed if (
                        (error_x**2 + error_y**2) <= 4*config.center_threshold**2) else 0,
                    vyaw=0,
                )
            else:
                yasmin.YASMIN_LOG_ERROR(
                    f'Lost detection ({lost}/{config.lost_tolerance}).')
                lost += 1

                if config.lost_tolerance <= lost:
                    yasmin.YASMIN_LOG_ERROR('Lost detection exceeded.')
                    drone.move_velocity()
                    return FAIL

            if self.check_timeout(config):
                yasmin.YASMIN_LOG_ERROR('Timeout.')
                drone.move_velocity()
                return TIMEOUT

            self.node.get_clock().sleep_until(now + Duration(seconds=1/30))

    def check_timeout(self, config):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=config.timeout) or \
            now - \
            self.start_state > Duration(seconds=config.timeout_per_state)
