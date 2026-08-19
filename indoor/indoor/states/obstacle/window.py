from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, PIDController, MoveReference
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult

from indoor import Config
import cv2
import time


class Window(State):
    def __init__(self, config: Config, position: str):
        super().__init__(outcomes=[SUCCEED, TIMEOUT, 'reacquire'])

        self.config = config
        self.position = position

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')

        pid_y: PIDController = blackboard.get('pid_y')
        pid_z: PIDController = blackboard.get('pid_z')

        image_handler_front: ImageHandler = blackboard.get(
            'image_handler_front')
        image_handler_front.image_processing_callback = blackboard.get(
            'callback_detector_gate')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        pid_y.reset()
        pid_z.reset()

        if self.position == 'first':
            color_window = self.config.first_color_window
        elif self.position == 'second':
            color_window = self.config.second_color_window
        elif self.position == 'room':
            color_window = self.config.room_color_window

        yasmin.YASMIN_LOG_INFO('Start.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Correcting drone altitude...')
        drone.move_to(
            x=0,
            y=0,
            z=self.config.gate_alt - drone.get_altitude(),
            yaw=0,
            precision=self.config.precision,
            method=self.config.navigation_method,
        )

        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        lost = 0
        detected = 0
        while color_window:
            result: DetectionResult = image_handler_front.take_photo()

            window = result.filter_by_class(
                ['blue' if color_window == 'blue' else 'red'])

            h, w = result.image.shape[:2]

            pid_y.set_setpoint((w + self.config.y_offset) / 2)
            pid_z.set_setpoint((h + self.config.z_offset) / 2)

            if window:
                lost = 0
                detected += 1

                center = window[0].center

                output_y = pid_y.update(center[0])
                output_z = pid_z.update(center[1])

                error_y = pid_y.setpoint - center[0]
                error_z = pid_z.setpoint - center[1]

                yasmin.YASMIN_LOG_INFO(
                    f'Centering: error_y={error_y:.0f}; error_z={error_z:.0f}; output_y={output_y:.1f}; output_z={output_z:.1f}; detected={detected}.')
                drone.move_velocity(
                    vx=0.2,
                    vy=output_y,
                    vz=output_z,
                    vyaw=0,
                )

            elif detected >= self.config.gate_detection_tolerance:
                drone.move_to(
                    x=0.75,
                    y=0,
                    z=0,
                    yaw=0,
                    reference=MoveReference.BODY,
                )

            else:
                yasmin.YASMIN_LOG_ERROR(f'Lost detection {lost}.')
                lost += 1

            if lost > self.config.find_tolerance:
                yasmin.YASMIN_LOG_ERROR(
                    'Blue window not found. Reacquiring...')
                return 'reacquire'

            if self.check_timeout():
                yasmin.YASMIN_LOG_ERROR('Timeout.')
                return TIMEOUT

        else:
            yasmin.YASMIN_LOG_INFO('Fly the drone to safe altitude.')
            drone.move_to(
                x=0,
                y=0,
                z=self.config.safe_altitude - drone.get_altitude(),
                yaw=0,
                precision=self.config.precision,
                method=self.config.navigation_method,
            )

        self.node.get_clock().sleep_for(Duration(seconds=1))
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        self.node.get_clock().sleep_for(Duration(seconds=1))
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
