from datetime import datetime
import numpy as np
import os
import cv2
import pathlib

from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavrosDrone, MoveReference
from nectar.vision import ImageHandler, Aruco


class GoToLandingBase(State):
    def __init__(self, fixed_base: bool = True):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.fixed_base = fixed_base

        self.node = YasminNode.get_instance()

        self.aruco = Aruco(5, 0.5)

        self.timeout: int | float = self.node.get_parameter('timeout').value
        self.timeout_per_state: int | float = self.node.get_parameter('timeout_per_state').value

        self.base_x: int | float = self.node.get_parameter(f'{"fixed" if fixed_base else "mobile"}_base_x').value
        self.base_y: int | float = self.node.get_parameter(f'{"fixed" if fixed_base else "mobile"}_base_y').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')
        safe_altitude: int = blackboard.get('safe_altitude')

        image_handler: ImageHandler = blackboard.get('image_handler')
        image_handler.image_processing_callback = self.callback_aruco

        self.start_time: Time = blackboard.get('start_time')
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
            z = safe_altitude,
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
            z = safe_altitude,
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
            z = safe_altitude,
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

    def callback_aruco(self, image: np.ndarray):
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(self.node.get_clock().now().nanoseconds / 1e9)

        indoor_path = pathlib.Path.home() / 'ros2_ws' / start.strftime('indoor-%Y-%m-%d_%H-%M-%S')
        raw_path = indoor_path / 'aruco'
        annotated_path = indoor_path / 'aruco_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(indoor_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        cv2.imwrite(raw_file, image)

        bbox, id = self.aruco.detect(image, draw=True)

        cv2.imwrite(annotated_file, image)

        return image, bbox, id
