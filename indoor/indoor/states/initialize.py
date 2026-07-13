from datetime import datetime
import numpy as np
import os
import cv2
import pathlib

from rclpy.parameter import Parameter
from ament_index_python.packages import get_package_share_directory

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import DroneFactory, MavrosConfig, PoseSource
from nectar.vision import ImageHandler
from nectar.ai import Detector, DetectionResult


class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.set_description('Initializes the drone, detector and camera.')

        self.node = YasminNode.get_instance()

        models_path = pathlib.Path(get_package_share_directory('indoor')) / 'models'

        self.node.declare_parameter('drone_type', Parameter.Type.STRING)
        self.node.declare_parameter('model_source', Parameter.Type.STRING)
        self.node.declare_parameter('confidence_threshold', Parameter.Type.DOUBLE)
        self.node.declare_parameter('image_source', Parameter.Type.STRING)

        self.drone_type = self.node.get_parameter('drone_type').value
        self.model_source = models_path / self.node.get_parameter('model_source').value
        self.confidence_threshold = self.node.get_parameter('confidence_threshold').value
        self.image_source = self.node.get_parameter('image_source').value

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO('Initializing...')

        # Start Time
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Start Time...')
            self.start_time = self.node.get_clock().now()
            blackboard['start_time'] = self.start_time
            yasmin.YASMIN_LOG_INFO('Successfull Start Time...')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'DroneFactory failed: {e}')
            return ABORT

        # Drone
        try:
            yasmin.YASMIN_LOG_INFO(f'Initializing Drone("{self.drone_type}")...')
            drone_config = MavrosConfig(
                pose_source = PoseSource.VISION,
            )
            drone = DroneFactory.create('mavros', drone_config)

            blackboard['drone'] = drone
            yasmin.YASMIN_LOG_INFO(f'Successfull start Drone(\'{self.drone_type}\')...')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'DroneFactory failed: {e}')
            return ABORT

        # Detector
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector...')
            self.detector = Detector(
                model_source = self.model_source,
                confidence_threshold = self.confidence_threshold,
            )

            blackboard['detector'] = self.detector
            yasmin.YASMIN_LOG_INFO('Successfull start Detector...')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Detector failed: {e}')
            return ABORT

        # ImageHandler
        try:
            yasmin.YASMIN_LOG_INFO('Initializing ImageHandler...')
            image_handler = ImageHandler(
                image_source = self.image_source,
                image_processing_callback = self.callback_detector,
            )

            yasmin.YASMIN_LOG_INFO('Open camera...')
            image_handler.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo...')
            image_handler.take_photo()

            blackboard['image_handler'] = image_handler
            yasmin.YASMIN_LOG_INFO('Successfull start ImageHandler...')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'ImageHandler failed: {e}')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED

    def callback_detector(self, image: np.ndarray) -> DetectionResult:
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(self.node.get_clock().now().nanoseconds / 1e9)

        indoor_path = pathlib.Path.home() / 'ros2_ws' / start.strftime('indoor-%Y-%m-%d_%H-%M-%S')
        raw_path = indoor_path / 'raw'
        annotated_path = indoor_path / 'annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(indoor_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        result = self.detector.detect(image)
        result.image = image
        result.annotated_image = self.detector.draw_detections(image, result)

        cv2.imwrite(raw_file, result.image)
        cv2.imwrite(annotated_file, result.annotated_image)

        return result
