from datetime import datetime
import traceback
import pathlib
import os
import numpy as np
import cv2 as cv

from ament_index_python import get_package_share_directory

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import DroneFactory, MavrosConfig, MavlinkConfig, PoseSource
from nectar.vision import ImageHandler, Aruco, ROSConfig, OpenCVConfig, RealSenseConfig
from nectar.ai import Detector, DetectionResult

from ...config import Config


class Initialize(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.node = YasminNode.get_instance()
        self.start_time = self.node.get_clock().now()

        self.config = config

    def execute(self, blackboard: Blackboard):
        models_path = pathlib.Path(
            get_package_share_directory('indoor')) / 'models'

        yasmin.YASMIN_LOG_INFO('Initializing...')

        # Drone
        try:
            blackboard.set('config', self.config)
            config: Config = self.config
            drone_type = config.drone_type

            yasmin.YASMIN_LOG_INFO(f'Initializing Drone("{drone_type}")...')

            match drone_type:
                case 'mavros':
                    drone_config = MavrosConfig(
                        pose_source=PoseSource.VISION,
                        connection_string=config.drone_connection_string,
                    )
                case 'mavlink':
                    drone_config = MavlinkConfig(
                        pose_source=PoseSource.VISION,
                        connection_string=config.drone_connection_string,
                    )
                case _:
                    yasmin.YASMIN_LOG_ERROR('Invalid "drone_type"')
                    return ABORT

            drone = DroneFactory.create(drone_type, drone_config)
            blackboard.set('drone', drone)
            yasmin.YASMIN_LOG_INFO(f'Successful start Drone("{drone_type}")!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'DroneFactory failed: {e}')
            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT

        # Detector - gate
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector(gate)...')
            self.detector_gate = Detector(
                model_source=str(
                    models_path / config.model_gate_source),
                confidence_threshold=config.model_gate_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(gate)...')
            self.detector_gate.load()

            blackboard.set('detector_gate', self.detector_gate)
            blackboard.set('callback_gate', self.callback_detector_gate)
            yasmin.YASMIN_LOG_INFO('successful start Detector(gate)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Detector(gate) failed: {e}')
            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT

        # Detector - baby
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector(baby)...')
            self.detector_baby = Detector(
                model_source=str(
                    models_path / config.model_baby_source),
                confidence_threshold=config.model_baby_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(baby)...')
            self.detector_baby.load()

            blackboard.set('detector_baby', self.detector_baby)
            blackboard.set('callback_baby', self.callback_detector_baby)
            yasmin.YASMIN_LOG_INFO('successful start Detector(baby)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Detector(baby) failed: {e}')
            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT

        # Detector - box
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector(box)...')
            self.detector_box = Detector(
                model_source=str(
                    models_path / config.model_box_source),
                confidence_threshold=config.model_box_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(box)...')
            self.detector_box.load()

            blackboard.set('detector_box', self.detector_box)
            blackboard.set('callback_box', self.callback_detector_box)
            yasmin.YASMIN_LOG_INFO('successful start Detector(box)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Detector(box) failed: {e}')
            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT

        # Aruco
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Aruco...')
            self.aruco = Aruco(
                marker_dict=config.aruco_marker_dict,
                tag_size=config.aruco_size,
            )

            blackboard.set('aruco', self.aruco)
            blackboard.set('callback_aruco', self.callback_aruco)
            yasmin.YASMIN_LOG_INFO('successful start Aruco!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Aruco failed: {e}')
            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT

        # ImageHandler - north
        try:
            camera_north_source = config.camera_north_source
            yasmin.YASMIN_LOG_INFO('Initializing ImageHandler(north)...')

            match camera_north_source:
                case 'ros':
                    handler_config_north = ROSConfig(
                        topic=config.camera_north_topic,
                        compressed=config.camera_north_is_compressed,
                    )
                case 'realsense':
                    handler_config_north = RealSenseConfig()
                case 'opencv':
                    handler_config_north = OpenCVConfig(
                        device_index=config.camera_north_id,
                    )
                case _:
                    yasmin.YASMIN_LOG_ERROR('Invalid "camera_north_source"')
                    return ABORT

            image_handler_north = ImageHandler(
                image_source=camera_north_source,
                config=handler_config_north
            )

            yasmin.YASMIN_LOG_INFO('Open camera (north)...')
            image_handler_north.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo (north)...')
            result = image_handler_north.take_photo()
            if result is None:
                yasmin.YASMIN_LOG_WARN(result)

            blackboard.set('image_handler_north', image_handler_north)
            yasmin.YASMIN_LOG_INFO('successful start ImageHandler(north)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'ImageHandler(north) failed: {e}')
            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT

        # ImageHandler - down
        try:
            camera_down_source = config.camera_down_source
            yasmin.YASMIN_LOG_INFO('Initializing ImageHandler(down)...')

            match camera_down_source:
                case 'ros':
                    handler_config_down = ROSConfig(
                        topic=config.camera_down_topic,
                        compressed=config.camera_down_is_compressed,
                    )
                case 'opencv':
                    handler_config_down = OpenCVConfig(
                        device_index=config.camera_down_id,
                    )

            image_handler_down = ImageHandler(
                image_source=camera_down_source,
                config=handler_config_down
            )

            yasmin.YASMIN_LOG_INFO('Open camera (down)...')
            image_handler_down.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo (down)...')
            result = image_handler_down.take_photo()
            if result is None:
                yasmin.YASMIN_LOG_WARN(result)

            blackboard.set('image_handler_down', image_handler_down)
            yasmin.YASMIN_LOG_INFO('successful start ImageHandler(down)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'ImageHandler(down) failed: {e}')
            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')

        try:
            input("\n[PAUSE] Press ENTER to proceed to TAKEOFF...\n")
        except (KeyboardInterrupt, EOFError):
            yasmin.YASMIN_LOG_WARN('Takeoff cancelled by user.')
            return ABORT

        return SUCCEED

    def callback_detector_gate(self, image: np.ndarray) -> DetectionResult:
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(
            self.node.get_clock().now().nanoseconds / 1e9)

        indoor_path = pathlib.Path.home() / 'ros2_ws' / \
            start.strftime('indoor-%Y-%m-%d_%H-%M-%S')
        raw_path = indoor_path / 'gate'
        annotated_path = indoor_path / 'gate_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / \
            now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(indoor_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        result = self.detector_gate.detect(image)
        result.image = image
        result.annotated_image = self.detector_gate.draw_detections(
            image, result)

        cv.imwrite(raw_file, result.image)
        cv.imwrite(annotated_file, result.annotated_image)

        return result

    def callback_detector_box(self, image: np.ndarray) -> DetectionResult:
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(
            self.node.get_clock().now().nanoseconds / 1e9)

        indoor_path = pathlib.Path.home() / 'ros2_ws' / \
            start.strftime('indoor-%Y-%m-%d_%H-%M-%S')
        raw_path = indoor_path / 'box'
        annotated_path = indoor_path / 'box_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / \
            now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(indoor_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        result = self.detector_box.detect(image)
        result.image = image
        result.annotated_image = self.detector_box.draw_detections(
            image, result)

        cv.imwrite(raw_file, result.image)
        cv.imwrite(annotated_file, result.annotated_image)

        return result

    def callback_detector_baby(self, image: np.ndarray) -> DetectionResult:
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(
            self.node.get_clock().now().nanoseconds / 1e9)

        indoor_path = pathlib.Path.home() / 'ros2_ws' / \
            start.strftime('indoor-%Y-%m-%d_%H-%M-%S')
        raw_path = indoor_path / 'baby'
        annotated_path = indoor_path / 'baby_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / \
            now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(indoor_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        result = self.detector_baby.detect(image)
        result.image = image
        result.annotated_image = self.detector_baby.draw_detections(
            image, result)

        cv.imwrite(raw_file, result.image)
        cv.imwrite(annotated_file, result.annotated_image)

        return result

    def callback_aruco(self, image: np.ndarray):
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(
            self.node.get_clock().now().nanoseconds / 1e9)

        indoor_path = pathlib.Path.home() / 'ros2_ws' / \
            start.strftime('indoor-%Y-%m-%d_%H-%M-%S')
        raw_path = indoor_path / 'aruco'
        annotated_path = indoor_path / 'aruco_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / \
            now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(indoor_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        cv.imwrite(raw_file, image)

        marker_id, translation, yaw = self.aruco.pose_estimate(
            image, draw=True)

        cv.imwrite(annotated_file, image)

        return image, marker_id, translation, yaw
