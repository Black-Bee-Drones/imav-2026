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


class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.node = YasminNode.get_instance()

    def configure(self):

        self.add_output_key('drone')

        self.add_output_key('callback_gate')
        self.add_output_key('callback_baby')
        self.add_output_key('callback_box')

        self.add_output_key('image_handler_front')
        self.add_output_key('image_handler_down')

        self.add_input_key('drone_type')
        self.add_input_key('drone_connection_string')

        self.add_input_key('model_gate_source')
        self.add_input_key('model_gate_conf')

        self.add_input_key('model_baby_source')
        self.add_input_key('model_baby_conf')

        self.add_input_key('model_box_source')
        self.add_input_key('model_box_conf')

        self.add_input_key('aruco_marker_dict')
        self.add_input_key('aruco_size')

        self.add_input_key('camera_front_source')
        self.add_input_key('camera_front_topic')
        self.add_input_key('camera_front_id')

        self.add_input_key('camera_down_source')
        self.add_input_key('camera_down_topic')
        self.add_input_key('camera_down_id')

    def execute(self, blackboard: Blackboard):
        models_path = pathlib.Path(
            get_package_share_directory('indoor')) / 'models'

        yasmin.YASMIN_LOG_INFO('Initializing...')

        # Drone
        try:
            drone_type = blackboard.get('drone_type')
            yasmin.YASMIN_LOG_INFO(f'Initializing Drone("{drone_type}")...')

            match drone_type:
                case 'mavros':
                    drone_config = MavrosConfig(
                        pose_source=PoseSource.VISION,
                        connection_string=blackboard.get(
                            'drone_connection_string'),
                    )
                case 'mavlink':
                    drone_config = MavlinkConfig(
                        pose_source=PoseSource.VISION,
                        connection_string=blackboard.get(
                            'drone_connection_string'),
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
                    models_path / blackboard.get('model_gate_source')),
                confidence_threshold=blackboard.get('model_gate_conf'),
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
                    models_path / blackboard.get('model_baby_source')),
                confidence_threshold=blackboard.get('model_baby_conf'),
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
                    models_path / blackboard.get('model_box_source')),
                confidence_threshold=blackboard.get('model_box_conf'),
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
                marker_dict=blackboard.get('aruco_marker_dict'),
                tag_size=blackboard.get('aruco_size'),
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

        # ImageHandler - front
        try:
            camera_front_source = blackboard.get('camera_front_source')
            yasmin.YASMIN_LOG_INFO('Initializing ImageHandler(front)...')

            match camera_front_source:
                case 'ros':
                    handler_config_front = ROSConfig(
                        topic=blackboard.get('camera_front_topic'),
                        compressed=True,
                    )
                case 'realsense':
                    handler_config_front = RealSenseConfig()
                case 'opencv':
                    handler_config_front = OpenCVConfig(
                        device_index=blackboard.get('camera_front_id'),
                    )
                case _:
                    yasmin.YASMIN_LOG_ERROR('Invalid "camera_front_source"')
                    return ABORT

            image_handler_front = ImageHandler(
                image_source=camera_front_source,
                config=handler_config_front
            )

            yasmin.YASMIN_LOG_INFO('Open camera (front)...')
            image_handler_front.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo (front)...')
            result = image_handler_front.take_photo()
            if result is None:
                yasmin.YASMIN_LOG_WARN(result)

            blackboard.set('image_handler_front', image_handler_front)
            yasmin.YASMIN_LOG_INFO('successful start ImageHandler(front)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'ImageHandler(front) failed: {e}')
            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT

        # ImageHandler - down
        try:
            camera_down_source = blackboard.get('camera_down_source')
            yasmin.YASMIN_LOG_INFO('Initializing ImageHandler(down)...')

            match camera_down_source:
                case 'ros':
                    handler_config_down = ROSConfig(
                        topic=blackboard.get('camera_down_topic'),
                        compressed=True,
                    )
                case 'opencv':
                    handler_config_down = OpenCVConfig(
                        device_index=blackboard.get('camera_down_id'),
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
