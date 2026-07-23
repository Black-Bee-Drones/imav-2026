from datetime import datetime
import numpy as np
import os
import cv2
import pathlib

from ament_index_python.packages import get_package_share_directory

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import DroneFactory, MavrosConfig, MavlinkConfig, PoseSource, PIDController
from nectar.vision import ImageHandler, Aruco, ROSConfig
from nectar.ai import Detector, DetectionResult

from indoor import Config


class Initialize(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        """
        Initializes the drone, detector and camera.
        """

        self.config = config

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO('Initializing...')

        # Start time
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Start time...')
            self.start_time = self.node.get_clock().now()

            blackboard.set('start_time', self.start_time)
            yasmin.YASMIN_LOG_INFO('successful Start time!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Start time failed: {e}')
            return ABORT

        # Drone
        try:
            yasmin.YASMIN_LOG_INFO(
                f'Initializing Drone("{self.config.drone_type}")...')
            if self.config.drone_type == 'mavros':
                drone_config = MavrosConfig(
                    pose_source=PoseSource.VISION,
                    connection_string=self.config.connection_string
                )

            elif self.config.drone_type == 'mavlink':
                drone_config = MavlinkConfig(
                    pose_source=PoseSource.VISION,
                    start_driver=False,
                    connection_string=self.config.connection_string
                )

            else:
                yasmin.YASMIN_LOG_ERROR('Invalid drone_type.')
                return ABORT

            drone = DroneFactory.create(self.config.drone_type, drone_config)

            blackboard.set('drone', drone)
            yasmin.YASMIN_LOG_INFO(
                f'Successful start Drone("{self.config.drone_type}")!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'DroneFactory failed: {e}')
            return ABORT

        # PID
        try:
            yasmin.YASMIN_LOG_INFO(f'Initializing PID (x, y and z)...')
            pid_x = PIDController(
                kp=self.config.controller_xy_kp,
                kd=self.config.controller_xy_kd,
                ki=self.config.controller_xy_ki,
                output_limits=(self.config.controller_xy_output_min,
                               self.config.controller_xy_output_max),
                integral_limits=(self.config.controller_xy_integral_min,
                                 self.config.controller_xy_integral_max),
            )
            pid_y = PIDController(
                kp=self.config.controller_xy_kp,
                kd=self.config.controller_xy_kd,
                ki=self.config.controller_xy_ki,
                output_limits=(self.config.controller_xy_output_min,
                               self.config.controller_xy_output_max),
                integral_limits=(self.config.controller_xy_integral_min,
                                 self.config.controller_xy_integral_max),
            )
            pid_z = PIDController(
                kp=self.config.controller_z_kp,
                kd=self.config.controller_z_kd,
                ki=self.config.controller_z_ki,
                output_limits=(self.config.controller_z_output_min,
                               self.config.controller_z_output_max),
                integral_limits=(self.config.controller_z_integral_min,
                                 self.config.controller_z_integral_max),
            )
            pid_yaw = PIDController(
                kp=self.config.controller_yaw_kp,
                kd=self.config.controller_yaw_kd,
                ki=self.config.controller_yaw_ki,
                output_limits=(self.config.controller_yaw_output_min,
                               self.config.controller_yaw_output_max),
                integral_limits=(self.config.controller_yaw_integral_min,
                                 self.config.controller_yaw_integral_max),
            )

            blackboard.set('pid_x', pid_x)
            blackboard.set('pid_y', pid_y)
            blackboard.set('pid_z', pid_z)
            blackboard.set('pid_yaw', pid_yaw)
            yasmin.YASMIN_LOG_INFO(f'Successful start PID (x, y, z and yaw)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'PID failed: {e}')
            return ABORT

        # Detector - gate
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector(gate)...')
            self.detector_gate = Detector(
                model_source=self.config.gate_model_source,
                confidence_threshold=self.config.gate_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(gate)...')
            self.detector_gate.load()

            blackboard.set('detector_gate', self.detector_gate)
            blackboard.set('callback_detector_gate',
                           self.callback_detector_gate)
            yasmin.YASMIN_LOG_INFO('successful start Detector(gate)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Detector(gate) failed: {e}')
            return ABORT

        # Detector - baby
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector(baby)...')
            self.detector_baby = Detector(
                model_source=self.config.baby_model_source,
                confidence_threshold=self.config.baby_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(baby)...')
            self.detector_baby.load()

            blackboard.set('detector_baby', self.detector_baby)
            blackboard.set('callback_detector_baby',
                           self.callback_detector_baby)
            yasmin.YASMIN_LOG_INFO('successful start Detector(baby)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Detector(baby) failed: {e}')
            return ABORT

        # Detector - box
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector(box)...')
            self.detector_box = Detector(
                model_source=self.config.box_model_source,
                confidence_threshold=self.config.box_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(box)...')
            self.detector_box.load()

            blackboard.set('detector_box', self.detector_box)
            blackboard.set('callback_detector_box', self.callback_detector_box)
            yasmin.YASMIN_LOG_INFO('successful start Detector(box)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Detector(box) failed: {e}')
            return ABORT

        # Aruco
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Aruco...')
            self.aruco = Aruco(
                marker_dict=self.config.marker_dict,
                tag_size=self.config.aruco_size,
            )

            blackboard.set('aruco', self.aruco)
            blackboard.set('callback_aruco', self.callback_aruco)
            yasmin.YASMIN_LOG_INFO('successful start Aruco!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Aruco failed: {e}')
            return ABORT

        # ImageHandler - front
        try:
            yasmin.YASMIN_LOG_INFO('Initializing ImageHandler(front)...')
            image_handler_front = ImageHandler(
                image_source=self.config.front_image_source,
                config=(ROSConfig(topic=self.config.front_ros_topic)
                        if self.config.front_image_source == 'ros'
                        else None),
            )

            yasmin.YASMIN_LOG_INFO('Open camera (front)...')
            image_handler_front.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo (front)...')
            image_handler_front.take_photo()

            blackboard.set('image_handler_front', image_handler_front)
            yasmin.YASMIN_LOG_INFO('successful start ImageHandler(front)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'ImageHandler(front) failed: {e}')
            return ABORT

        # ImageHandler - down
        try:
            yasmin.YASMIN_LOG_INFO('Initializing ImageHandler(down)...')
            image_handler_down = ImageHandler(
                image_source=self.config.down_image_source,
                config=(ROSConfig(topic=self.config.down_ros_topic)
                        if self.config.down_image_source == 'ros'
                        else None),
            )

            yasmin.YASMIN_LOG_INFO('Open camera (down)...')
            image_handler_down.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo (down)...')
            image_handler_down.take_photo()

            blackboard.set('image_handler_down', image_handler_down)
            yasmin.YASMIN_LOG_INFO('successful start ImageHandler(down)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'ImageHandler(down) failed: {e}')
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

        cv2.imwrite(raw_file, result.image)
        cv2.imwrite(annotated_file, result.annotated_image)

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

        cv2.imwrite(raw_file, result.image)
        cv2.imwrite(annotated_file, result.annotated_image)

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

        cv2.imwrite(raw_file, result.image)
        cv2.imwrite(annotated_file, result.annotated_image)

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

        cv2.imwrite(raw_file, image)

        marker_id, translation, yaw = self.aruco.pose_estimate(
            image, draw=True)

        cv2.imwrite(annotated_file, image)

        return image, marker_id, translation, yaw
