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

from nectar.control import DroneFactory, MavrosConfig, MavlinkConfig,PoseSource, PIDController
from nectar.vision import ImageHandler, Aruco, ROSConfig
from nectar.ai import Detector, DetectionResult


class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        """
        Initializes the drone, detector and camera.
        """

        self.node = YasminNode.get_instance()

        models_path = pathlib.Path(get_package_share_directory('indoor')) / 'models'

        # Global
        self.node.declare_parameter('timeout', 1800)  # seconds
        self.node.declare_parameter('timeout_per_state', 300)  # seconds
        self.node.declare_parameter('safe_altitude', 3.0)  # meters
        self.node.declare_parameter('max_altitude', 7.0)  # meters

        self.timeout = self.node.get_parameter('timeout').value
        self.timeout_per_state = self.node.get_parameter('timeout_per_state').value
        self.safe_altitude = self.node.get_parameter('safe_altitude').value
        self.max_altitude = self.node.get_parameter('max_altitude').value

        # Initialize - Drone
        self.node.declare_parameter('drone_type', 'mavros')
        self.node.declare_parameter('connection_string', 'serial:///dev/ttyUSB0:921600')

        self.drone_type: str = self.node.get_parameter('drone_type').value
        self.connection_string = self.node.get_parameter('connection_string').value

        # Initialize - Detector
        self.node.declare_parameter('gate_model_source', 'gate.pt')
        self.node.declare_parameter('gate_conf', 0.5)
        self.node.declare_parameter('baby_model_source', 'best.pt')
        self.node.declare_parameter('baby_conf', 0.5)
        self.node.declare_parameter('box_model_source', 'package.pt')
        self.node.declare_parameter('box_conf', 0.5)
        self.node.declare_parameter('marker_dict', 5)  # 5x5

        self.gate_model_source: str = str(models_path / self.node.get_parameter('gate_model_source').value)
        self.gate_conf: float = self.node.get_parameter('gate_conf').value
        self.baby_model_source: str = str(models_path / self.node.get_parameter('baby_model_source').value)
        self.baby_conf: float = self.node.get_parameter('baby_conf').value
        self.box_model_source: str = str(models_path / self.node.get_parameter('box_model_source').value)
        self.box_conf: float = self.node.get_parameter('box_conf').value
        self.marker_dict: int = self.node.get_parameter('marker_dict').value

        # Initialize - ImageHandler
        self.node.declare_parameter('front_image_source', 'ros')
        self.node.declare_parameter('front_ros_topic', '/camera/color/image_raw')
        self.node.declare_parameter('down_image_source', 'webcam')
        self.node.declare_parameter('down_ros_topic', '/donw_camera/image')

        self.front_image_source: str = self.node.get_parameter('front_image_source').value
        self.front_ros_topic: str = self.node.get_parameter('front_ros_topic').value
        self.down_image_source: str = self.node.get_parameter('down_image_source').value
        self.down_ros_topic: str = self.node.get_parameter('down_ros_topic').value

        # Takeoff
        self.node.declare_parameter('takeoff_altitude', 1.2)  # meters

        # Obstacle / Window
        self.node.declare_parameter('window_threshold', 50)  # pixels

        # Inspect / GoToWindown
        self.node.declare_parameter('room_x', 10.0)  # meters
        self.node.declare_parameter('room_y', 2.0)  # meters

        # Inspect / FindWindow
        self.node.declare_parameter('find_tolerance', 2)
        self.node.declare_parameter('back_speed', -0.5)  # meters per second

        # Precise landing / GoToLandingBase
        self.node.declare_parameter('fixed_base_x', 0.0)  # meters
        self.node.declare_parameter('fixed_base_y', 2.0)  # meters
        self.node.declare_parameter('mobile_base_x', 0.0)  # meters
        self.node.declare_parameter('mobile_base_y', -2.0)  # meters

        # Precise landing / Center
        self.node.declare_parameter('center_threshold', 50)  # pixels
        self.node.declare_parameter('lost_tolerance', 10)
        self.node.declare_parameter('land_altitude', 1.0)  # meters
        self.node.declare_parameter('land_speed', -0.5)  # meters per second

        # Precise landing / Reacquire
        self.node.declare_parameter('reacquire_step', 0.5)  # meters

        # PID xy
        self.node.declare_parameter('controller_xy_kp', 1.0)
        self.node.declare_parameter('controller_xy_kd', 1.0)
        self.node.declare_parameter('controller_xy_ki', 1.0)
        self.node.declare_parameter('controller_xy_output_min', -1.0)
        self.node.declare_parameter('controller_xy_output_max', 1.0)
        self.node.declare_parameter('controller_xy_integral_min', -1.0)
        self.node.declare_parameter('controller_xy_integral_max', 1.0)

        self.controller_xy_kp = self.node.get_parameter('controller_xy_kp').value
        self.controller_xy_kd = self.node.get_parameter('controller_xy_kd').value
        self.controller_xy_ki = self.node.get_parameter('controller_xy_ki').value
        self.controller_xy_output_min = self.node.get_parameter('controller_xy_output_min').value
        self.controller_xy_output_max = self.node.get_parameter('controller_xy_output_max').value
        self.controller_xy_integral_min = self.node.get_parameter('controller_xy_integral_min').value
        self.controller_xy_integral_max = self.node.get_parameter('controller_xy_integral_max').value

        # PID z
        self.node.declare_parameter('controller_z_kp', 1.0)
        self.node.declare_parameter('controller_z_kd', 1.0)
        self.node.declare_parameter('controller_z_ki', 1.0)
        self.node.declare_parameter('controller_z_output_min', -1.0)
        self.node.declare_parameter('controller_z_output_max', 1.0)
        self.node.declare_parameter('controller_z_integral_min', -1.0)
        self.node.declare_parameter('controller_z_integral_max', 1.0)

        self.controller_z_kp = self.node.get_parameter('controller_z_kp').value
        self.controller_z_kd = self.node.get_parameter('controller_z_kd').value
        self.controller_z_ki = self.node.get_parameter('controller_z_ki').value
        self.controller_z_output_min = self.node.get_parameter('controller_z_output_min').value
        self.controller_z_output_max = self.node.get_parameter('controller_z_output_max').value
        self.controller_z_integral_min = self.node.get_parameter('controller_z_integral_min').value
        self.controller_z_integral_max = self.node.get_parameter('controller_z_integral_max').value

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO('Initializing...')

        # Const
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Start time...')
            self.start_time = self.node.get_clock().now()

            blackboard.set('start_time', self.start_time)
            yasmin.YASMIN_LOG_INFO('successful Start Const!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Start time failed: {e}')
            return ABORT

        # Drone
        try:
            yasmin.YASMIN_LOG_INFO(f'Initializing Drone("{self.drone_type}")...')
            if self.drone_type == 'mavros':
                drone_config = MavrosConfig(
                    pose_source = PoseSource.VISION,
                    connection_string=self.connection_string
                )

            elif self.drone_type == 'mavlink':
                drone_config = MavlinkConfig(
                    pose_source=PoseSource.VISION,
                    start_driver=False,
                    connection_string=self.connection_string
                )

            else:
                yasmin.YASMIN_LOG_ERROR('Invalid drone_type.')
                return ABORT

            drone = DroneFactory.create(self.drone_type, drone_config)

            blackboard['drone'] = drone
            yasmin.YASMIN_LOG_INFO(f'Successful start Drone("{self.drone_type}")!')

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
                kp = self.controller_xy_kp,
                kd = self.controller_xy_kd,
                ki = self.controller_xy_ki,
                output_limits = (self.controller_xy_output_min, self.controller_xy_output_max),
                integral_limits = (self.controller_xy_integral_min, self.controller_xy_integral_max),
            )
            pid_y = PIDController(
                kp = self.controller_xy_kp,
                kd = self.controller_xy_kd,
                ki = self.controller_xy_ki,
                output_limits = (self.controller_xy_output_min, self.controller_xy_output_max),
                integral_limits = (self.controller_xy_integral_min, self.controller_xy_integral_max),
            )
            pid_z = PIDController(
                kp = self.controller_z_kp,
                kd = self.controller_z_kd,
                ki = self.controller_z_ki,
                output_limits = (self.controller_z_output_min, self.controller_z_output_max),
                integral_limits = (self.controller_z_integral_min, self.controller_z_integral_max),
            )

            blackboard['pid_x'] = pid_x
            blackboard['pid_y'] = pid_y
            blackboard['pid_z'] = pid_z
            yasmin.YASMIN_LOG_INFO(f'Successful start PID (x, y and z)!')

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
                model_source = self.gate_model_source,
                confidence_threshold = self.gate_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(gate)...')
            self.detector_gate.load()

            blackboard['detector_gate'] = self.detector_gate
            blackboard['callback_detector_gate'] = self.callback_detector_gate
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
                model_source = self.baby_model_source,
                confidence_threshold = self.baby_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(baby)...')
            self.detector_baby.load()

            blackboard['detector_baby'] = self.detector_baby
            blackboard['callback_detector_baby'] = self.callback_detector_baby
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
                model_source = self.box_model_source,
                confidence_threshold = self.box_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(box)...')
            self.detector_box.load()

            blackboard['detector_box'] = self.detector_box
            blackboard['callback_detector_box'] = self.callback_detector_box
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
                marker_dict = self.marker_dict,
                tag_size = 1.0,
            )

            blackboard['aruco'] = self.aruco
            blackboard['callback_aruco'] = self.callback_aruco
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
                image_source = self.front_image_source,
                config = ROSConfig(topic=self.front_ros_topic) if self.front_image_source == 'ros' else None,
            )

            yasmin.YASMIN_LOG_INFO('Open camera (front)...')
            image_handler_front.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo (front)...')
            image_handler_front.take_photo()

            blackboard['image_handler_front'] = image_handler_front
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
                image_source = self.down_image_source,
                config = ROSConfig(topic=self.down_ros_topic) if self.down_image_source == 'ros' else None,
            )

            yasmin.YASMIN_LOG_INFO('Open camera (down)...')
            image_handler_down.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo (down)...')
            image_handler_down.take_photo()

            blackboard['image_handler_down'] = image_handler_down
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
        now = datetime.fromtimestamp(self.node.get_clock().now().nanoseconds / 1e9)

        indoor_path = pathlib.Path.home() / 'ros2_ws' / start.strftime('indoor-%Y-%m-%d_%H-%M-%S')
        raw_path = indoor_path / 'gate'
        annotated_path = indoor_path / 'gate_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(indoor_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        result = self.detector_gate.detect(image)
        result.image = image
        result.annotated_image = self.detector_gate.draw_detections(image, result)

        cv2.imwrite(raw_file, result.image)
        cv2.imwrite(annotated_file, result.annotated_image)

        return result

    def callback_detector_box(self, image: np.ndarray) -> DetectionResult:
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(self.node.get_clock().now().nanoseconds / 1e9)

        indoor_path = pathlib.Path.home() / 'ros2_ws' / start.strftime('indoor-%Y-%m-%d_%H-%M-%S')
        raw_path = indoor_path / 'box'
        annotated_path = indoor_path / 'box_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(indoor_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        result = self.detector_box.detect(image)
        result.image = image
        result.annotated_image = self.detector_box.draw_detections(image, result)

        cv2.imwrite(raw_file, result.image)
        cv2.imwrite(annotated_file, result.annotated_image)

        return result

    def callback_detector_baby(self, image: np.ndarray) -> DetectionResult:
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(self.node.get_clock().now().nanoseconds / 1e9)

        indoor_path = pathlib.Path.home() / 'ros2_ws' / start.strftime('indoor-%Y-%m-%d_%H-%M-%S')
        raw_path = indoor_path / 'baby'
        annotated_path = indoor_path / 'baby_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(indoor_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        result = self.detector_baby.detect(image)
        result.image = image
        result.annotated_image = self.detector_baby.draw_detections(image, result)

        cv2.imwrite(raw_file, result.image)
        cv2.imwrite(annotated_file, result.annotated_image)

        return result

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
