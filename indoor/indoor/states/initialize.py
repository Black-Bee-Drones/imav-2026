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

from nectar.control import DroneFactory, MavrosConfig, PoseSource, PIDController
from nectar.vision import ImageHandler
from nectar.ai import Detector, DetectionResult


class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        """
        Initializes the drone, detector and camera.
        """

        self.node = YasminNode.get_instance()

        models_path = pathlib.Path(get_package_share_directory('indoor')) / 'models'

        self.node.declare_parameter('drone_type', Parameter.Type.STRING)
        self.node.declare_parameter('model_source', Parameter.Type.STRING)
        self.node.declare_parameter('confidence_threshold', Parameter.Type.DOUBLE)
        self.node.declare_parameter('image_source', Parameter.Type.STRING)
        self.node.declare_parameter('timeout', Parameter.Type.INTEGER)
        self.node.declare_parameter('timeout_per_state', Parameter.Type.INTEGER)
        self.node.declare_parameter('px_threshold', Parameter.Type.INTEGER)
        self.node.declare_parameter('safe_altitude', Parameter.Type.DOUBLE)
        self.node.declare_parameter('max_altitude', Parameter.Type.DOUBLE)

        self.drone_type = self.node.get_parameter('drone_type').value
        self.model_source = str(models_path / self.node.get_parameter('model_source').value)
        self.confidence_threshold = self.node.get_parameter('confidence_threshold').value
        self.image_source = self.node.get_parameter('image_source').value
        self.timeout = self.node.get_parameter('timeout').value
        self.timeout_per_state = self.node.get_parameter('timeout_per_state').value
        self.px_threshold = self.node.get_parameter('px_threshold').value
        self.safe_altitude = self.node.get_parameter('safe_altitude').value
        self.max_altitude = self.node.get_parameter('max_altitude').value

        # PID xy
        self.node.declare_parameter('controller_xy_kp', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_xy_kd', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_xy_ki', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_xy_output_min', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_xy_output_max', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_xy_integral_min', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_xy_integral_max', Parameter.Type.DOUBLE)

        self.controller_xy_kp = self.node.get_parameter('controller_xy_kp').value
        self.controller_xy_kd = self.node.get_parameter('controller_xy_kd').value
        self.controller_xy_ki = self.node.get_parameter('controller_xy_ki').value
        self.controller_xy_output_min = self.node.get_parameter('controller_xy_output_min').value
        self.controller_xy_output_max = self.node.get_parameter('controller_xy_output_max').value
        self.controller_xy_integral_min = self.node.get_parameter('controller_xy_integral_min').value
        self.controller_xy_integral_max = self.node.get_parameter('controller_xy_integral_max').value

        # PID z
        self.node.declare_parameter('controller_z_kp', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_z_kd', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_z_ki', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_z_output_min', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_z_output_max', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_z_integral_min', Parameter.Type.DOUBLE)
        self.node.declare_parameter('controller_z_integral_max', Parameter.Type.DOUBLE)

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
            yasmin.YASMIN_LOG_INFO('Initializing Start Const...')
            self.start_time = self.node.get_clock().now()

            blackboard['start_time'] = self.start_time
            blackboard['timeout'] = self.timeout
            blackboard['timeout_per_state'] = self.timeout_per_state
            blackboard['px_threshold'] = self.px_threshold
            blackboard['safe_altitude'] = self.safe_altitude
            blackboard['max_altitude'] = self.max_altitude

            yasmin.YASMIN_LOG_INFO('successful Start Const!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Const failed: {e}')
            return ABORT

        # Drone
        try:
            yasmin.YASMIN_LOG_INFO(f'Initializing Drone("{self.drone_type}")...')
            drone_config = MavrosConfig(
                pose_source = PoseSource.VISION,
            )
            drone = DroneFactory.create('mavros', drone_config)

            blackboard['drone'] = drone
            yasmin.YASMIN_LOG_INFO(f'successful start Drone(\'{self.drone_type}\')!')

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
            yasmin.YASMIN_LOG_INFO(f'successful start PID (x, y and z)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'PID failed: {e}')
            return ABORT

        # Detector
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector...')
            self.detector = Detector(
                model_source = self.model_source,
                confidence_threshold = self.confidence_threshold,
            )

            yasmin.YASMIN_LOG_INFO('Load detector...')
            self.detector.load()

            blackboard['detector'] = self.detector
            yasmin.YASMIN_LOG_INFO('successful start Detector!')

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
            yasmin.YASMIN_LOG_INFO('successful start ImageHandler!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'ImageHandler failed: {e}')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
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
