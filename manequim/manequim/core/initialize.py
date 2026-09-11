import yasmin
import numpy as np
import cv2
import os
import pathlib

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from datetime import datetime

from nectar.control import (
    MavlinkConfig,
    DroneFactory,
    MavrosDrone,
    MavlinkDrone,
    MavrosConfig,
    MavlinkConfig,
    PoseSource,
    PIDController,
    RTLMethod,
    SITL_GAZEBO_CONFIG,
    PIDController,
)
from nectar.vision import ImageHandler, ROSConfig, OpenCVConfig
from nectar.ai import Detector, DetectionResult
import manequim.core.constants as config


class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        # Start Simulation Time
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Start time...')
            self.start_time = self.node.get_clock().now()

            blackboard['start_time'] = self.start_time
            yasmin.YASMIN_LOG_INFO('successful Start time!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Start time failed: {e}')
            return ABORT

        # Drone
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Drone...')
            if config.DRONE_TYPE == 'mavros':
                drone_config = MavrosConfig(
                    pose_source=PoseSource.GPS,
                    connection_string=config.CONNECTION_STRING,
                    arm_timeout=15.0
                )

            elif config.DRONE_TYPE == 'mavlink':
                drone_config = MavlinkConfig(
                    pose_source=PoseSource.GPS,
                    connection_string=config.CONNECTION_STRING,
                    arm_timeout=15.0,
                )
            else:
                yasmin.YASMIN_LOG_ERROR('Invalid drone_type.')
                return ABORT
            if config.SIM_MODE:
                drone = DroneFactory.create("mavros", SITL_GAZEBO_CONFIG)
            else:
                drone = DroneFactory.create(config.DRONE_TYPE, drone_config)

            blackboard['drone'] = drone
            yasmin.YASMIN_LOG_INFO(f'Successful start Drone("{config.DRONE_TYPE}")!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'DroneFactory failed: {e}')
            return ABORT

        # PID controller
        try:
            yasmin.YASMIN_LOG_INFO("Initializing PID Controller...")
            pid_cx = PIDController(
                kp=getattr(config, 'X_KP', 1.0),
                ki=getattr(config, 'X_KI', 0.0),
                kd=getattr(config, 'X_KD', 0.0),
                output_limits=getattr(config, 'XY_OUTPUT_LIM', 1.0),
                integral_limits=getattr(config, 'XY_INTEGRAL_LIM', 1.0),
            )
            pid_cy = PIDController(
                kp=getattr(config, 'Y_KP', 1.0),
                ki=getattr(config, 'Y_KI', 0.0),
                kd=getattr(config, 'Y_KD', 0.0),
                output_limits=getattr(config, 'XY_OUTPUT_LIM', 1.0),
                integral_limits=getattr(config, 'XY_INTEGRAL_LIM', 1.0),
            )
            blackboard["pid_cx"] = pid_cx
            blackboard["pid_cy"] = pid_cy
            yasmin.YASMIN_LOG_INFO(f'Successful start PID (x and y)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'PID failed: {e}')
            return ABORT

        # Detector - mannequin
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector(mannequin)...')
            self.detector_mannequin = Detector(
                model_source=config.DETECTOR_MODEL_SOURCE,
                confidence_threshold=config.DETECTOR_CONFIDENCE_THRESHOLD,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(mannequin)...')
            self.detector_mannequin.load()

            blackboard['detector_mannequin'] = self.detector_mannequin
            yasmin.YASMIN_LOG_INFO('Successful start Detector(mannequin)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Detector(mannequin) failed: {e}')
            return ABORT

        # Camera (Image Handler)
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Camera...')
            if (config.SIM_MODE):
                image_source = config.SIM_IMAGE_SOURCE
                cam_config = ROSConfig(
                    topic=config.SIM_IMAGE_SOURCE,
                    compressed=getattr(config, 'IMAGE_COMPRESSED', False),
                )
            else:
                image_source = config.IMAGE_SOURCE
                cam_config = OpenCVConfig(
                    width=config.IMAGE_WIDTH,
                    height=config.IMAGE_HEIGHT,
                )

            camera = ImageHandler(
                image_source=image_source,
                config=cam_config,
                image_processing_callback=self.detector_mannequin_callback,
            )

            yasmin.YASMIN_LOG_INFO('Open camera...')
            camera.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo...')
            frame_test = camera.take_photo()
            if frame_test is None:
                yasmin.YASMIN_LOG_ERROR("Failed to get frame from camera.")
                return ABORT

            blackboard['camera'] = camera
            yasmin.YASMIN_LOG_INFO('Successful start camera!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Camera failed: {e}')
            return ABORT
        return SUCCEED


    def detector_mannequin_callback(self, image : np.ndarray):
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(
        self.node.get_clock().now().nanoseconds / 1e9)

        pkg_path = pathlib.Path.home() / 'ros2_ws/imav-2026/manequim' / \
            start.strftime('mannequin-%Y-%m-%d_%H-%M-%S')
        raw_path = pkg_path / 'images/mannequin'
        annotated_path = pkg_path / 'images/mannequin_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / \
            now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(pkg_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        result = self.detector_mannequin.detect(image, conf=config.DETECTOR_CONFIDENCE_THRESHOLD)
        result.image = image
        result.annotated_image = self.detector_mannequin.draw_detections(image, result)

        cv2.imwrite(str(raw_file), result.image)
        cv2.imwrite(str(annotated_file), result.annotated_image)
        return result
