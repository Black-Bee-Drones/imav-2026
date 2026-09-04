from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import pathlib
import queue
import threading
import traceback

import cv2 as cv
import numpy as np
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import ABORT, SUCCEED
from yasmin_ros.yasmin_node import YasminNode

from nectar.ai import DetectionResult, Detector
from nectar.control import DroneFactory, MavlinkConfig, MavrosConfig, PoseSource
from nectar.vision import (
    Aruco,
    DepthCam,
    ImageHandler,
    OpenCVConfig,
    RealSenseConfig,
    ROSConfig,
    ROSDepthConfig,
)

from ...config import Config

_JPEG_QUALITY = 80


def _camera_config(source: str, config: Config, which: str):
    if source in (None, '', 'skip'):
        return None
    if which == 'north':
        topic, compressed, device_index = (
            config.camera_north_topic,
            config.camera_north_is_compressed,
            config.camera_north_id,
        )
        depth_topic = config.camera_north_depth_topic
    elif which == 'south':
        topic, compressed, device_index = (
            config.camera_south_topic,
            config.camera_south_is_compressed,
            config.camera_south_id,
        )
        depth_topic = None
    else:
        topic, compressed, device_index = (
            config.camera_down_topic,
            config.camera_down_is_compressed,
            config.camera_down_id,
        )
        depth_topic = None

    match source:
        case 'ros_depth':
            return ROSDepthConfig(
                topic=topic,
                compressed=compressed,
                depth_topic=depth_topic,
                depth_compressed=False,
                enable_depth=True,
            )
        case 'ros':
            return ROSConfig(topic=topic, compressed=compressed)
        case 'realsense':
            return RealSenseConfig()
        case 'opencv':
            return OpenCVConfig(device_index=device_index)
        case _:
            raise ValueError(f'invalid camera source "{source}"')


class Initialize(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()
        self.start_time = self.node.get_clock().now()
        self.config = config
        self._debug_root: pathlib.Path | None = None
        self._save_dirs: set[str] = set()
        self._save_q: queue.Queue | None = None
        self._save_thread: threading.Thread | None = None

    def execute(self, blackboard: Blackboard):
        config = self.config
        blackboard.set('config', config)

        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        self._debug_root = (
            pathlib.Path.home()
            / 'ros2_ws'
            / 'imav26-indoor'
            / start.strftime('%Y-%m-%d_%H-%M-%S')
        )
        self._debug_root.mkdir(parents=True, exist_ok=True)
        blackboard.set('debug_root', self._debug_root)
        self._save_q = queue.Queue(maxsize=24)
        self._save_thread = threading.Thread(target=self._save_worker, daemon=True)
        self._save_thread.start()
        blackboard.set('save_jpg', self._save_jpg)

        yasmin.YASMIN_LOG_INFO('Initializing...')

        def load_detector(label: str, source: str, conf: float) -> Detector:
            yasmin.YASMIN_LOG_INFO(f'Load Detector({label})...')
            detector = Detector(model_source=source, confidence_threshold=conf)
            detector.load()
            yasmin.YASMIN_LOG_INFO(f'successful start Detector({label})!')
            return detector

        try:
            with ThreadPoolExecutor(max_workers=3) as pool:
                fut_gate = pool.submit(
                    load_detector, 'gate', config.model_gate_source, config.model_gate_conf
                )
                fut_baby = pool.submit(
                    load_detector, 'baby', config.model_baby_source, config.model_baby_conf
                )
                fut_box = pool.submit(
                    load_detector, 'box', config.model_box_source, config.model_box_conf
                )

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

                self.detector_gate = fut_gate.result()
                self.detector_baby = fut_baby.result()
                self.detector_box = fut_box.result()

            blackboard.set('detector_gate', self.detector_gate)
            blackboard.set('callback_gate', self.callback_detector_gate)
            blackboard.set('detector_baby', self.detector_baby)
            blackboard.set('callback_baby', self.callback_detector_baby)
            blackboard.set('detector_box', self.detector_box)
            blackboard.set('callback_box', self.callback_detector_box)
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Initialize failed: {e}')
            yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
            return ABORT

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

        if not self._open_camera(blackboard, 'north', required=True):
            return ABORT
        if not self._open_camera(blackboard, 'south', required=False):
            return ABORT
        if not self._open_camera(blackboard, 'down', required=True):
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
        if not config.sitl:
            try:
                input('\n[PAUSE] Press ENTER to proceed to TAKEOFF...\n')
            except (KeyboardInterrupt, EOFError):
                yasmin.YASMIN_LOG_WARN('Takeoff cancelled by user.')
                return ABORT
        return SUCCEED

    def _open_camera(self, blackboard: Blackboard, which: str, required: bool) -> bool:
        config = self.config
        source = getattr(config, f'camera_{which}_source')
        yasmin.YASMIN_LOG_INFO(f'Initializing ImageHandler({which})...')
        try:
            handler_config = _camera_config(source, config, which)
            if handler_config is None:
                yasmin.YASMIN_LOG_WARN(f'ImageHandler({which}) skipped.')
                blackboard.set(f'image_handler_{which}', None)
                if which == 'north':
                    blackboard.set('depth_cam', None)
                return True

            handler = ImageHandler(image_source=source, config=handler_config)
            yasmin.YASMIN_LOG_INFO(f'Open camera ({which})...')
            handler.open()
            yasmin.YASMIN_LOG_INFO(f'Take testing photo ({which})...')
            result = handler.take_photo()
            if result is None:
                yasmin.YASMIN_LOG_WARN(f'ImageHandler({which}): no test frame.')

            blackboard.set(f'image_handler_{which}', handler)
            if which == 'north':
                depth_cam = (
                    handler.camera if isinstance(handler.camera, DepthCam) else None
                )
                blackboard.set('depth_cam', depth_cam)
            yasmin.YASMIN_LOG_INFO(f'successful start ImageHandler({which})!')
            return True
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return False
        except Exception as e:
            if required:
                yasmin.YASMIN_LOG_ERROR(f'ImageHandler({which}) failed: {e}')
                yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
                return False
            yasmin.YASMIN_LOG_WARN(f'ImageHandler({which}) failed, continuing: {e}')
            blackboard.set(f'image_handler_{which}', None)
            return True

    def _save_worker(self) -> None:
        while True:
            item = self._save_q.get()
            if item is None:
                return
            path, image = item
            cv.imwrite(str(path), image, [int(cv.IMWRITE_JPEG_QUALITY), _JPEG_QUALITY])

    def _save_jpg(self, folder: str, stem: str, image: np.ndarray) -> None:
        if image is None or self._debug_root is None or self._save_q is None:
            return
        if folder not in self._save_dirs:
            (self._debug_root / folder).mkdir(parents=True, exist_ok=True)
            self._save_dirs.add(folder)
        now = datetime.fromtimestamp(self.node.get_clock().now().nanoseconds / 1e9)
        path = self._debug_root / folder / now.strftime(f'{stem}-%Y-%m-%d_%H-%M-%S-%f.jpg')
        try:
            self._save_q.put_nowait((path, image.copy()))
        except queue.Full:
            pass

    def callback_detector_gate(self, image: np.ndarray) -> DetectionResult:
        result = self.detector_gate.detect(image)
        result.image = image
        annotated = self.detector_gate.draw_detections(image, result)
        result.annotated_image = annotated
        self._save_jpg('gate', 'raw', image)
        return result

    def callback_detector_box(self, image: np.ndarray) -> DetectionResult:
        result = self.detector_box.detect(image)
        result.image = image
        annotated = self.detector_box.draw_detections(image, result)
        result.annotated_image = annotated
        self._save_jpg('box', 'raw', image)
        self._save_jpg('box_annotated', 'annotated', annotated)
        return result

    def callback_detector_baby(self, image: np.ndarray) -> DetectionResult:
        result = self.detector_baby.detect(image)
        result.image = image
        annotated = self.detector_baby.draw_detections(image, result)
        result.annotated_image = annotated
        self._save_jpg('baby', 'raw', image)
        self._save_jpg('baby_annotated', 'annotated', annotated)
        return result

    def callback_aruco(self, image: np.ndarray):
        self._save_jpg('aruco', 'raw', image)
        marker_id, translation, yaw = self.aruco.pose_estimate(image, draw=True)
        self._save_jpg('aruco_annotated', 'annotated', image)
        return image, marker_id, translation, yaw
