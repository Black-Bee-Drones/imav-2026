import datetime
from traceback import print_exc

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from nectar.control import (
    MavlinkConfig,
    DroneFactory,
    MavrosConfig,
    PoseSource,
    SITL_GAZEBO_CONFIG,
)
from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision.camera import ROSConfig
from nectar.ai import Detector

from core.constants import(
    SIM_MODE,
    CAMERA_SOURCE,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    DETECTOR_MODEL_SOURCE,
    DETECTOR_CONFIDENCE_THRESHOLD,
    DRONE_TYPE,
    CONNECTION_STRING,
)

class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.node = YasminNode.get_instance()

        timestamp = self.node.get_clock().now().nanoseconds / 1e9
        now = datetime.datetime.fromtimestamp(timestamp)
        self.photos_folder = now.strftime('manequim_search-%Y-%m-%d-%H-%M')

    def execute(self, blackboard: Blackboard):
        try:
            # ---- Yasmin ----
            node = YasminNode.get_instance()
            yasmin.YASMIN_LOG_INFO(f'Inicializing Drone Config ("{DRONE_TYPE}")...')

            # ---- Nectar ----
            
            if SIM_MODE :
                config = SITL_GAZEBO_CONFIG
            
            elif DRONE_TYPE == "mavlink":
                MavlinkConfig(
                    pose_source=PoseSource.GPS,
                    connection_string=CONNECTION_STRING
                )
            elif DRONE_TYPE == "mavros" :
                MavrosConfig(
                    pose_source=PoseSource.GPS,
                    connection_string=CONNECTION_STRING
                )
            else :
                yasmin.YASMIN_LOG_INFO('\033[31m Invalid Drone Type!\033[0m')
                return ABORT

            # config = (
            #     SITL_GAZEBO_CONFIG if SIM_MODE
            #     else MavlinkConfig(connection_string=CONNECTION_STRING)
            # )
            
            drone = DroneFactory.create(DRONE_TYPE, config, node._executor)

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_ERROR('Execution interrupted by user!')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Drone failed: {e}")
            print_exc()
            return ABORT

        try:
            yasmin.YASMIN_LOG_INFO("Initializing Detector...")

            self.detector = Detector( #Creates the detector
                model_source= DETECTOR_MODEL_SOURCE,
                confidence_threshold= DETECTOR_CONFIDENCE_THRESHOLD,
            )
        
            yasmin.YASMIN_LOG_INFO("Loading the Detector...")
            self.detector.load()
        
            blackboard["detector"] = self.detector
            yasmin.YASMIN_LOG_INFO("Detector succesfully loaded.")
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Detector failed: {e}")
            return ABORT

        try:
            yasmin.YASMIN_LOG_INFO("Initializing Camera...")

            if SIM_MODE:
                cam_config = ROSConfig(topic=CAMERA_SOURCE, compressed=False)
            else:
                cam_config = OpenCVConfig(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)

            camera = ImageHandler(
                image_source=CAMERA_SOURCE,
                config=cam_config,
                image_processing_callback=self.camera_callback,
            )
            
            camera.open()
            frame = camera.take_photo()

            if frame is None:
                yasmin.YASMIN_LOG_ERROR("Failed to get frame from camera.")
                return ABORT

            blackboard["camera"] = camera
            yasmin.YASMIN_LOG_INFO(f"Camera ready!")

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Camera failed: {e}")
            return ABORT

        return SUCCEED

    def camera_callback(self, image): #Runs everytime we call camera.take_photo()
        try:
            result = self.detector.detect(image) #Runs the detector on the frame
            result.image = image
            
            return result
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Detector not working: {e}")
            return image