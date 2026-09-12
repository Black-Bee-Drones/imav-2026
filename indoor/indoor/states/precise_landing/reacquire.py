from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone
from nectar.vision import ImageHandler

from ...config import Config


class Reacquire(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()


    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        drone: MavlinkDrone = blackboard.get('drone')
        handler: ImageHandler = blackboard.get('image_handler_down')
        handler.image_processing_callback = blackboard.get('callback_land_aruco')

        max_alt: float = config.max_alt
        self.start_time: Time = blackboard.get('start_time')
        self.timeout: int = config.timeout

        self.start_mission: Time = config.precise_start_time
        self.mission_timeout: int = config.precise_timeout

        vz: float = config.precise_reacquire_vz

        while (drone.get_altitude() < max_alt):
            if self.check_timeout():
                drone.move_velocity()
                return TIMEOUT

            image, marker_id, translation, yaw = handler.take_photo()
            if marker_id is not None:
                return SUCCEED

            yasmin.YASMIN_LOG_INFO(f'Up...')
            drone.move_velocity(vz=vz)
            blackboard.set('align_debug', {'output_z': vz})

        return CANCEL

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)