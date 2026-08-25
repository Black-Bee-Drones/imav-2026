from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone
from nectar.vision import ImageHandler


class Reacquire(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

    def configure(self):
        self.add_input_key('drone')
        self.add_input_key('image_handler_down')
        self.add_input_key('callback_aruco')

        self.add_input_key('timeout')
        self.add_input_key('start_time')
        self.add_input_key('max_alt')

        self.add_input_key('precise_timeout')
        self.add_input_key('precise_start_time')

        self.add_input_key('precise_reacquire_vz')

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')
        handler: ImageHandler = blackboard.get('image_handler_down')
        handler.image_processing_callback = blackboard.get('callback_aruco')

        self.timeout: int = blackboard.get('timeout')
        self.start_time: Time = blackboard.get('start_time')
        max_alt: float = blackboard.get('max_alt')

        self.mission_timeout: int = blackboard.get('precise_timeout')
        self.start_mission: Time = blackboard.get('precise_start_time')
    
        vz: float = blackboard.get('precise_reacquire_vz')

        while (drone.get_altitude() < max_alt):

            if self.check_timeout():
                drone.move_velocity()
                return TIMEOUT

            image, marker_id, translation, yaw = handler.take_photo()
            if marker_id is not None:
                return SUCCEED

            yasmin.YASMIN_LOG_INFO(f'Up...')
            drone.move_velocity(
                x=0,
                y=0,
                z=vz,
                yaw=0,
            )

        return CANCEL

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
