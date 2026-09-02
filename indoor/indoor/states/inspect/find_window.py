from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult

from ...config import Config


class FindWindow(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()


    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        self.timeout: int = config.timeout
        self.mission_timeout: int = config.inspect_timeout

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = config.inspect_start_time

        drone: MavlinkDrone = blackboard.get('drone')

        handler: ImageHandler = blackboard.get('image_handler_north')
        handler.image_processing_callback = blackboard.get('callback_gate')

        for _ in range(30):
            if self.check_timeout():
                return TIMEOUT

            result: DetectionResult = handler.take_photo()

            if result is not None and result.filter_by_class([
                    config.model_gate_class_name]):
                return SUCCEED

            yasmin.YASMIN_LOG_INFO(f'Go Back...')
            drone.move_velocity(vx=config.inspect_back_speed)

        return CANCEL

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
