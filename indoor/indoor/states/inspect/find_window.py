from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult


class FindWindow(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_input_key('timeout')
        self.add_input_key('inspect_timeout')

        self.add_input_key('start_time')
        self.add_input_key('inspect_start_time')

        self.add_input_key('drone')

        self.add_input_key('image_handler_front')
        self.add_input_key('callback_gate')

        self.add_input_key('inspect_back_speed')

    def execute(self, blackboard: Blackboard):
        self.timeout: int = blackboard.get('timeout')
        self.mission_timeout: int = blackboard.get('inspect_timeout')

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = blackboard.get('inspect_start_time')

        drone: MavlinkDrone = blackboard.get('drone')

        handler: ImageHandler = blackboard.get('image_handler_front')
        handler.image_processing_callback = blackboard.get('callback_gate')

        for _ in range(30):
            if self.check_timeout():
                return TIMEOUT

            result: DetectionResult = handler.take_photo()

            if result is not None and result.filter_by_class([
                    blackboard.get('model_gate_class_name')]):
                return SUCCEED

            yasmin.YASMIN_LOG_INFO(f'Go Back...')
            drone.move_velocity(vx=blackboard.get('inspect_back_speed'))

        return CANCEL

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
