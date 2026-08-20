from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult

from indoor import Config


class ReacquireWindow(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        image_handler_front: ImageHandler = blackboard.get(
            'image_handler_front')
        image_handler_front.image_processing_callback = blackboard.get(
            'callback_detector_gate')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO('Start reacquiring window.')
        if self.check_timeout():
            yasmin.YASMIN_LOG_ERROR('Timeout.')
            return TIMEOUT

        if self._window_detected(image_handler_front):
            yasmin.YASMIN_LOG_INFO('Blue window already detected.')
            return SUCCEED

        for direction, label in ((0.25, 'right'), (-0.25, 'left')):
            yasmin.YASMIN_LOG_INFO(
                f'Window not found. Sweeping {label} while scanning.')

            sweep_start = self.node.get_clock().now()
            while self.node.get_clock().now() - sweep_start < Duration(seconds=2.0):
                if self.check_timeout():
                    yasmin.YASMIN_LOG_ERROR('Timeout.')
                    return TIMEOUT

                if self._window_detected(image_handler_front):
                    yasmin.YASMIN_LOG_INFO(
                        f'Window reacquired while moving {label}.')
                    drone.move_velocity(vx=0, vy=0, vz=0, vyaw=0)
                    return SUCCEED

                drone.move_velocity(vx=0, vy=direction, vz=0, vyaw=0)

                now = self.node.get_clock().now()
                self.node.get_clock().sleep_until(now + Duration(seconds=1/30))

            drone.move_velocity(vx=0, vy=0, vz=0, vyaw=0)

        yasmin.YASMIN_LOG_INFO('Window reacquire maneuver completed.')
        return SUCCEED

    def _window_detected(self, image_handler_front: ImageHandler):
        result: DetectionResult = image_handler_front.take_photo()
        window = result.filter_by_class(['blue_window'])

        return bool(window)

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=config.timeout) or \
            now - \
            self.start_state > Duration(seconds=config.timeout_per_state)
