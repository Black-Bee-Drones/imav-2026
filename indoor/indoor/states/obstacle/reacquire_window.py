from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, PIDController
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult

from ...config import Config


class ReacquireWindow(State):
    def __init__(self, position):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.position = position
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')

        self.timeout: int = config.timeout
        self.mission_timeout: int = config.obstacle_timeout

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = config.obstacle_start_time

        drone: MavlinkDrone = blackboard.get('drone')

        handler: ImageHandler = blackboard.get('image_handler_north')
        handler.image_processing_callback = blackboard.get('callback_gate')

        pid_z = PIDController(
            kp=config.obstacle_z_kp,
            kd=config.obstacle_z_kd,
            ki=config.obstacle_z_ki,
            setpoint=config.obstacle_gate_alt,
        )

        for direction, label in ((0.25, 'right'), (-0.25, 'left')):
            yasmin.YASMIN_LOG_INFO(
                f'Window not found. Sweeping {label} while scanning.')

            start = self.node.get_clock().now()
            while self.node.get_clock().now() - start < Duration(seconds=2.0):
                result: DetectionResult | None = handler.take_photo()
                if result is not None and result.filter_by_class([
                        config.model_gate_class_name]):
                    drone.move_velocity()
                    return SUCCEED

                if self.check_timeout():
                    return TIMEOUT

                output_z = pid_z.update(drone.get_altitude())
                drone.move_velocity(vy=direction, vz=output_z, duration=2.0)

        blackboard.set(f'obstacle_gate_{self.position}_skip', True)
        return CANCEL

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
