from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, PIDController
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult


class ReacquireWindow(State):
    def __init__(self, position):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.position = position
        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_output_key(f'obstacle_gate_{self.position}_skip')

        self.add_input_key('drone')
        self.add_input_key('image_handler_front')
        self.add_input_key('callback_gate')

        self.add_input_key('model_gate_class_name')
    
        self.add_input_key('start_time')
        self.add_input_key('timeout')

        self.add_input_key('obstacle_start_time')
        self.add_input_key('obstacle_timeout')

        self.add_input_key('obstacle_z_kp')
        self.add_input_key('obstacle_z_kd')
        self.add_input_key('obstacle_z_ki')

        self.add_input_key('obstacle_gate_alt')

    def execute(self, blackboard: Blackboard):

        self.timeout: int = blackboard.get('timeout')
        self.mission_timeout: int = blackboard.get('obstacle_timeout')

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = blackboard.get('obstacle_start_time')

        drone: MavlinkDrone = blackboard.get('drone')

        handler: ImageHandler = blackboard.get('image_handler_front')
        handler.image_processing_callback = blackboard.get('callback_gate')

        pid_z = PIDController(
            kp=blackboard.get('obstacle_z_kp'),
            kd=blackboard.get('obstacle_z_kd'),
            ki=blackboard.get('obstacle_z_ki'),
            setpoint=blackboard.get('obstacle_gate_alt'),
        )

        for direction, label in ((0.25, 'right'), (-0.25, 'left')):
            yasmin.YASMIN_LOG_INFO(
                f'Window not found. Sweeping {label} while scanning.')

            start = self.node.get_clock().now()
            while self.node.get_clock().now() - start < Duration(seconds=2.0):
                result: DetectionResult | None = handler.take_photo()
                if result is not None and result.filter_by_class([
                        blackboard.get('model_gate_class_name')]):
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
