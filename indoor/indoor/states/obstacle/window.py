from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, PIDController
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult


class Window(State):
    def __init__(self, position: str):
        super().__init__(outcomes=[SUCCEED, TIMEOUT, 'reacquire'])

        self.position = position
        self.node = YasminNode.get_instance()

    def configure(self) -> None:
        self.add_input_key('timeout')
        self.add_input_key('start_time')

        self.add_input_key('model_gate_class_name')

        if self.position == 'room':
            self.add_input_key('inspect_timeout')
            self.add_input_key('inspect_start_time')
        else:
            self.add_input_key('obstacle_timeout')
            self.add_input_key('obstacle_start_time')

        self.add_input_key('drone')

        self.add_input_key('image_handler_front')
        self.add_input_key('callback_gate')

        self.add_input_key('safe_alt')

        self.add_input_key(f'obstacle_gate_{self.position}_skip')
        self.add_input_key('obstacle_gate_alt')

        self.add_input_key('obstacle_gate_lost_tolerance')
        self.add_input_key('obstacle_gate_aligned_tolerance')
        self.add_input_key('obstacle_gate_aligned_threshold')

        self.add_input_key('obstacle_xy_kp')
        self.add_input_key('obstacle_xy_kd')
        self.add_input_key('obstacle_xy_ki')

        self.add_input_key('obstacle_z_kp')
        self.add_input_key('obstacle_z_kd')
        self.add_input_key('obstacle_z_ki')

    def execute(self, blackboard: Blackboard):
        self.timeout: int = blackboard.get('timeout')
        self.start_time: Time = blackboard.get('start_time')

        if self.position == 'room':
            self.start_mission: Time = blackboard.get('inspect_timeout')
            self.mission_timeout: int = blackboard.get('inspect_start_time')
        else:
            self.start_mission: Time = blackboard.get('obstacle_start_time')
            self.mission_timeout: int = blackboard.get('obstacle_timeout')

        drone: MavlinkDrone = blackboard.get('drone')

        handler: ImageHandler = blackboard.get('image_handler_front')
        handler.image_processing_callback = blackboard.get('callback_gate')

        safe_alt = blackboard.get('safe_alt')

        skip = blackboard.get(f'obstacle_gate_{self.position}_skip')

        lost_tolerance = blackboard.get('obstacle_gate_lost_tolerance')
        aligned_tolerance = blackboard.get('obstacle_gate_aligned_tolerance')
        aligned_threshold = blackboard.get('obstacle_gate_aligned_threshold')

        result = handler.take_photo()
        if result is None:
            yasmin.YASMIN_LOG_WARN('Fail to get frame. Skip now..')
            skip = True

        pid_y = PIDController(
            kp=blackboard.get('obstacle_xy_kp'),
            kd=blackboard.get('obstacle_xy_kd'),
            ki=blackboard.get('obstacle_xy_ki'),
            setpoint= result.image.shape[1]/2 if (result is not None) else 0,
        )

        pid_z = PIDController(
            kp=blackboard.get('obstacle_z_kp'),
            kd=blackboard.get('obstacle_z_kd'),
            ki=blackboard.get('obstacle_z_ki'),
            setpoint=blackboard.get('obstacle_gate_alt'),
        )

        lost = 0
        aligned = 0
        while not skip:
            result: DetectionResult | None = handler.take_photo()
            if result is None:
                continue

            window = result.filter_by_class([blackboard.get('model_gate_class_name')])

            if window:
                lost = 0

                output_y = pid_y.update(window[0].center[0])
                output_z = pid_z.update(drone.get_altitude())

                error_y = pid_y._last_error
                error_z = pid_z._last_error

                if abs(error_y) < aligned_tolerance and abs(error_z) < 0.1:
                    aligned += 1

                yasmin.YASMIN_LOG_INFO(

                    f'Centering: ({aligned}/{aligned_threshold}) ',
                    f'| Error: y={error_y:.0f}; z={error_z:.0f}. ',
                    f'| Output: y={output_y:.1f}; z={output_z:.1f}. '
                )
                drone.move_velocity(vy=output_y, vz=output_z)

            if aligned >= aligned_threshold:
                break

            else:
                lost += 1
                yasmin.YASMIN_LOG_INFO(
                    f'Lost ({lost}/{lost_tolerance}): ',
                    f'| Error: z={pid_z._last_error:.0f}. ',
                    f'| Output: z={output_z:.1f}. '
                )
                drone.move_velocity(vz=output_z)

                if lost >= lost_tolerance:
                    return 'reacquire'

            if self.check_timeout():
                return TIMEOUT

        else:
            yasmin.YASMIN_LOG_INFO('Fly to safe altitude.')
            drone.move_to(
                x=0,
                y=0,
                z=safe_alt - drone.get_altitude(),
                yaw=0,
            )

        if self.check_timeout():
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO('Fly through the window.')
        drone.move_to(
            x=0.75,
            y=0,
            z=0,
            yaw=0,
        )

        yasmin.YASMIN_LOG_INFO('Fly to gate alt.')
        drone.move_to(
            x=0,
            y=0,
            z=blackboard.get('obstacle_gate_alt') - drone.get_altitude(),
            yaw=0,
        )

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
