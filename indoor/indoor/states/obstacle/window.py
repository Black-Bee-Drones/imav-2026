from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, PIDController, MoveReference
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult

from ...config import Config


class Window(State):
    def __init__(self, position: str):
        super().__init__(outcomes=[SUCCEED, TIMEOUT, 'reacquire'])

        self.position = position
        self.node = YasminNode.get_instance()


    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        self.timeout: int = config.timeout
        self.start_time: Time = blackboard.get('start_time')

        if self.position == 'room':
            self.start_mission: Time = config.inspect_start_time
            self.mission_timeout: int = config.inspect_timeout
        else:
            self.start_mission: Time = config.obstacle_start_time
            self.mission_timeout: int = config.obstacle_timeout

        drone: MavlinkDrone = blackboard.get('drone')

        handler: ImageHandler = blackboard.get('image_handler_north')
        handler.image_processing_callback = blackboard.get('callback_gate')

        safe_alt = config.safe_alt

        skip = getattr(config, f'obstacle_gate_{self.position}_skip')

        lost_tolerance = config.obstacle_gate_lost_tolerance
        aligned_tolerance = config.obstacle_gate_aligned_tolerance
        aligned_threshold = config.obstacle_gate_aligned_threshold

        result = handler.take_photo()
        if result is None:
            yasmin.YASMIN_LOG_WARN('Fail to get frame. Skip now..')
            skip = True

        pid_y = PIDController(
            kp=config.obstacle_xy_kp,
            kd=config.obstacle_xy_kd,
            ki=config.obstacle_xy_ki,
            setpoint= result.image.shape[1]/2 if (result is not None) else 0,
            output_limits=(-0.3, 0.3)
        )

        pid_z = PIDController(
            kp=config.obstacle_z_kp,
            kd=config.obstacle_z_kd,
            ki=config.obstacle_z_ki,
            setpoint=result.image.shape[0]/2 if (result is not None) else 0,
            output_limits=(-0.1, 0.1)
        )

        output_y = 0.0
        output_z = 0.0
        error_y = 0.0
        error_z = 0.0

        lost = 0
        aligned = 0
        while not skip:
            result: DetectionResult | None = handler.take_photo()
            if result is None:
                continue

            window = result.filter_by_class([config.model_gate_class_name])

            if window:
                lost = 0

                output_y = pid_y.update(window[0].center[0])
                output_z = pid_z.update(window[0].center[1])

                error_y = pid_y._last_error
                error_z = pid_z._last_error

                if abs(error_y) < aligned_tolerance and abs(error_z) < 0.1:
                    aligned += 1

                yasmin.YASMIN_LOG_INFO(

                    f'Centering: ({aligned}/{aligned_threshold}) '
                    f'| Error: y={error_y:.0f}; z={error_z:.0f}. '
                    f'| Output: y={output_y:.1f}; z={output_z:.1f}. '
                )
                drone.move_velocity(0.1, vy=output_y, vz=output_z)

            if aligned >= aligned_threshold:
                break

            else:
                lost += 1
                yasmin.YASMIN_LOG_INFO(
                    f'Lost ({lost}/{lost_tolerance}): '
                    f'| Error: z={error_z:.0f}. '
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

        if self.position == 'first':
            drone.move_to(
                x=0.75,
                y=0.0,
                z=0,
                yaw=0,
                reference=MoveReference.BODY
            )
        else:
            drone.move_to(
                x=6.25,
                y=0.0,
                z=0,
                reference=MoveReference.TAKEOFF
            )

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
