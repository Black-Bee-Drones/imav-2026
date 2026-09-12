from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT

from nectar.control import MavlinkDrone, PIDController, MoveReference
from nectar.vision import ImageHandler

from indoor import Config


class CenterFixed(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, FAIL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        output_limits = (
            config.obstacle_gate_output_min,
            config.obstacle_gate_output_max,
        )

        image_handler_down: ImageHandler = blackboard.get('image_handler_down')
        image_handler_down.image_processing_callback = blackboard.get(
            'callback_land_aruco')

        pid_x = PIDController(
            kp=config.precise_xy_kp,
            kd=config.obstacle_xy_kd,
            ki=config.precise_xy_ki,
            setpoint=0.0,
            output_limits=output_limits,
        )
        pid_y = PIDController(
            kp=config.precise_xy_kp,
            kd=config.obstacle_xy_kd,
            ki=config.precise_xy_ki,
            setpoint=0.0,
            output_limits=output_limits,
        )
        pid_z = PIDController(
            kp=config.obstacle_alt_kp,
            kd=config.obstacle_xy_kd,
            ki=config.obstacle_xy_ki,
            setpoint=0.0,
            output_limits=output_limits,
        )
        pid_yaw = PIDController(
            kp=config.obstacle_xy_kp,
            kd=config.obstacle_xy_kd,
            ki=config.obstacle_xy_ki,
            setpoint=0.0,
            output_limits=output_limits,
        )

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        pid_x.reset()
        pid_y.reset()
        pid_z.reset()
        pid_yaw.reset()

        yasmin.YASMIN_LOG_INFO(
            'Center state started: beginning yaw alignment phase.')
        if self.check_timeout(config):
            yasmin.YASMIN_LOG_ERROR(
                'Timeout before yaw alignment could start.')
            return TIMEOUT

        lost_count = 0
        while True:
            now = self.node.get_clock().now()
            image, marker_id, translation, error_yaw = image_handler_down.take_photo()

            if marker_id is not None:
                error_yaw %= 90 # correction
                if error_yaw > 45:
                    error_yaw = abs(error_yaw - 90)
                lost_count = 0

                error_y, error_x, _ = translation
                error_z = drone.get_altitude()

                if (error_x**2 + error_y**2) <= config.center_threshold_xy**2:
                    output_z = config.precise_descend_vz
                    if drone.get_altitude() <= config.land_altitude:
                        yasmin.YASMIN_LOG_INFO('Altitude settled, landing...')
                        return SUCCEED
                else:
                    output_z = 0

                output_x = pid_x.update(error_x - config.precise_x_offset)
                output_y = pid_y.update(error_y - config.precise_y_offset)
                output_yaw = pid_yaw.update(error_yaw)

                # Published so the aruco callback's debug HUD (drawn on the
                # *next* frame) can show this loop's error/output values.
                blackboard.set('align_debug', {
                    'error_x': error_x,
                    'error_y': error_y,
                    'error_z': error_z,
                    'output_x': output_x,
                    'output_y': output_y,
                    'output_z': output_z,
                    'output_yaw': output_yaw,
                    'lost_count': lost_count,
                    'lost_tolerance': config.lost_tolerance,
                    'within_xy': (error_x**2 + error_y**2) <= config.center_threshold_xy**2,
                    'landing': drone.get_altitude() <= config.land_altitude,
                })

                yasmin.YASMIN_LOG_INFO(
                    'Error:'
                    f'x={error_x:.2f};'
                    f'y={error_y:.2f};'
                    f'z={error_z:.2f};'
                    f'yaw={error_yaw:.2f}.'
                )

                yasmin.YASMIN_LOG_INFO(
                    'Output:'
                    f'vx={output_x:.2f};'
                    f'vy={output_y:.2f};'
                    f'vz={output_z:.2f}'
                    f'vyaw={output_yaw:.2f}'
                )

                drone.move_velocity(
                    vx=output_x,
                    vy=output_y,
                    vz=output_z,
                    vyaw=output_yaw
                )

            else:
                yasmin.YASMIN_LOG_INFO(
                    f'Lost detection ({lost_count}/{config.lost_tolerance}).')
                lost_count += 1
                blackboard.set('align_debug', {
                    'lost_count': lost_count,
                    'lost_tolerance': config.lost_tolerance,
                })

                if config.lost_tolerance <= lost_count:
                    yasmin.YASMIN_LOG_INFO('Lost detection exceeded.')
                    drone.move_velocity()
                    return FAIL

            if self.check_timeout(config):
                yasmin.YASMIN_LOG_ERROR('Timeout during yaw alignment.')
                drone.move_velocity()
                return TIMEOUT

            self.node.get_clock().sleep_until(now + Duration(seconds=1 / 30))

    def check_timeout(self, config: Config):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=config.timeout) or \
            now - \
            self.start_state > Duration(seconds=config.precise_timeout)