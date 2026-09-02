from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference, PIDController
from nectar.vision import LineDetector, ImageHandler

import cv2
import math

from ...config import Config


class BlueBar(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        safe_alt: float = config.safe_alt
        self.start_time: Time = blackboard.get('start_time')
        self.timeout: int = config.timeout

        self.start_mission: Time = config.obstacle_start_time
        self.mission_timeout: int = config.obstacle_timeout

        start_x: float = config.obstacle_start_x
        start_y: float = config.obstacle_start_y

        down_handler: ImageHandler = blackboard.get('image_handler_down')

        match config.obstacle_blue_1:
            case 1:
                alt_1 = config.obstacle_blue_step_alt_1
            case 2:
                alt_1 = config.obstacle_blue_step_alt_2
            case 3:
                alt_1 = config.obstacle_blue_step_alt_3
            case _:
                alt_1 = safe_alt

        match config.obstacle_blue_2:
            case 1:
                alt_2 = config.obstacle_blue_step_alt_1
            case 2:
                alt_2 = config.obstacle_blue_step_alt_2
            case 3:
                alt_2 = config.obstacle_blue_step_alt_3
            case _:
                alt_2 = safe_alt

        points = [
            (0, 0, alt_1),
            (1.0, 0, alt_1),
            (0, 0, alt_2),
            (1.0, 0, alt_2),
        ]

        if self.check_timeout():
            return TIMEOUT

        blue_line = LineDetector(
            color="blue",
        )

        red_line = LineDetector(
            color="red"
        )

        centrilized = False
        error = -100.0
        img = down_handler.take_photo()

        pid_x = PIDController(
            kp=config.obstacle_xy_kp,
            kd=config.obstacle_xy_kd,
            ki=config.obstacle_xy_ki,
            setpoint= img.shape[0]/2 if (img is not None) else 0,
            output_limits=(-0.3, 0.3)
        )

        while not centrilized:

            if abs(error) < 20 and error != 0.0:
                centrilized = True
                yasmin.YASMIN_LOG_INFO(f"Centrilized between lines | Erro {error}")
                break

            img = down_handler.take_photo()

            blue_img, blue_mask, blue_cx, blue_cy, blue_angle, blue_w, blue_h = blue_line.detect_line(img)
            red_img, red_mask, red_cx, red_cy, red_angle, red_w, red_h = red_line.detect_line(img)

            cv2.imwrite(f"/home/jetson/blue_line/f{self.node.get_clock().now()}.jpg", blue_img)
            cv2.imwrite(f"/home/jetson/red_line/f{self.node.get_clock().now()}.jpg", red_img)            

            error = pid_x._last_error

            if math.isnan(red_cy):
                red_cy = 0

            yasmin.YASMIN_LOG_INFO(f"Blue center: {blue_cy} | Red center: {red_cy} | Error: {error}")

            if blue_cy:
                vx = pid_x.update((blue_cy + red_cy)/2)
            else:
                yasmin.YASMIN_LOG_INFO("Blue line not detected")
                vx=0
                break

            drone.move_velocity(vx=vx)

        for x, y, z in points:
            yasmin.YASMIN_LOG_INFO(f"Fly to x = {x} y = {y} z = {z}")
            drone.move_to(
                x=x,
                y=y,
                reference=MoveReference.BODY
            )
            drone.move_to(
                z=z,
                reference=MoveReference.TAKEOFF
            )


        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
