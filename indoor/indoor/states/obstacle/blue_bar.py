from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference, PIDController
from nectar.vision import LineDetector, ImageHandler

import cv2
import math


class BlueBar(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_input_key('drone')

        self.add_input_key('safe_alt')
        self.add_input_key('start_time')
        self.add_input_key('timeout')

        self.add_input_key('obstacle_start_time')
        self.add_input_key('obstacle_timeout')

        self.add_input_key('obstacle_start_x')
        self.add_input_key('obstacle_start_y')
        self.add_input_key('obstacle_blue_step_alt_1')
        self.add_input_key('obstacle_blue_step_alt_2')
        self.add_input_key('obstacle_blue_step_alt_3')
        self.add_input_key('obstacle_blue_1')
        self.add_input_key('obstacle_blue_2')

        self.add_input_key('image_handler_down')

        self.add_input_key('obstacle_xy_kp')
        self.add_input_key('obstacle_xy_kd')
        self.add_input_key('obstacle_xy_ki')

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')

        safe_alt: float = blackboard.get('safe_alt')
        self.start_time: Time = blackboard.get('start_time')
        self.timeout: int = blackboard.get('timeout')

        self.start_mission: Time = blackboard.get('obstacle_start_time')
        self.mission_timeout: int = blackboard.get('obstacle_timeout')

        start_x: float = blackboard.get('obstacle_start_x')
        start_y: float = blackboard.get('obstacle_start_y')

        down_handler: ImageHandler = blackboard.get('image_handler_down')

        match blackboard.get('obstacle_blue_1'):
            case 1:
                alt_1 = blackboard.get('obstacle_blue_step_alt_1')
            case 2:
                alt_1 = blackboard.get('obstacle_blue_step_alt_2')
            case 3:
                alt_1 = blackboard.get('obstacle_blue_step_alt_3')
            case _:
                alt_1 = safe_alt

        match blackboard.get('obstacle_blue_2'):
            case 1:
                alt_2 = blackboard.get('obstacle_blue_step_alt_1')
            case 2:
                alt_2 = blackboard.get('obstacle_blue_step_alt_2')
            case 3:
                alt_2 = blackboard.get('obstacle_blue_step_alt_3')
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
            kp=blackboard.get('obstacle_xy_kp'),
            kd=blackboard.get('obstacle_xy_kd'),
            ki=blackboard.get('obstacle_xy_ki'),
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
