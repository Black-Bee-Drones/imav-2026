from indoor import Config
from indoor.states import map_and_choose_box

from yasmin import Blackboard, State, YASMIN_LOG_INFO
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, CANCEL

from nectar.vision import ImageHandler
from nectar.control import MavlinkDrone, MavrosDrone
from nectar.control.types import AltitudeSource
from nectar.control.vehicle.types import LocalPose

import math
from rclpy.time import Time

class AltitudeSourceError(Exception): ...

class MapBoxes(State):
    def __init__(self) -> None:
        super().__init__(outcomes=[SUCCEED, TIMEOUT, CANCEL])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard) -> str:
        drone: MavlinkDrone | MavrosDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        image_handler_down: ImageHandler = blackboard.get('image_handler_down')
        image_handler_down.image_processing_callback = blackboard.get('callback_box')

        self.start_time: Time = blackboard.get('start_time')
        self.start_state = self.node.get_clock().now()

        altitude_read_counter = 0

        while altitude_read_counter < 10:
            position = drone.position

            if isinstance(position, LocalPose):
                pos_x = position.position.x
                pos_y = position.position.y
                pos_yaw = position.yaw # Already in radians

                position_parsed = pos_x, pos_y, pos_yaw
            else:
                raise TypeError(f'Expected position to be LocalPose type, got {type(position).__name__}')

            altitude = drone.get_altitude(AltitudeSource.LIDAR)
            if altitude is not None:
                map_result = map_and_choose_box(image_handler_down, drone, config, position_parsed)
                if math.isnan(map_result[0][0]):
                    return TIMEOUT
                blackboard.set('box_led_pos',  map_result[0])
                blackboard.set('box_cone_pos', map_result[1])
                return SUCCEED
            else:
                altitude_read_counter += 1
                drone.delay(0.05)
                continue
        return TIMEOUT
