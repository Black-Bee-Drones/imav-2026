from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from ...config import Config

class ToBarCorridor(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        drone: MavlinkDrone = blackboard.get("drone")

        safe_alt: float = config.safe_alt
        self.start_time: Time = blackboard.get("start_time")
        self.timeout: int = config.timeout

        self.start_mission: Time = config.obstacle_start_time
        self.mission_timeout: int = config.obstacle_timeout

        if config.obstacle_after_first_skip:
            return CANCEL

        start_x: float = config.obstacle_bar_start_x
        start_y: float = config.obstacle_start_y

        match config.obstacle_red:
            case 1:
                alt: float = config.obstacle_red_step_alt_1
            case 2:
                alt: float = config.obstacle_red_step_alt_2
            case 3:
                alt: float = config.obstacle_red_step_alt_3
            case _:
                alt: float = safe_alt

        points = [
            (None, None, alt),
            (start_x, start_y, alt),
        ]

        if self.check_timeout():
            return TIMEOUT

        for x, y, z in points:
            yasmin.YASMIN_LOG_INFO(
                f"Bar corridor: x={x} y={y} z={z:.2f} (takeoff, yaw hold)."
            )
            drone.move_to(
                x=x,
                y=y,
                z=z,
                yaw=None,
                reference=MoveReference.TAKEOFF,
                precision=0.12,
            )

            if self.check_timeout():
                return TIMEOUT

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(
            seconds=self.timeout
        ) or now - self.start_mission > Duration(seconds=self.mission_timeout)
