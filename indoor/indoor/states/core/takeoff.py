import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavlinkDrone
from ...config import Config


class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        blackboard.set('start_time', self.node.get_clock().now())
        drone: MavlinkDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        takeoff_alt: float = config.takeoff_alt

        yasmin.YASMIN_LOG_INFO(f'Taking off (altitude={takeoff_alt} m)...')
        drone.takeoff(takeoff_alt)

        return SUCCEED
