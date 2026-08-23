import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavlinkDrone


class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_output_key('start_time')

        self.add_input_key('drone')

        self.add_input_key('takeoff_alt')

    def execute(self, blackboard: Blackboard):
        blackboard.set('start_time', self.node.get_clock().now())
        drone: MavlinkDrone = blackboard.get('drone')

        takeoff_alt: float = blackboard.get('takeoff_alt')

        yasmin.YASMIN_LOG_INFO(f'Taking off (altitude={takeoff_alt} m)...')
        drone.takeoff(takeoff_alt)

        return SUCCEED
