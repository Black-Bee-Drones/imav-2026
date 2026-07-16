import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL

from nectar.control import MavrosDrone

from indoor import Config


class Drop(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, FAIL])

        self.config = config

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        yasmin.YASMIN_LOG_INFO('Start drop.')
        drone.set_actuator(
            index = self.config.drop_index,
            value = self.config.drop_value,
        )

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
        drone.move_velocity()
        return SUCCEED
