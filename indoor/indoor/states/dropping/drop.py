import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL

from nectar.control import MavrosDrone


class Drop(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, FAIL])

        self.node = YasminNode.get_instance()

        self.drop_index: int | float = self.node.get_parameter('drop_index').value
        self.drop_value: int | float = self.node.get_parameter('drop_value').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        yasmin.YASMIN_LOG_INFO('Start drop.')
        drone.set_actuator(
            index = self.drop_index,
            value = self.drop_value,
        )

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
        drone.move_velocity()
        return SUCCEED
