import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, CANCEL

from nectar.control import MavlinkDrone

from indoor import Config


class Drop(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, FAIL, CANCEL])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        try:
            yasmin.YASMIN_LOG_INFO('Start drop.')
            drone.set_actuator(
                index=config.drop_index,
                value=config.drop_value,
            )

            yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
            drone.move_velocity()
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Drop failed: {e}')
            return CANCEL
