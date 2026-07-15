import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone


class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        """
        Take off the drone to the target altitude.
        """

        self.node = YasminNode.get_instance()

        self.altitude: int | float = self.node.get_parameter('takeoff_altitude').value

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard['drone']

        yasmin.YASMIN_LOG_INFO(f'Taking off to altitude: {self.altitude} m...')

        try:
            drone.takeoff(self.altitude)

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Taking off failed: {e}')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED
