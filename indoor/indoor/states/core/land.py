import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone


class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        """
        Land the drone.
        """

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        yasmin.YASMIN_LOG_INFO('Landing...')

        try:
            drone.land()

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Landing failed: {e}')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED
