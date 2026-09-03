import yasmin
from yasmin import Blackboard, State
from yasmin_ros.basic_outcomes import ABORT, SUCCEED
from yasmin_ros.yasmin_node import YasminNode

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

        if config.skip_takeoff:
            yasmin.YASMIN_LOG_INFO(
                'Skip takeoff: GUIDED + origin, not arming.'
            )
            if not drone.set_mode('GUIDED'):
                yasmin.YASMIN_LOG_WARN('Failed to set GUIDED.')
            else:
                drone.delay(1.0)
            try:
                drone.set_takeoff_position()
            except Exception as e:
                yasmin.YASMIN_LOG_WARN(f'Takeoff position not set: {e}')
            return SUCCEED

        takeoff_alt = config.takeoff_alt
        yasmin.YASMIN_LOG_INFO(f'Taking off (altitude={takeoff_alt} m)...')
        if drone.takeoff(takeoff_alt, max_retries=5):
            yasmin.YASMIN_LOG_INFO('Takeoff successful!')
            return SUCCEED

        yasmin.YASMIN_LOG_ERROR('Takeoff failed!')
        return ABORT
