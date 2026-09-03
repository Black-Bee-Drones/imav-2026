import yasmin
from yasmin import Blackboard, State
from yasmin_ros.basic_outcomes import ABORT, SUCCEED

from nectar.control import MavlinkDrone
from nectar.control.types import RTLMethod

from ...config import Config


class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        if config.rtl:
            yasmin.YASMIN_LOG_INFO('RTL...')
            if drone.rtl(method=RTLMethod.NAVIGATE):
                return SUCCEED
            yasmin.YASMIN_LOG_WARN('RTL failed')

        yasmin.YASMIN_LOG_INFO('Landing...')
        drone.land()
        return SUCCEED
