import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavlinkDrone


class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone = blackboard.get('drone')

        if blackboard.get('rlt'):
            yasmin.YASMIN_LOG_INFO('RTL...')
            status = drone.rtl()

        if not self.rtl or not status:
            yasmin.YASMIN_LOG_INFO('Landing...')
            drone.land()

        return SUCCEED
