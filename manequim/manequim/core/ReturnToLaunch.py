from yasmin import State, Blackboard, YASMIN_LOG_INFO, YASMIN_LOG_ERROR
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone, MavlinkDrone, RTLMethod

from manequim.core.constants import RTL_ALTITUDE

class ReturnToLaunch(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.drone: MavrosDrone | MavlinkDrone

    def execute(self, blackboard: Blackboard):

        self.drone = blackboard["drone"]

        try:

            YASMIN_LOG_INFO(f"Returning to launch at {RTL_ALTITUDE}m...")
            self.drone.rtl(
                altitude=RTL_ALTITUDE,
                method=RTLMethod.NAVIGATE,
                land=False,
            )
            self.drone.delay(2)
            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT