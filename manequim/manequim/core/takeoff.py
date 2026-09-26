from yasmin import State, Blackboard, YASMIN_LOG_INFO, YASMIN_LOG_ERROR
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone, MavlinkDrone

from manequim.core.constants import TAKEOFF_HEIGHT

from traceback import print_exc

class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED,ABORT])

        self.drone: MavrosDrone | MavlinkDrone

    def execute(self, blackboard: Blackboard):
        self.drone = blackboard["drone"]
        altitude = self.drone.get_altitude()

        try:
            YASMIN_LOG_INFO(f"Taking off to {TAKEOFF_HEIGHT}m...")
            ok = self.drone.takeoff(altitude=TAKEOFF_HEIGHT, max_retries=5, adjust_altitude=False)
            self.drone.delay(3.0)
            if altitude is not None:
                self.drone.arm()
                self.drone.delay(5)
                ok = self.drone.move_to(x=0.0,y=0.0,z=(2.0 - altitude))

            YASMIN_LOG_INFO("Takeoff complete.")
            return SUCCEED if ok else ABORT

        except Exception as e:
            YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            print_exc()
            return ABORT
