import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, CANCEL

from nectar.control import MavlinkDrone, MavrosDrone

from indoor.config import Config

def do_gripper(drone, config: Config) -> bool:
    pwm = config.dropping_servo_open_pwm
    retries = max(1, int(config.dropping_servo_retries))
    for attempt in range(1, retries + 1):
        yasmin.YASMIN_LOG_INFO(f"Set {pwm} pwm (attempt {attempt}/{retries})")
        if drone.do_servo(aux_out=config.dropping_servo_channel, pwm_value=pwm):
            drone.delay(config.dropping_servo_action_delay)
            return True
        yasmin.YASMIN_LOG_WARN(f"do_servo failed (attempt {attempt}/{retries})")
        if attempt < retries:
            drone.delay(config.dropping_servo_retry_delay)
    yasmin.YASMIN_LOG_ERROR("Failed to do servo.")
    return False

class Drop(State):
    def __init__(self, blackboard: Blackboard):
        super().__init__(outcomes=[SUCCEED, FAIL])

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone | MavrosDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')

        try:
            #drone.move_to(z=-0.2)
            drone.move_velocity(0.0, 0.0, 0.0)
            if not do_gripper(drone, config):
                return FAIL
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN("Execution interrupted by user.")
            return FAIL
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Gripper failed: {e}")
            return FAIL

        yasmin.YASMIN_LOG_INFO("Completed successfully.")
        return SUCCEED
