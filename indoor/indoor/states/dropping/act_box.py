import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, CANCEL, TIMEOUT

import traceback

from nectar.control import MavlinkDrone, MavrosDrone

from indoor.config import Config

import requests

def led(
        *,
        on:bool=True,
        ip:str='wled.local',
        brightness:int=255,
        r:int=0,
        g:int=0,
        b:int=0,
        w:int=0
    ):

    requests.post(
        f'http://{ip}/json/state',
        json={
            'on': on,
            'seg': [{
                'fx': 0,
                'bri': brightness,
                'col': [[r, g, b, w]],
            }]
        }
    )

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

def blink_led(drone, config: Config) -> bool:
    try:
        for i in range(3):
            led(r=255, brightness=255)
            drone.delay(0.5)
            led(r=0, brightness=0)
            if i != 2:
                drone.delay(0.5)
        return True

    except Exception as e:
        yasmin.YASMIN_LOG_ERROR(f'Error while blink_led function: {e}')
        traceback.print_exc()
        return False


class ActBoxes(State):
    def __init__(self, action: str):
        super().__init__(outcomes=[SUCCEED, FAIL, TIMEOUT])
        self.action = action

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone | MavrosDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')
        action = do_gripper if self.action == 'drop' else blink_led

        try:
            #drone.move_to(z=-0.2)
            drone.move_velocity(0.0, 0.0, 0.0)
            if not action(drone, config):
                return FAIL
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN("Execution interrupted by user.")
            return FAIL
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"{'Gripper' if self.action == 'drop' else 'Led'} failed: {e}")
            return FAIL

        yasmin.YASMIN_LOG_INFO("Completed successfully.")
        return SUCCEED
