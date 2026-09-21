from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference

from ...config import Config
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

class GoOut(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()


    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        self.timeout: int = config.timeout
        self.mission_timeout: int = config.inspect_timeout

        self.start_time: Time = blackboard.get('start_time')
        self.start_mission: Time = config.inspect_start_time

        drone: MavlinkDrone = blackboard.get('drone')

        if self.check_timeout():
            config.obstacle_skip = True
            config.inspect_skip = True
            config.droping_skip = True
            config.precise_skip = True
            config.rtl = False
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO(f'Correcting altitude to {config.inspect_gate_alignment_alt}')

        go_out_dist = config.inspect_gate_standoff + config.inspect_gate_commit_extra
        yasmin.YASMIN_LOG_INFO(f'Fly through the window {go_out_dist}m.')

        drone.move_to(
            x=go_out_dist,
            y=0,
            z=0,
            yaw=0,
            reference=MoveReference.BODY
        )

        if self.check_timeout():
            return TIMEOUT

        yasmin.YASMIN_LOG_INFO("Turning off LED...")
        led(on=False)

        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(seconds=self.timeout) or \
            now - self.start_mission > Duration(seconds=self.mission_timeout)
