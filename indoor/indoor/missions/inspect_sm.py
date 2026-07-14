from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from states import *


class InspectSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.set_description('State machine for inspecting a dark room.')
