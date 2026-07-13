from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from states import *


class DroppingSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.set_description('State machine for dropping the cone onto hot spot.')
