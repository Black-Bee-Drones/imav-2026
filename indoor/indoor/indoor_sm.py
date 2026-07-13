from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, ABORT

from .states import (
    Initialize,
    Land,
    Takeoff,
)
from .missions import (
    DroppingSM,
    InspectSM,
    ObstacleSM,
    PreciseLandingSM,
)


class IndoorSM(StateMachine):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.set_name('Indoor Mission')
        self.set_description('Main state machine for the IMAV 2026 Indoor Competition.')

        self.add_state(
            'INITIALIZE',
            Initialize(),
            transitions = {SUCCEED: 'TAKEOFF', ABORT: ABORT},
        )

        self.add_state(
            'TAKEOFF',
            Takeoff(),
            transitions = {SUCCEED: 'OBSTACLESM', ABORT: 'LAND'},
        )

        self.add_state(
            'OBSTACLESM',
            ObstacleSM(),
            transitions = {SUCCEED: 'INSPECTSM', TIMEOUT: 'LAND'},
        )

        self.add_state(
            'INSPECTSM',
            InspectSM(),
            transitions = {SUCCEED: 'DROPPINGSM', ABORT: 'LAND'},
        )

        self.add_state(
            'DROPPINGSM',
            DroppingSM(),
            transitions = {SUCCEED: 'PRECISELANDINGSM', ABORT: 'LAND'},
        )

        self.add_state(
            'PRECISELANDINGSM',
            PreciseLandingSM(),
            transitions = {SUCCEED: 'LAND', ABORT: 'LAND'},
        )

        self.add_state(
            'LAND',
            Land(),
            transitions = {SUCCEED: SUCCEED, ABORT: ABORT},
        )

        self.set_start_state('INITIALIZE')
