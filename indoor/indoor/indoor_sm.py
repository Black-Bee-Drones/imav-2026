from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT

from indoor.states import (
    Initialize,
    Land,
    Takeoff,
)
from indoor.missions import (
    ObstacleSM,
    InspectSM,
    DroppingSM,
    PreciseLandingSM,
)

from indoor.config import Config


class IndoorSM(StateMachine):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL])

        self.add_state(
            'INITIALIZE',
            Initialize(config),
            transitions={SUCCEED: 'TAKEOFF', ABORT: ABORT},
        )

        self.add_state(
            'TAKEOFF',
            Takeoff(),
            transitions={SUCCEED: 'PRECISION_LANDING', ABORT: ABORT},
        )

        self.add_state(
            'OBSTACLESM',
            ObstacleSM(),
            transitions={SUCCEED: 'INSPECTSM', CANCEL: 'INSPECTSM'},
        )

        self.add_state(
            'INSPECTSM',
            InspectSM(),
            transitions={SUCCEED: 'DROPPINGSM', CANCEL: 'DROPPINGSM'},
        )

        self.add_state(
            'DROPPINGSM',
            DroppingSM(),
            transitions={SUCCEED: 'PRECISION_LANDING', CANCEL: 'PRECISION_LANDING'},
        )

        self.add_state(
            'PRECISION_LANDING',
            PreciseLandingSM(config),
            transitions={SUCCEED: 'LAND', CANCEL: 'LAND'},
        )

        self.add_state(
            'LAND',
            Land(),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT},
        )

        self.set_start_state('INITIALIZE')
