from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, FAIL, ABORT

from indoor.states import (
    GoToLandingBase,
    Center,
    Reacquire,
    Land,
)


class PreciseLandingSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        """
        Precise landing state machine.
        """

        self.add_state(
            'GO_TO_LAND',
            GoToLandingBase(),
            transitions={SUCCEED: 'CENTER', TIMEOUT: ABORT},
        )

        self.add_state(
            'CENTER',
            Center(),
            transitions={SUCCEED: 'LAND', FAIL: 'REACQUIRE', TIMEOUT: ABORT},
        )

        self.add_state(
            'REACQUIRE',
            Reacquire(),
            transitions={SUCCEED: 'CENTER', ABORT: ABORT},
        )

        self.add_state(
            'LAND',
            Land(),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT},
        )

        self.set_start_state('GO_TO_LAND')
