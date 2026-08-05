from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, FAIL, CANCEL, ABORT

from indoor.states import (
    GoToLandingBase,
    CenterFixed,
    CenterMoving,
    Reacquire,
)
from indoor import Config


class PreciseLandingSM(StateMachine):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, CANCEL, ABORT, TIMEOUT])
        """
        Precise landing state machine.
        """

        self.add_state(
            'GO_TO_LAND',
            GoToLandingBase(config),
            transitions={SUCCEED: 'CENTER', TIMEOUT: ABORT},
        )

        self.add_state(
            'CENTER',
            CenterFixed(
                config) if config.landing_mode == "PRECISION_FIXED" else CenterMoving(config),
            transitions={SUCCEED: SUCCEED, FAIL: 'REACQUIRE', TIMEOUT: ABORT},
        )

        self.add_state(
            'REACQUIRE',
            Reacquire(config),
            transitions={SUCCEED: 'CENTER', CANCEL: ABORT},
        )

        self.set_start_state('GO_TO_LAND')
