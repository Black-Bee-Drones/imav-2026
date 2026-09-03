from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, FAIL, CANCEL, ABORT

from indoor.states import (
    GoToBox,
    CenterBox,
    Reacquire,
    Drop,
)


class DroppingSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, ABORT, TIMEOUT])
        """
        Dropping on hot spot state machine.
        """

        self.add_state(
            'GO_TO_BOX',
            GoToBox(),
            transitions={SUCCEED: SUCCEED, TIMEOUT: CANCEL, CANCEL: CANCEL},
        )

        self.add_state(
            'CENTER',
            CenterBox(),
            transitions={SUCCEED: 'DROP', FAIL: 'REACQUIRE', TIMEOUT: CANCEL},
        )

        self.add_state(
            'REACQUIRE',
            Reacquire(),
            transitions={SUCCEED: 'CENTER', TIMEOUT: TIMEOUT, CANCEL: CANCEL},
        )

        self.add_state(
            'DROP',
            Drop(),
            transitions={SUCCEED: SUCCEED, CANCEL: CANCEL},
        )

        self.set_start_state('GO_TO_BOX')
