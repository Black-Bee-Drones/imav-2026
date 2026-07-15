from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT, ABORT

from indoor.states import (
    GoToWindow,
    FindWindow,
    Window,
    GoOut,
)


class InspectSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, ABORT])
        """
        Inspect a dark room state machine.
        """

        self.add_state(
            'GO_TO_WINDOW',
            GoToWindow(),
            transitions={SUCCEED: 'FIND_WINDOW', TIMEOUT: CANCEL},
        )

        self.add_state(
            'FIND_WINDOW',
            FindWindow(),
            transitions={SUCCEED: 'WINDOW', TIMEOUT: CANCEL},
        )

        self.add_state(
            'WINDOW',
            Window(),
            transitions={SUCCEED: 'GO_OUT', TIMEOUT: ABORT},
        )

        self.add_state(
            'GO_OUT',
            GoOut(),
            transitions={SUCCEED: SUCCEED, TIMEOUT: ABORT},
        )

        self.set_start_state('GO_TO_WINDOW')
