from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT, ABORT

from states import (
    GoToWindown,
    FindWindown,
    Windows,
    GoOut,
)


class InspectSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, ABORT])
        self.set_name('Inspect a dark room state machine')
        self.set_description('State machine for inspecting a dark room.')

        self.add_state(
            'GO_TO_WINDOWN',
            GoToWindown(),
            transitions={SUCCEED: 'FIND_WINDOWN', TIMEOUT: CANCEL},
        )

        self.add_state(
            'FIND_WINDOWN',
            FindWindown(),
            transitions={SUCCEED: 'WINDOWS', TIMEOUT: CANCEL},
        )

        self.add_state(
            'WINDOWS',
            Windows(),
            transitions={SUCCEED: 'GO_OUT', TIMEOUT: ABORT},
        )

        self.add_state(
            'GO_OUT',
            GoOut(),
            transitions={SUCCEED: SUCCEED, TIMEOUT: ABORT},
        )

        self.set_start_state('GO_TO_WINDOWN')
