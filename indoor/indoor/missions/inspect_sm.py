from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, CANCEL

from indoor.states import (
    GoToWindow,
    FindWindow,
    Window,
    CountBabies,
    GoOut,
)


class InspectSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL])

        self.add_state(
            'GO_TO_WINDOW',
            GoToWindow(),
            transitions={
                SUCCEED: 'FIND_WINDOW',
                CANCEL: CANCEL,
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            'FIND_WINDOW',
            FindWindow(),
            transitions={
                SUCCEED: 'WINDOW',
                CANCEL: CANCEL,
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            'WINDOW',
            Window('room'),
            transitions={
                SUCCEED: 'COUNT_BABIES',
                TIMEOUT: CANCEL,
                'reacquire': 'FIND_WINDOW'
            },
        )

        self.add_state(
            'COUNT_BABIES',
            CountBabies(),
            transitions={
                SUCCEED: 'GO_OUT'
            },
        )

        self.add_state(
            'GO_OUT',
            GoOut(),
            transitions={
                SUCCEED: SUCCEED,
                TIMEOUT: CANCEL,
            },
        )

        self.set_start_state('GO_TO_WINDOW')
