from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, CANCEL, ABORT

from indoor.states import (
    GoToWindow,
    FindWindow,
    Window,
    CountBabies,
    GoOut,
)
from indoor import Config


class InspectSM(StateMachine):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, CANCEL, ABORT, TIMEOUT])
        """
        Inspect a dark room state machine.
        """

        self.add_state(
            'GO_TO_WINDOW',
            GoToWindow(config),
            transitions={SUCCEED: 'FIND_WINDOW', TIMEOUT: CANCEL},
        )

        self.add_state(
            'FIND_WINDOW',
            FindWindow(config),
            transitions={SUCCEED: 'WINDOW', TIMEOUT: CANCEL},
        )

        self.add_state(
            'WINDOW',
            Window(config, 'room'),
            transitions={SUCCEED: 'COUNT_BABIES', TIMEOUT: CANCEL},
        )

        self.add_state(
            'COUNT_BABIES',
            CountBabies(config),
            transitions={SUCCEED: 'GO_OUT'},
        )

        self.add_state(
            'GO_OUT',
            GoOut(config),
            transitions={SUCCEED: SUCCEED, TIMEOUT: CANCEL},
        )

        self.set_start_state('GO_TO_WINDOW')
