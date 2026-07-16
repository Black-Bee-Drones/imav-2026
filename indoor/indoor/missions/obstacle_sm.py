from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, CANCEL, ABORT

from indoor.states import (
    Window,
    RedBar,
    BlueBar,
    Tubes,
)


class ObstacleSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL, ABORT])
        """
        Obstacle course state machine.
        """

        self.add_state(
            'FIRST_WINDOW',
            Window(),
            transitions={SUCCEED: 'RED_BAR', TIMEOUT: CANCEL},
        )

        self.add_state(
            'RED_BAR',
            RedBar(),
            transitions={SUCCEED: 'BLUE_BAR', TIMEOUT: CANCEL},
        )

        self.add_state(
            'BLUE_BAR',
            BlueBar(),
            transitions={SUCCEED: 'TUBES', TIMEOUT: CANCEL},
        )

        self.add_state(
            'TUBES',
            Tubes(),
            transitions={SUCCEED: 'SECOND_WINDOW', TIMEOUT: CANCEL},
        )

        self.add_state(
            'SECOND_WINDOW',
            Window(),
            transitions={SUCCEED: SUCCEED, TIMEOUT: CANCEL},
        )

        self.set_start_state('FIRST_WINDOW')
