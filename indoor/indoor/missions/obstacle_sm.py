from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, ABORT

from indoor.states import (
    Window,
    RedBar,
    BlueBar,
    Tubes,
)


class ObstacleSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT, ABORT])
        """
        Obstacle course state machine.
        """

        self.add_state(
            'FIRST_WINDOW',
            Window(),
            transitions={SUCCEED: 'RED_BAR', TIMEOUT: TIMEOUT},
        )

        self.add_state(
            'RED_BAR',
            RedBar(),
            transitions={SUCCEED: 'BLUE_BAR', TIMEOUT: TIMEOUT},
        )

        self.add_state(
            'BLUE_BAR',
            BlueBar(),
            transitions={SUCCEED: 'TUBES', TIMEOUT: TIMEOUT},
        )

        self.add_state(
            'TUBES',
            Tubes(),
            transitions={SUCCEED: 'SECOND_WINDOW', TIMEOUT: TIMEOUT},
        )

        self.add_state(
            'SECOND_WINDOW',
            Window(),
            transitions={SUCCEED: SUCCEED, TIMEOUT: TIMEOUT},
        )

        self.set_start_state('FIRST_WINDOW')
