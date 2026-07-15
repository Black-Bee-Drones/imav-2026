from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from indoor.states import (
    Window,
    RedBar,
    BlueBar,
    Tubes,
)


class ObstacleSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL])
        """
        Obstacle course state machine.
        """

        self.add_state(
            'WINDOWS1',
            Window(),
            transitions={SUCCEED: 'RED_BAR', TIMEOUT: CANCEL},
        )

        self.add_state(
            'RED_BAR',
            RedBar(),
            transitions={SUCCEED: 'BLUE_BAR1', TIMEOUT: CANCEL},
        )

        self.add_state(
            'BLUE_BAR1',
            BlueBar(),
            transitions={SUCCEED: 'BLUE_BAR2', TIMEOUT: CANCEL},
        )

        self.add_state(
            'BLUE_BAR2',
            BlueBar(),
            transitions={SUCCEED: 'TUBES', TIMEOUT: CANCEL},
        )

        self.add_state(
            'TUBES',
            Tubes(),
            transitions={SUCCEED: 'WINDOWS2', TIMEOUT: CANCEL},
        )

        self.add_state(
            'WINDOWS2',
            Windows(),
            transitions={SUCCEED: SUCCEED, TIMEOUT: CANCEL},
        )

        self.set_start_state('RED_BAR')
