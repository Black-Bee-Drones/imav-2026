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
            'WINDOWS1',
            Window(),
            transitions={SUCCEED: 'RED_BAR', TIMEOUT: TIMEOUT},
        )

        self.add_state(
            'RED_BAR',
            RedBar(),
            transitions={SUCCEED: 'BLUE_BAR1', TIMEOUT: TIMEOUT},
        )

        self.add_state(
            'BLUE_BAR1',
            BlueBar(),
            transitions={SUCCEED: 'BLUE_BAR2', TIMEOUT: TIMEOUT},
        )

        self.add_state(
            'BLUE_BAR2',
            BlueBar(),
            transitions={SUCCEED: 'TUBES', TIMEOUT: TIMEOUT},
        )

        self.add_state(
            'TUBES',
            Tubes(),
            transitions={SUCCEED: 'WINDOWS2', TIMEOUT: TIMEOUT},
        )

        self.add_state(
            'WINDOWS2',
            Window(),
            transitions={SUCCEED: SUCCEED, TIMEOUT: TIMEOUT},
        )

        self.set_start_state('WINDOWS1')
