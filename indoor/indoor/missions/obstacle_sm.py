from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, CANCEL, ABORT

from indoor.states import (
    GoToObstacles,
    Window,
    RedBar,
    BlueBar,
    Tubes,
)
from indoor import Config


class ObstacleSM(StateMachine):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, CANCEL, ABORT, TIMEOUT])
        """
        Obstacle course state machine.
        """
        self.add_state(
            'GO_TO_OBSTACLES',
            GoToObstacles(config),
            transitions={SUCCEED: 'FIRST_WINDOW', TIMEOUT: CANCEL}
        )

        self.add_state(
            'FIRST_WINDOW',
            Window(config, 'first'),
            transitions={SUCCEED: 'RED_BAR', TIMEOUT: CANCEL},
        )

        self.add_state(
            'RED_BAR',
            RedBar(config),
            transitions={SUCCEED: 'BLUE_BAR', TIMEOUT: CANCEL},
        )

        self.add_state(
            'BLUE_BAR',
            BlueBar(config),
            transitions={SUCCEED: 'TUBES', TIMEOUT: CANCEL},
        )

        self.add_state(
            'TUBES',
            Tubes(config),
            transitions={SUCCEED: 'SECOND_WINDOW', TIMEOUT: CANCEL},
        )

        self.add_state(
            'SECOND_WINDOW',
            Window(config, 'second'),
            transitions={SUCCEED: SUCCEED, TIMEOUT: CANCEL},
        )

        self.set_start_state('GO_TO_OBSTACLES')
