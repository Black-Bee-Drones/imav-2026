from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, CANCEL, ABORT

from indoor.states import (
    GoToObstacles,
    Window,
    ReacquireWindow,
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
            GoToObstacles(),
            transitions={SUCCEED: 'FIRST_WINDOW', TIMEOUT: CANCEL}
        )

        self.add_state(
            'FIRST_WINDOW',
            Window('first'),
            transitions={SUCCEED: 'RED_BAR', TIMEOUT: CANCEL, 'reacquire': 'REACQUIRE_WINDOW'},
        )

        self.add_state(
            'REACQUIRE_WINDOW',
            ReacquireWindow(),
            transitions={SUCCEED: 'FIRST_WINDOW', TIMEOUT: CANCEL},
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
            Window('second'),
            transitions={SUCCEED: SUCCEED, TIMEOUT: CANCEL},
        )

        self.set_start_state('GO_TO_OBSTACLES')
