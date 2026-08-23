from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from indoor.states import (
    GoToObstacles,
    Window,
    ReacquireWindow,
    RedBar,
    BlueBar,
    Tubes,
)


class ObstacleSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL])

        self.add_state(
            'GO_TO_OBSTACLES',
            GoToObstacles(),
            transitions={
                SUCCEED: 'FIRST_WINDOW',
                CANCEL: CANCEL,
                TIMEOUT: CANCEL,
            }
        )

        self.add_state(
            'FIRST_WINDOW',
            Window('first'),
            transitions={
                SUCCEED: 'RED_BAR',
                TIMEOUT: CANCEL,
                'reacquire': 'REACQUIRE_FIRST_WINDOW',
            },
        )

        self.add_state(
            'REACQUIRE_FIRST_WINDOW',
            ReacquireWindow('first'),
            transitions={
                SUCCEED: 'FIRST_WINDOW',
                CANCEL: 'FIRST_WINDOW',
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            'RED_BAR',
            RedBar(),
            transitions={
                SUCCEED: 'BLUE_BAR',
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            'BLUE_BAR',
            BlueBar(),
            transitions={
                SUCCEED: 'TUBES',
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            'TUBES',
            Tubes(),
            transitions={
                SUCCEED: 'SECOND_WINDOW',
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            'SECOND_WINDOW',
            Window('second'),
            transitions={
                SUCCEED: SUCCEED,
                TIMEOUT: CANCEL,
                'reacquire': 'REACQUIRE_SECOND_WINDOW',
            },
        )

        self.add_state(
            'REACQUIRE_SECOND_WINDOW',
            ReacquireWindow('second'),
            transitions={
                SUCCEED: 'SECOND_WINDOW',
                CANCEL: 'SECOND_WINDOW',
                TIMEOUT: CANCEL,
            },
        )

        self.set_start_state('GO_TO_OBSTACLES')
