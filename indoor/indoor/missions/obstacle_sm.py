from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from indoor.states import (
    GoToObstacles,
    Window,
    ReacquireWindow,
    ToBarCorridor,
    FindCenterDescendBars,
    PassBlue,
    Tubes,
)


class ObstacleSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL])

        self.add_state(
            "GO_TO_OBSTACLES",
            GoToObstacles(),
            transitions={
                SUCCEED: "FIRST_WINDOW",
                CANCEL: CANCEL,
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            "FIRST_WINDOW",
            Window("first"),
            transitions={
                SUCCEED: "TO_BAR_CORRIDOR",
                "skip": "TO_BAR_CORRIDOR",
                TIMEOUT: CANCEL,
                "reacquire": "REACQUIRE_FIRST_WINDOW",
            },
        )

        self.add_state(
            "REACQUIRE_FIRST_WINDOW",
            ReacquireWindow("first"),
            transitions={
                SUCCEED: "FIRST_WINDOW",
                CANCEL: "TO_BAR_CORRIDOR",
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            "TO_BAR_CORRIDOR",
            ToBarCorridor(),
            transitions={
                SUCCEED: "FIND_CENTER_DESCEND_BARS",
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            "FIND_CENTER_DESCEND_BARS",
            FindCenterDescendBars(),
            transitions={
                SUCCEED: "PASS_BLUE",
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            "PASS_BLUE",
            PassBlue(),
            transitions={
                SUCCEED: "TUBES",
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            "TUBES",
            Tubes(),
            transitions={
                SUCCEED: "SECOND_WINDOW",
                TIMEOUT: CANCEL,
            },
        )

        self.add_state(
            "SECOND_WINDOW",
            Window("second"),
            transitions={
                SUCCEED: SUCCEED,
                "skip": SUCCEED,
                TIMEOUT: CANCEL,
                "reacquire": "REACQUIRE_SECOND_WINDOW",
            },
        )

        self.add_state(
            "REACQUIRE_SECOND_WINDOW",
            ReacquireWindow("second"),
            transitions={
                SUCCEED: "SECOND_WINDOW",
                CANCEL: SUCCEED,
                TIMEOUT: CANCEL,
            },
        )

        self.set_start_state("GO_TO_OBSTACLES")
