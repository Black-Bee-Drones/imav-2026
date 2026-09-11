from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from navigation import (
    InitPosition,
    Ascend,
    SearchNavigation,
)

from constants import *

class SearchSM(StateMachine):
    def __init__(self, node: YasminNode):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INIT_POSITION",
            InitPosition(),
            outcomes=[SUCCEED, ABORT],
            transitions={
                SUCCEED: "ASCEND",
                ABORT:   ABORT,
            },
        )

        self.add_state(
            "ASCEND",
            Ascend(),
            outcomes=[SUCCEED, ABORT, SQUARE_SEARCH, MANEQUIM_FOUND],
            transitions={
                SQUARE_SEARCH: "SEARCH_NAVIGATION",
                MANEQUIM_FOUND: SUCCEED,
                ABORT:          ABORT,
            },
        )

        self.add_state(
            "SEARCH_NAVIGATION",
            SearchNavigation(),
            outcomes=[SUCCEED, ABORT, MANEQUIM_FOUND],
            transitions={
                MANEQUIM_FOUND: SUCCEED,
                ABORT:          ABORT,
            },
        )