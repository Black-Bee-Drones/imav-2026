from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from .states import (
    InitPosition,
    Ascend,
    SearchNavigation,
)
from .constants import *


class SearchSM(StateMachine):
    def __init__(self, node: YasminNode | None = None):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INIT_POSITION",
            InitPosition(),
            transitions={
                SUCCEED: "ASCEND",
                ABORT: ABORT,
            },
        )

        self.add_state(
            "ASCEND",
            Ascend(),
            transitions={
                SQUARE_SEARCH: "SEARCH_NAVIGATION",
                MANEQUIM_FOUND: SUCCEED,
                ABORT: ABORT,
            },
        )

        self.add_state(
            "SEARCH_NAVIGATION",
            SearchNavigation(),
            transitions={
                SUCCEED: SUCCEED,
                MANEQUIM_FOUND: SUCCEED,
                ABORT: ABORT,
            },
        )

        self.set_start_state("ASCEND")