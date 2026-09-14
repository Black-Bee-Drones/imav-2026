from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from .states import (
    AlignState,
    DescendState,
    ReestablishState,
    DropState,
)
from .constants import (
    LOST_PERSON,
    ALIGNMENT_FAILED,
    DROP_RETRY,
)


class PackageSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, LOST_PERSON, ALIGNMENT_FAILED, DROP_RETRY])

        self.add_state(
            "DESCEND",
            DescendState(),
            transitions={
                "ALIGN": "ALIGN",
                SUCCEED: "DROP",
                ABORT: "REESTABLISH",
                ALIGNMENT_FAILED: ABORT,
            },
        )

        self.add_state(
            "ALIGN",
            AlignState(),
            transitions={
                SUCCEED: "DESCEND",
                LOST_PERSON: "REESTABLISH",
                ALIGNMENT_FAILED: ABORT,
                ABORT: "REESTABLISH",
            },
        )

        self.add_state(
            "REESTABLISH",
            ReestablishState(),
            transitions={
                SUCCEED: "ALIGN",
                LOST_PERSON: ABORT,
                ABORT: ABORT,
            },
        )

        self.add_state(
            "DROP",
            DropState(),
            transitions={
                SUCCEED: SUCCEED,
                DROP_RETRY: "DROP",
                ABORT: "REESTABLISH",
            },
        )

        self.set_start_state("DESCEND")
