from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

class PackageSM(StateMachine):
    def __init__(self, node: YasminNode):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "ALIGN",
            ALIGN(),
            outcomes=[SUCCEED, ABORT, ALIGNED, READY],
            transitions={
                ALIGNED: "DESCEND",
                READY: "DROP",
                LOST: "REESTABLISH",
                ABORT:   ABORT,
            },
        )

        self.add_state(
            "DESCEND",
            DESCEND(),
            outcomes=[SUCCEED, ABORT],
            transitions={
                SUCCEED: "ALIGN",
                ABORT:   ABORT,
            },
        )

        self.add_state(
            "REESTABLISH",
            REESTABLISH(),
            outcomes=[SUCCEED, ABORT],
            transitions={
                SUCCEED: "ALIGN",
                ABORT:   ABORT,
            },
        )

        self.add_state(
            "DROP",
            DROP(),
            outcomes=[SUCCEED, ABORT],
            transitions={
                SUCCEED: SUCCEED,
                ABORT:   ABORT,
            },
        )
