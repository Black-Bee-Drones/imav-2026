from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, ABORT

from indoor.states import (
    Reacquire,
)


class DroppingSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT, ABORT])
        """
        Dropping on hot spot state machine.
        """

        self.add_state(
            'REACQUIRE',
            Reacquire(),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT},
        )

        self.set_start_state('REACQUIRE')
