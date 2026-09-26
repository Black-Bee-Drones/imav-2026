from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, FAIL, CANCEL

from indoor.states import (
    GoToBox,
    MapBoxes,
    CenterBox,
    ActBoxes,
)

class DroppingSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, CANCEL])
        """
        Dropping on hot spot state machine.
        """

        self.add_state(
            'GO_TO_BOX',
            GoToBox(),
            transitions={SUCCEED: 'MAP_BOXES', TIMEOUT: CANCEL, CANCEL: CANCEL},
        )

        self.add_state(
            'MAP_BOXES',
            MapBoxes(),
            transitions={SUCCEED: 'CENTER_DROP_BOX', TIMEOUT: CANCEL, CANCEL: CANCEL}
        )

        self.add_state(
            'CENTER_DROP_BOX',
            CenterBox('drop'),
            transitions={SUCCEED: 'DROP', FAIL: CANCEL, TIMEOUT: CANCEL},
        )

        self.add_state(
            'DROP',
            ActBoxes('drop'),
            transitions={SUCCEED: "CENTER_LED_BOX" , FAIL: CANCEL},
        )

        self.add_state(
            'CENTER_LED_BOX',
            CenterBox('led'),
            transitions={SUCCEED: 'BLINK_LED', FAIL: CANCEL, TIMEOUT: CANCEL},
        )

        self.add_state(
            'BLINK_LED',
            ActBoxes('led'),
            transitions={SUCCEED: SUCCEED, FAIL: CANCEL, TIMEOUT: CANCEL}
        )

        self.set_start_state('GO_TO_BOX')
