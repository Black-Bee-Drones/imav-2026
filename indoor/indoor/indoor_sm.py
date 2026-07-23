from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT

from indoor.states import (
    Initialize,
    Land,
    Takeoff,
)
from indoor.missions import (
    DroppingSM,
    InspectSM,
    ObstacleSM,
    PreciseLandingSM,
)
from indoor import Config


class IndoorSM(StateMachine):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL, TIMEOUT])
        """
        Indoor Mission

        Main state machine for the IMAV 2026 Indoor Competition
        """

        missions_sm = {
            'OBSTACLESM': ObstacleSM,
            'INSPECTSM': InspectSM,
            'DROPPINGSM': DroppingSM,
            'PRECISELANDINGSM': PreciseLandingSM,
        }

        self.add_state(
            'INITIALIZE',
            Initialize(config),
            transitions={SUCCEED: 'TAKEOFF', ABORT: ABORT},
        )

        self.add_state(
            'TAKEOFF',
            Takeoff(config),
            transitions={SUCCEED: 'OBSTACLESM', ABORT: 'LAND'},
        )

        for i, mission in enumerate(config.missions):
            self.add_state(
                mission,
                missions_sm.get(mission)(config),
                transitions={
                    SUCCEED: config.missions[i + 1] if i < len(config.missions)-1 else 'LAND',
                    CANCEL: 'PRECISELANDINGSM',
                    ABORT: 'LAND',
                    TIMEOUT: ABORT,
                },
            )

        self.add_state(
            'LAND',
            Land(config),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT},
        )

        self.set_start_state('INITIALIZE')
