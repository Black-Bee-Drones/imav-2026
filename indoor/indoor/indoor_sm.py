from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT, ABORT

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


class IndoorSM(StateMachine):
    def __init__(self, missions: tuple[str] = ('OBSTACLESM', 'INSPECTSM', 'DROPPINGSM', 'PRECISELANDINGSM')):
        super().__init__(outcomes=[SUCCEED, ABORT])
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
            Initialize(),
            transitions = {SUCCEED: 'TAKEOFF', ABORT: ABORT},
        )

        self.add_state(
            'TAKEOFF',
            Takeoff(),
            transitions = {SUCCEED: 'OBSTACLESM', ABORT: 'LAND'},
        )

        for i, mission in enumerate(missions):
            self.add_state(
                mission,
                missions_sm.get(mission),
                transitions = {
                    SUCCEED: missions[i + 1] if i < len(missions)-1 else 'LAND',
                    TIMEOUT: 'PRECISELANDINGSM',
                    ABORT: 'LAND'
                },
            )

        self.add_state(
            'LAND',
            Land(),
            transitions = {SUCCEED: SUCCEED, ABORT: ABORT},
        )

        self.set_start_state('INITIALIZE')
