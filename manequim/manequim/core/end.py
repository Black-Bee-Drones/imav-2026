from yasmin import State, Blackboard, YASMIN_LOG_INFO
from yasmin_ros.basic_outcomes import SUCCEED, ABORT


class End(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        _ = blackboard
        YASMIN_LOG_INFO("Manequim mission finished.")
        return SUCCEED
