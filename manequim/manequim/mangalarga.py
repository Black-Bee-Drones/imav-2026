import rclpy

import nectar

from traceback import print_exc

from yasmin import StateMachine, Blackboard
from yasmin_ros import set_ros_loggers as yasmin_set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub

from core.states import Initialize, Takeoff, ReturnToLaunch, End

class ManequimSM(StateMachine):
    def __init__(self) -> None:
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            'INITIALIZE',
            Initialize(),
            transitions={
                SUCCEED: 'TAKEOFF',
                ABORT: 'END'
            }
        )

        self.add_state(
            'TAKEOFF',
            Takeoff(),
            transitions={
                SUCCEED: 'RETURN_TO_LAUNCH',
                ABORT: 'END'
            }
        )

        self.add_state(
            'RETURN_TO_LAUNCH',
            ReturnToLaunch(),
            transitions={
                SUCCEED: 'END',
            }
        )
        self.add_state(
            'END',
            End(),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: ABORT
            }
        )

        self.set_start_state('INITIALIZE')


def mangalarga() -> None:

    try:
        rclpy.init()
        yasmin_set_ros_loggers()

        executor = YasminNode.get_instance()._executor
        assert executor is not None, 'Executor is not initialized'

        nectar.use_executor(executor)

        mangalarga_sm = ManequimSM()

        # Initialize a fresh Blackboard specifically for the mangalarga sequence
        mangalarga_blackboard = Blackboard()

        # Keep the viewer alive, even though the object is never called
        _ = YasminViewerPub(mangalarga_sm, 'MANGALARGA_FSM')

        outcome = mangalarga_sm(blackboard=mangalarga_blackboard)
        print (f"Mangalarga finished with status: {outcome}")

    except KeyboardInterrupt:
        print('Stopping by keyboard interrupt...')
        if mangalarga_sm is not None:
            mangalarga_sm.cancel_state()

    except Exception as e:
        print(f'Mangalarga finished with exception: {e}')
        print_exc()

    finally:

        if nectar.is_initialized():
            nectar.shutdown()

        if rclpy.ok():
            rclpy.shutdown()

        return

if __name__ == "__main__":
    mangalarga()