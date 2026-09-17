import rclpy

import nectar

from traceback import print_exc

from yasmin import StateMachine, Blackboard
from yasmin_ros import set_ros_loggers as yasmin_set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub

from .core import Initialize, Takeoff, ReturnToLaunch, End
from .searchSM import SearchSM
from .packageSM import PackageSM
from .searchSM.constants import configure_coordinates, parse_args

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
                SUCCEED: 'SEARCH',
                ABORT: 'END'
            }
        )

        self.add_state(
            'SEARCH',
            SearchSM(),
            transitions={
                SUCCEED: 'PACKAGE',
                ABORT: 'RETURN_TO_LAUNCH'
            }
        )
        
        self.add_state(
            'PACKAGE',
            PackageSM(),
            transitions={
                SUCCEED: 'RETURN_TO_LAUNCH',
                ABORT: 'RETURN_TO_LAUNCH'
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


def main(args=None) -> None:
    mangalarga_sm = None

    try:
        parsed_args = parse_args(args)
        if parsed_args.latitude is not None and parsed_args.longitude is not None:
            configure_coordinates(parsed_args.latitude, parsed_args.longitude)

        rclpy.init(args=args)
        yasmin_set_ros_loggers()

        executor = YasminNode.get_instance()._executor
        assert executor is not None, 'Executor is not initialized'

        nectar.use_executor(executor)

        mangalarga_sm = ManequimSM()

        mangalarga_blackboard = Blackboard()
        _ = YasminViewerPub(mangalarga_sm, 'MANGALARGA_FSM')

        outcome = mangalarga_sm(blackboard=mangalarga_blackboard)
        print(f"Mangalarga finished with status: {outcome}")

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
    main()