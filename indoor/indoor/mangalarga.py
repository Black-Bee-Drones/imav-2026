import argparse
import traceback

import nectar
import rclpy
import yasmin
from yasmin import Blackboard
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin_ros.yasmin_node import YasminNode

from indoor import Config, IndoorSM


def main(args=None):
    parser = argparse.ArgumentParser(
        prog='ros2 run indoor mangalarga',
        description='IMAV 2026 - Indoor Drone State Machine',
        add_help=True,
    )
    parser.add_argument('--sitl', action='store_true')
    parser.add_argument(
        '--preset',
        type=str,
        default=None,
        choices=Config.list_preset(),
    )
    parser.add_argument('-o-skip', '--obstacle-skip', action='store_true')
    parser.add_argument('-i-skip', '--inspect-skip', action='store_true')
    parser.add_argument('-d-skip', '--droping-skip', action='store_true')
    parser.add_argument('-p-skip', '--precise-skip', action='store_true')
    parser.add_argument('--no-takeoff', action='store_true')
    parsed_args, _ = parser.parse_known_args()

    rclpy.init(args=args)
    set_ros_loggers()
    executor = YasminNode.get_instance()._executor
    if executor is not None:
        nectar.use_executor(executor)

    indoor_sm = None
    try:
        config = Config().apply_args(parsed_args)
        indoor_sm = IndoorSM(config)
        final_outcome = indoor_sm(Blackboard())
    except KeyboardInterrupt:
        yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
        if indoor_sm is not None and indoor_sm.is_running():
            indoor_sm.cancel_state()
    except Exception as e:
        yasmin.YASMIN_LOG_ERROR(f'Indoor state machine failed: {e}')
        yasmin.YASMIN_LOG_ERROR(traceback.format_exc())
    else:
        if final_outcome == SUCCEED:
            yasmin.YASMIN_LOG_INFO(final_outcome)
        else:
            yasmin.YASMIN_LOG_ERROR(final_outcome)

    nectar.shutdown()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
