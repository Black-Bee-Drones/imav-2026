import sys
import argparse
import rclpy

import yasmin
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED

import nectar

from indoor import (
    Config,
    STILConfig,
    CLEITINHO,
    STIL_CLEITINHO,
    JORGE,
    STIL_JORGE,
    IndoorSM,
)

CONFIG_PROFILES = {
    'default': Config,
    'stil': STILConfig,
    'cleitinho': CLEITINHO,
    'stil_cleitinho': STIL_CLEITINHO,
    'jorge': JORGE,
    'stil_jorge': STIL_JORGE,
}


def main(args=None):
    parser = argparse.ArgumentParser(description='Indoor Drone State Machine')
    parser.add_argument(
        '--config',
        type=str,
        default='default',
        choices=CONFIG_PROFILES.keys(),
        help='Which drone configuration profile to load'
    )

    input_args = args if args is not None else sys.argv[1:]
    parsed_args, remaining_args = parser.parse_known_args(input_args)

    rclpy.init(args=sys.argv)
    set_ros_loggers()

    nectar.use_executor(YasminNode.get_instance()._executor)

    try:
        yasmin.YASMIN_LOG_INFO(
            f"Loading configuration profile: {parsed_args.config.upper()}")
        config_class = CONFIG_PROFILES[parsed_args.config]
        config = config_class()

        indoor_sm = IndoorSM(config)
        final_outcome = indoor_sm()

    except KeyboardInterrupt:
        if indoor_sm.is_running():
            indoor_sm.cancel_state()

    except Exception as e:
        yasmin.YASMIN_LOG_ERROR(f'Indoor state machine failed: {e}')

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
