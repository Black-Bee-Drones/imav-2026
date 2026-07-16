import rclpy

import yasmin
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED

import nectar

from indoor import (
    Config,
    IndoorSM,
)


def main(args=None):
    rclpy.init(args=args)

    set_ros_loggers()

    nectar.use_executor(YasminNode.get_instance()._executor)

    try:
        config = Config()

        indoor_sm = IndoorSM(config)

        final_outcome = indoor_sm()

    except KeyboardInterrupt:
        if indoor_sm.is_running():
            indoor_sm.cancel_state()

    except Exception as e:
        yasmin.YASMIN_LOG_ERROR(f'Indoor state machine failed: {e}')

    except rclpy.exceptions.ParameterUninitializedException as e:
        yasmin.YASMIN_LOG_ERROR("Required parameter is not configured!")
        yasmin.YASMIN_LOG_ERROR(f"Details: {e}")
        yasmin.YASMIN_LOG_ERROR("Check the YAML parameter file.")

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
