from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, TIMEOUT

from nectar.control import MavlinkDrone, PIDController
from nectar.vision import ImageHandler
from nectar.ai import DetectionResult

from ...config import Config

_SWEEP_SEC_BASE = 4.0
_SWEEP_VX = 0.12
_SWEEP_VY = 0.12
_CONFIRM = 5

class ReacquireWindow(State):
    def __init__(self, position):
        super().__init__(outcomes=[SUCCEED, CANCEL, TIMEOUT])

        self.position = position
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        self.timeout: int = config.timeout
        self.mission_timeout: int = config.obstacle_timeout

        self.start_time: Time = blackboard.get("start_time")
        self.start_mission: Time = config.obstacle_start_time

        drone: MavlinkDrone = blackboard.get("drone")

        handler: ImageHandler = blackboard.get("image_handler_north")
        handler.image_processing_callback = blackboard.get("callback_gate")

        class_name = config.model_gate_class_name
        pid_z = PIDController(
            kp=config.obstacle_alt_kp,
            kd=0.0,
            ki=0.0,
            setpoint=config.obstacle_gate_alt,
            output_limits=(-0.2, 0.2),
        )


        if self.position == "room":
            sweeps = ((_SWEEP_VX, "backward", 2 * _SWEEP_SEC_BASE),
                      (_SWEEP_VY, "right", _SWEEP_SEC_BASE),
                      (-_SWEEP_VY, "left", 2 * _SWEEP_SEC_BASE))
        else:
            sweeps = ((_SWEEP_VY, "right", _SWEEP_SEC_BASE),
                      (-_SWEEP_VY, "left", 2 * _SWEEP_SEC_BASE))

        for direction, label, sweep_sec in sweeps:
            yasmin.YASMIN_LOG_INFO(
                f"Gate {self.position} reacquire: sweep {label} "
                f"{sweep_sec:.0f} s at {abs(direction):.2f} m/s, "
                f"need {_CONFIRM} consecutive detections."
            )
            found = 0
            start = self.node.get_clock().now()
            while self.node.get_clock().now() - start < Duration(seconds=sweep_sec):
                if self.check_timeout():
                    yasmin.YASMIN_LOG_WARN(f"Gate {self.position} reacquire: timeout.")
                    drone.move_velocity()
                    return TIMEOUT

                result: DetectionResult | None = handler.take_photo()
                window = (
                    result.filter_by_class([class_name]) if result is not None else None
                )
                if window:
                    found += 1
                    det = max(window, key=lambda d: d.area)
                    yasmin.YASMIN_LOG_INFO(
                        f"Gate {self.position} reacquire {label}: "
                        f"found ({found}/{_CONFIRM}) conf={det.confidence:.2f}."
                    )
                    if found >= _CONFIRM:
                        yasmin.YASMIN_LOG_INFO(
                            f"Gate {self.position} reacquire: confirmed on {label}."
                        )
                        drone.move_velocity()
                        return SUCCEED
                else:
                    if found:
                        yasmin.YASMIN_LOG_INFO(
                            f"Gate {self.position} reacquire {label}: "
                            f"lost after {found} hits, reset."
                        )
                    found = 0

                altitude = drone.get_altitude()
                output_z = pid_z.update(altitude) if altitude is not None else 0.0
                if direction == "backward":
                    drone.move_velocity(vx=direction, vz=output_z)
                else:
                    drone.move_velocity(vy=direction, vz=output_z)

        yasmin.YASMIN_LOG_WARN(
            f"Gate {self.position} reacquire: not confirmed, leave gate."
        )
        drone.move_velocity()
        return CANCEL

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(
            seconds=self.timeout
        ) or now - self.start_mission > Duration(seconds=self.mission_timeout)
