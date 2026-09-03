from rclpy.time import Time, Duration

import numpy as np
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, PIDController
from nectar.vision import DepthCam, ImageCalculus, ImageHandler
from nectar.ai import DetectionResult

from ...config import Config

_FRAME_EXPAND = 0.25


class Window(State):
    def __init__(self, position: str):
        super().__init__(outcomes=[SUCCEED, TIMEOUT, "reacquire", "skip"])

        self.position = position
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        self.timeout: int = config.timeout
        self.start_time: Time = blackboard.get("start_time")

        if self.position == "room":
            self.start_mission: Time = config.inspect_start_time
            self.mission_timeout: int = config.inspect_timeout
        else:
            self.start_mission: Time = config.obstacle_start_time
            self.mission_timeout: int = config.obstacle_timeout

        drone: MavlinkDrone = blackboard.get("drone")

        handler: ImageHandler = blackboard.get("image_handler_north")
        handler.image_processing_callback = blackboard.get("callback_gate")

        pass_x = config.obstacle_gate_pass_x
        creep_vx = config.obstacle_gate_creep_vx
        standoff = config.obstacle_gate_standoff
        skip = getattr(config, f"obstacle_gate_{self.position}_skip")
        depth_cam = blackboard.get("depth_cam")
        if depth_cam is None and isinstance(handler.camera, DepthCam):
            depth_cam = handler.camera

        if skip:
            if self.position == "room":
                safe_alt = config.safe_alt
                yasmin.YASMIN_LOG_INFO(
                    f"Gate {self.position} skip: overfly at {safe_alt:.1f} m, "
                    f"then +{pass_x:.1f} m body-forward."
                )
                drone.move_to(
                    x=0.0,
                    y=0.0,
                    z=safe_alt - drone.get_altitude(),
                    yaw=0.0,
                    precision=0.12,
                )
                drone.move_to(x=pass_x, y=0.0, z=0.0, yaw=0.0, precision=0.12)
            else:
                yasmin.YASMIN_LOG_INFO(
                    f"Gate {self.position} skip: leave without search."
                )
            return "skip"

        lost_tolerance = config.obstacle_gate_lost_tolerance
        aligned_tolerance = config.obstacle_gate_aligned_tolerance
        aligned_threshold = config.obstacle_gate_aligned_threshold
        class_name = config.model_gate_class_name
        offset_y_m = config.camera_north_offset_y
        offset_z_m = config.camera_north_offset_z
        hfov = config.camera_north_hfov

        pid_y = None
        pid_z = None
        lost = 0
        aligned = 0
        phase = "align"
        ever_aligned = False
        color_shape = None
        width = height = 0

        while True:
            if self.check_timeout():
                yasmin.YASMIN_LOG_WARN(f"Gate {self.position}: timeout.")
                return TIMEOUT

            result: DetectionResult | None = handler.take_photo()
            if result is None:
                yasmin.YASMIN_LOG_WARN(f"Gate {self.position}: no frame.")
                continue

            if pid_y is None:
                height, width = result.image.shape[:2]
                color_shape = (height, width)
                focal = ImageCalculus.focal_length_px(width, hfov)
                pid_y = PIDController(
                    kp=config.obstacle_xy_kp,
                    kd=config.obstacle_xy_kd,
                    ki=config.obstacle_xy_ki,
                    setpoint=width / 2
                    + ImageCalculus.meters_to_pixels(
                        offset_y_m, standoff, focal
                    ),
                    output_limits=(-0.3, 0.3),
                    output_deadband=0.02,
                )
                pid_z = PIDController(
                    kp=config.obstacle_z_kp,
                    kd=config.obstacle_z_kd,
                    ki=config.obstacle_z_ki,
                    setpoint=height / 2
                    + ImageCalculus.meters_to_pixels(
                        offset_z_m, standoff, focal
                    ),
                    output_limits=(-0.3, 0.3),
                    output_deadband=0.02,
                )
                yasmin.YASMIN_LOG_INFO(
                    f"Gate {self.position}: image {width}x{height}, "
                    f"setpoint=({pid_y.setpoint:.0f}, {pid_z.setpoint:.0f}) "
                    f"shift y={pid_y.setpoint - width / 2:.0f} "
                    f"z={pid_z.setpoint - height / 2:.0f} px "
                    f"(offset y={offset_y_m:.3f} z={offset_z_m:.3f} m, "
                    f"range={standoff:.2f} m)."
                )

            window = result.filter_by_class([class_name])
            if not window:
                lost += 1
                aligned = 0
                dets = (
                    ", ".join(f"{d.class_name}:{d.confidence:.2f}" for d in result)
                    or "none"
                )
                yasmin.YASMIN_LOG_INFO(
                    f"Gate {self.position} lost ({lost}/{lost_tolerance}) "
                    f"phase={phase} detections=[{dets}]."
                )
                drone.move_velocity()
                if lost >= lost_tolerance:
                    if phase == "creep" and ever_aligned:
                        return self._commit(drone, pass_x, "lost")
                    yasmin.YASMIN_LOG_WARN(
                        f"Gate {self.position}: lost {lost} frames, reacquire."
                    )
                    return "reacquire"
                continue

            lost = 0
            det = max(window, key=lambda d: d.area)
            cx, cy = det.center
            median = self._frame_median_m(depth_cam, det, color_shape)
            range_m = median if median is not None else standoff
            focal = ImageCalculus.focal_length_px(width, hfov)
            pid_y.setpoint = width / 2 + ImageCalculus.meters_to_pixels(
                offset_y_m, range_m, focal
            )
            pid_z.setpoint = height / 2 + ImageCalculus.meters_to_pixels(
                offset_z_m, range_m, focal
            )
            error_y = pid_y.setpoint - cx
            error_z = pid_z.setpoint - cy
            output_y = pid_y.update(cx)
            output_z = pid_z.update(cy)

            if phase == "align":
                if (
                    abs(error_y) < aligned_tolerance
                    and abs(error_z) < aligned_tolerance
                ):
                    aligned += 1
                else:
                    aligned = 0
                yasmin.YASMIN_LOG_INFO(
                    f"Gate {self.position} align ({aligned}/{aligned_threshold}) "
                    f"| error y={error_y:.0f} z={error_z:.0f} px "
                    f"| vel y={output_y:.2f} z={output_z:.2f} m/s"
                )
                drone.move_velocity(vy=output_y, vz=output_z)
                if aligned >= aligned_threshold:
                    ever_aligned = True
                    phase = "creep"
                    yasmin.YASMIN_LOG_INFO(
                        f"Gate {self.position} aligned. Creep vx={creep_vx:.2f} m/s."
                    )
                continue

            depth_txt = "none" if median is None else f"{median:.2f} m"
            yasmin.YASMIN_LOG_INFO(
                f"Gate {self.position} creep "
                f"| error y={error_y:.0f} z={error_z:.0f} px "
                f"| vel x={creep_vx:.2f} y={output_y:.2f} z={output_z:.2f} m/s "
                f"| depth={depth_txt}"
            )
            drone.move_velocity(vx=creep_vx, vy=output_y, vz=output_z)
            if median is not None and median <= standoff:
                commit_x = max(0.6, min(median + 0.5, 2.0))
                return self._commit(drone, commit_x, "depth")

    def _commit(self, drone: MavlinkDrone, commit_x: float, reason: str):
        yasmin.YASMIN_LOG_INFO(
            f"Gate {self.position} commit ({reason}): "
            f"+{commit_x:.1f} m body-forward."
        )
        drone.move_velocity()
        drone.move_to(x=commit_x, y=0.0, z=0.0, yaw=0.0, precision=0.12)
        return SUCCEED

    @staticmethod
    def _frame_median_m(depth_cam, det, color_shape) -> float | None:
        if depth_cam is None or color_shape is None:
            return None
        try:
            frame = depth_cam.get_depth_frame()
        except Exception:
            return None
        if frame is None or getattr(frame, "size", 0) == 0:
            return None
        x1, y1, x2, y2 = (float(v) for v in det.xyxy)
        dh, dw = frame.shape[:2]
        ch, cw = color_shape[:2]
        if cw <= 0 or ch <= 0 or dw <= 0 or dh <= 0:
            return None
        x1 *= dw / cw
        x2 *= dw / cw
        y1 *= dh / ch
        y2 *= dh / ch
        w = x2 - x1
        h = y2 - y1
        if w < 8.0 or h < 8.0:
            return None
        ix1 = max(0, int(x1))
        ix2 = min(dw, int(x2))
        iy1 = max(0, int(y1))
        iy2 = min(dh, int(y2))
        ox1 = max(0, int(x1 - _FRAME_EXPAND * w))
        ox2 = min(dw, int(x2 + _FRAME_EXPAND * w))
        oy1 = max(0, int(y1 - _FRAME_EXPAND * h))
        oy2 = min(dh, int(y2 + _FRAME_EXPAND * h))
        if ox2 <= ox1 or oy2 <= oy1 or ix2 <= ix1 or iy2 <= iy1:
            return None
        ring = frame[oy1:oy2, ox1:ox2].copy()
        hx1 = ix1 - ox1
        hx2 = ix2 - ox1
        hy1 = iy1 - oy1
        hy2 = iy2 - oy1
        ring[hy1:hy2, hx1:hx2] = 0.0
        valid = ring[np.isfinite(ring) & (ring > 0.05)]
        if valid.size < 16:
            return None
        return float(np.median(valid))

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(
            seconds=self.timeout
        ) or now - self.start_mission > Duration(seconds=self.mission_timeout)
