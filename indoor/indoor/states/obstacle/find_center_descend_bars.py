from math import hypot, isnan
from pathlib import Path

import cv2 as cv
from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, MoveReference, PIDController
from nectar.vision import (
    ColorSpace,
    ImageCalculus,
    ImageHandler,
    LineDetector,
    RotatedRect,
)

from ...config import Config

_JPEG_QUALITY = 80
_XY_LIMIT = 0.18
_CENTER_FRAMES = 8
_EDGE_PX = 16
_MOVE_PRECISION = 0.12
_GAP_OVERRUN_M = 0.25

def _in_fov(cx: float, cy: float, width: int, height: int) -> bool:
    if isnan(cx) or isnan(cy):
        return False
    return _EDGE_PX <= cx < width - _EDGE_PX and _EDGE_PX <= cy < height - _EDGE_PX

def _half_gap_px(focal: float, bar_gap: float, altitude: float | None) -> float:
    if altitude is None:
        return 0.0
    return ImageCalculus.meters_to_pixels(bar_gap / 2.0, max(altitude, 0.5), focal)

def _takeoff_x(drone: MavlinkDrone) -> float | None:
    pose = drone.position or getattr(drone, "local_pose", None)
    takeoff = getattr(drone, "_takeoff_position", None) or getattr(
        drone, "_takeoff_local", None
    )
    if pose is None or takeoff is None:
        return None
    px = getattr(getattr(pose, "position", None), "x", None)
    tx = getattr(getattr(takeoff, "position", None), "x", None)
    if px is None or tx is None:
        return None
    return float(px) - float(tx)


class FindCenterDescendBars(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, TIMEOUT])

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        config: Config = blackboard.get('config')
        drone: MavlinkDrone = blackboard.get("drone")

        safe_alt: float = config.safe_alt
        self.start_time: Time = blackboard.get("start_time")
        self.timeout: int = config.timeout

        self.start_mission: Time = config.obstacle_start_time
        self.mission_timeout: int = config.obstacle_timeout

        match config.obstacle_blue_1:
            case 1:
                alt_1 = config.obstacle_blue_step_alt_1
            case 2:
                alt_1 = config.obstacle_blue_step_alt_2
            case 3:
                alt_1 = config.obstacle_blue_step_alt_3
            case _:
                alt_1 = safe_alt

        if self.check_timeout():
            return TIMEOUT

        handler: ImageHandler = blackboard.get("image_handler_down")
        handler.image_processing_callback = None

        det_red = det_blue = None
        calib = config.color_calibration_path
        if calib and not Path(calib).exists():
            yasmin.YASMIN_LOG_WARN(
                f"Bars: calibration missing ({calib}), using default."
            )
            calib = None

        try:
            det_red = LineDetector(
                color=config.obstacle_bar_red_color,
                estimation_method=RotatedRect(),
                color_space=ColorSpace.HSV,
                file_path=calib,
            )
            det_blue = LineDetector(
                color=config.obstacle_bar_blue_color,
                estimation_method=RotatedRect(),
                color_space=ColorSpace.LAB,
                file_path=calib,
            )
        except ValueError as e:
            yasmin.YASMIN_LOG_WARN(
                f"Bars: LineDetector load failed ({e}). Hold altitude."
            )
            return self._hold_and_descend(drone, alt_1)

        center_tol = config.obstacle_bar_center_tolerance
        acquire_timeout = config.obstacle_bar_acquire_timeout
        find_vx = config.obstacle_bar_find_vx
        bar_gap = config.obstacle_bar_gap
        gap_x = config.obstacle_bar_start_x
        offset_x_m = config.camera_down_offset_x
        offset_y_m = config.camera_down_offset_y
        hfov = config.camera_down_hfov
        roi = (
            int(config.obstacle_bar_roi_w),
            int(config.obstacle_bar_roi_h),
        )
        debug_root = blackboard.get("debug_root")
        jpeg_dir = None
        if debug_root is not None:
            jpeg_dir = Path(debug_root) / "bars"
            jpeg_dir.mkdir(parents=True, exist_ok=True)

        pid_x = None
        pid_y = None

        phase = "find"
        centered = 0
        acquire_start = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO(
            f"Bars: search red/blue, center, descend to {alt_1:.2f} m."
        )

        while True:
            if self.check_timeout():
                yasmin.YASMIN_LOG_WARN("Bars: timeout.")
                drone.move_velocity()
                return TIMEOUT

            frame = handler.take_photo()
            if frame is None:
                yasmin.YASMIN_LOG_WARN("Bars: no down frame.")
                continue

            height, width = frame.shape[:2]
            altitude = drone.get_altitude()
            if pid_x is None:
                focal = ImageCalculus.focal_length_px(width, hfov)
                pid_x = PIDController(
                    kp=config.obstacle_xy_kp,
                    kd=config.obstacle_xy_kd,
                    ki=config.obstacle_xy_ki,
                    setpoint=height / 2
                    + ImageCalculus.meters_to_pixels(offset_x_m, altitude, focal),
                    output_limits=(-_XY_LIMIT, _XY_LIMIT),
                    output_deadband=0.02,
                )
                pid_y = PIDController(
                    kp=config.obstacle_xy_kp,
                    kd=config.obstacle_xy_kd,
                    ki=config.obstacle_xy_ki,
                    setpoint=width / 2
                    + ImageCalculus.meters_to_pixels(offset_y_m, altitude, focal),
                    output_limits=(-_XY_LIMIT, _XY_LIMIT),
                    output_deadband=0.02,
                )
                yasmin.YASMIN_LOG_INFO(
                    f"Bars: image {width}x{height}, roi={roi[0]}x{roi[1]}, "
                    f"setpoint=({pid_y.setpoint:.0f}, {pid_x.setpoint:.0f}) "
                    f"shift x={pid_y.setpoint - width / 2:.0f} "
                    f"y={pid_x.setpoint - height / 2:.0f} px "
                    f"(offset x={offset_x_m:.3f} y={offset_y_m:.3f} m)."
                )
            else:
                focal = ImageCalculus.focal_length_px(width, hfov)
                pid_x.setpoint = height / 2 + ImageCalculus.meters_to_pixels(
                    offset_x_m, altitude, focal
                )
                pid_y.setpoint = width / 2 + ImageCalculus.meters_to_pixels(
                    offset_y_m, altitude, focal
                )

            vis_red = frame.copy()
            vis_blue = frame.copy()
            _, _, cx_r, cy_r, _, _, _ = det_red.detect_line(
                vis_red, region=roi, draw=True
            )
            _, _, cx_b, cy_b, _, _, _ = det_blue.detect_line(
                vis_blue, region=roi, draw=True
            )
            have_red = not isnan(cx_r) and not isnan(cy_r)
            have_blue = not isnan(cx_b) and not isnan(cy_b)
            use_red = _in_fov(cx_r, cy_r, width, height)
            use_blue = _in_fov(cx_b, cy_b, width, height)
            pose_x = _takeoff_x(drone)

            if phase == "find":
                if use_red or use_blue:
                    phase = "center"
                    src = []
                    if use_red:
                        src.append("red")
                    if use_blue:
                        src.append("blue")
                    yasmin.YASMIN_LOG_INFO(f"Bars: detected {', '.join(src)}, center.")
                else:
                    if self.node.get_clock().now() - acquire_start > Duration(
                        seconds=acquire_timeout
                    ):
                        yasmin.YASMIN_LOG_WARN("Bars: acquire timeout, hold altitude.")
                        return self._hold_and_descend(drone, alt_1)
                    at_gap = pose_x is not None and pose_x >= gap_x
                    if at_gap:
                        yasmin.YASMIN_LOG_WARN(
                            f"Bars: reached gap x={pose_x:.2f} m with no "
                            "detection, hold."
                        )
                        return self._hold_and_descend(drone, alt_1)
                    seen = []
                    if have_red:
                        seen.append(f"red=({cx_r:.0f},{cy_r:.0f})")
                    if have_blue:
                        seen.append(f"blue=({cx_b:.0f},{cy_b:.0f})")
                    yasmin.YASMIN_LOG_INFO(
                        f"Bars find vx={find_vx:.2f} "
                        f'detections=[{", ".join(seen) or "none"}]'
                        f'{"" if pose_x is None else f" x={pose_x:.2f}"}.'
                    )
                    drone.move_velocity(vx=find_vx)
                    continue

            if use_red and use_blue:
                mid_x = 0.5 * (cx_r + cx_b)
                mid_y = 0.5 * (cy_r + cy_b)
                source = "both"
            elif use_blue:
                dpx = _half_gap_px(focal, bar_gap, altitude)
                mid_x = cx_b
                mid_y = cy_b + dpx
                source = "blue"
            elif use_red:
                dpx = _half_gap_px(focal, bar_gap, altitude)
                mid_x = cx_r
                mid_y = cy_r - dpx
                source = "red"
            else:
                centered = 0
                seen = []
                if have_red:
                    seen.append(f"red=({cx_r:.0f},{cy_r:.0f})")
                if have_blue:
                    seen.append(f"blue=({cx_b:.0f},{cy_b:.0f})")
                yasmin.YASMIN_LOG_INFO(
                    f"Bars center lost line, hover "
                    f'detections=[{", ".join(seen) or "none"}].'
                )
                drone.move_velocity()
                continue

            err_x = mid_x - pid_y.setpoint
            err_y = mid_y - pid_x.setpoint
            offset = hypot(err_x, err_y)
            vx = -pid_x.update(mid_y)
            vy = pid_y.update(mid_x)
            if pose_x is not None and pose_x > gap_x + _GAP_OVERRUN_M:
                vx = min(0.0, vx)

            if offset < center_tol:
                centered += 1
            else:
                centered = 0

            yasmin.YASMIN_LOG_INFO(
                f"Bars center source={source} | "
                f"red=({cx_r:.0f},{cy_r:.0f}) "
                f"blue=({cx_b:.0f},{cy_b:.0f}) "
                f"mid=({mid_x:.0f},{mid_y:.0f}) "
                f"err x={err_x:.0f} y={err_y:.0f} px ({offset:.0f}"
                f'{" in-gap" if offset <= center_tol else ""}) '
                f"{centered}/{_CENTER_FRAMES} "
                f"| vel x={vx:.2f} y={vy:.2f} z=0.00 m/s"
                f'{"" if altitude is None else f" alt={altitude:.2f}"}'
                f'{"" if pose_x is None else f" x={pose_x:.2f}"}'
            )
            drone.move_velocity(vx=vx, vy=vy)

            if jpeg_dir is not None:
                vis = vis_red.copy()
                cv.circle(vis, (int(mid_x), int(mid_y)), 6, (0, 255, 255), 2)
                cv.circle(
                    vis,
                    (int(pid_y.setpoint), int(pid_x.setpoint)),
                    4,
                    (0, 255, 0),
                    2,
                )
                now = self.node.get_clock().now()
                stamp = now.nanoseconds
                cv.imwrite(
                    str(jpeg_dir / f"bars-{stamp}.jpg"),
                    vis,
                    [int(cv.IMWRITE_JPEG_QUALITY), _JPEG_QUALITY],
                )

            if centered >= _CENTER_FRAMES:
                break

        drone.move_velocity()
        yasmin.YASMIN_LOG_INFO(
            f"Bars: centered {_CENTER_FRAMES} frames, descend to {alt_1:.2f} m."
        )
        drone.move_to(
            x=None,
            y=None,
            z=alt_1,
            yaw=None,
            reference=MoveReference.TAKEOFF,
            precision=_MOVE_PRECISION,
        )
        return SUCCEED

    def _hold_and_descend(self, drone: MavlinkDrone, alt_1: float):
        drone.move_velocity()
        yasmin.YASMIN_LOG_INFO(f"Bars: stop search, descend to {alt_1:.2f} m.")
        drone.move_to(
            x=None,
            y=None,
            z=alt_1,
            yaw=None,
            reference=MoveReference.TAKEOFF,
            precision=_MOVE_PRECISION,
        )
        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(
            seconds=self.timeout
        ) or now - self.start_mission > Duration(seconds=self.mission_timeout)
