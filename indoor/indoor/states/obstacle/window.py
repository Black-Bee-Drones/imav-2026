from rclpy.time import Time, Duration

import cv2
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
_CYAN = (255, 255, 0)
_GREEN = (0, 255, 0)
_MAGENTA = (255, 0, 255)
_FONT = cv2.FONT_HERSHEY_SIMPLEX
_DEPTH_MIN = 0.05
_DEPTH_MAX = 5.0


def _px_to_m(px: float, range_m: float | None, focal: float) -> float:
    if focal <= 0 or range_m is None or range_m <= 0:
        return 0.0
    return px * range_m / focal


def _colorize_depth(depth) -> np.ndarray:
    vis = np.zeros((*depth.shape[:2], 3), dtype=np.uint8)
    valid = np.isfinite(depth) & (depth > _DEPTH_MIN)
    if not np.any(valid):
        return vis
    norm = np.clip((depth - _DEPTH_MIN) / (_DEPTH_MAX - _DEPTH_MIN), 0.0, 1.0)
    gray = np.zeros(depth.shape[:2], dtype=np.uint8)
    gray[valid] = (norm[valid] * 255.0).astype(np.uint8)
    vis = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    vis[~valid] = 0
    return vis


def _overlay_gate_ring(img, depth, color_shape, rects) -> None:
    if img is None or depth is None or color_shape is None or rects is None:
        return
    ox1, oy1, ox2, oy2, ix1, iy1, ix2, iy2 = rects
    ch, cw = color_shape[:2]
    dh, dw = depth.shape[:2]
    if cw <= 0 or ch <= 0 or dw <= 0 or dh <= 0:
        return
    sx, sy = cw / dw, ch / dh

    def _c(x, y):
        return int(round(x * sx)), int(round(y * sy))

    c_ox1, c_oy1 = _c(ox1, oy1)
    c_ox2, c_oy2 = _c(ox2, oy2)
    c_ix1, c_iy1 = _c(ix1, iy1)
    c_ix2, c_iy2 = _c(ix2, iy2)
    c_ox1, c_oy1 = max(0, c_ox1), max(0, c_oy1)
    c_ox2, c_oy2 = min(cw, c_ox2), min(ch, c_oy2)
    if c_ox2 <= c_ox1 or c_oy2 <= c_oy1:
        return
    crop = _colorize_depth(depth[oy1:oy2, ox1:ox2])
    crop = cv2.resize(crop, (c_ox2 - c_ox1, c_oy2 - c_oy1), interpolation=cv2.INTER_NEAREST)
    hx1 = max(0, min(crop.shape[1], c_ix1 - c_ox1))
    hx2 = max(0, min(crop.shape[1], c_ix2 - c_ox1))
    hy1 = max(0, min(crop.shape[0], c_iy1 - c_oy1))
    hy2 = max(0, min(crop.shape[0], c_iy2 - c_oy1))
    if hx2 > hx1 and hy2 > hy1:
        crop[hy1:hy2, hx1:hx2] = 0
    roi = img[c_oy1:c_oy2, c_ox1:c_ox2]
    mask = crop.any(axis=2)
    if np.any(mask):
        blended = cv2.addWeighted(roi, 0.45, crop, 0.55, 0)
        roi[mask] = blended[mask]
    cv2.rectangle(img, (c_ox1, c_oy1), (c_ox2, c_oy2), _CYAN, 1)
    cv2.rectangle(img, (c_ix1, c_iy1), (c_ix2, c_iy2), _MAGENTA, 1)


def _px_to_m(px: float, range_m: float | None, focal: float) -> float:
    if focal <= 0 or range_m is None or range_m <= 0:
        return 0.0
    return px * range_m / focal


def _frame(result: DetectionResult):
    img = result.annotated_image
    return img if img is not None else result.image


def _annotate_align(
    img,
    sx: float,
    sy: float,
    det_xy,
    error_y: float,
    error_z: float,
    err_y_m: float,
    err_z_m: float,
    vy: float,
    vz: float,
    aligned: int,
    n: int,
    phase: str,
    depth_m: float | None,
) -> None:
    if img is None:
        return
    sx, sy = int(round(sx)), int(round(sy))
    cv2.drawMarker(img, (sx, sy), _CYAN, cv2.MARKER_TILTED_CROSS, 14, 1)
    if det_xy is not None:
        cx, cy = int(round(det_xy[0])), int(round(det_xy[1]))
        cv2.drawMarker(img, (cx, cy), _MAGENTA, cv2.MARKER_CROSS, 12, 1)
        if abs(cx - sx) > 3 or abs(cy - sy) > 3:
            cv2.arrowedLine(
                img, (cx, cy), (sx, sy), _GREEN, 1, tipLength=0.12, line_type=cv2.LINE_AA
            )
    depth = "—" if depth_m is None else f"{depth_m:.2f}m"
    line = (
        f"{phase} {aligned}/{n}  "
        f"y={error_y:+.0f}px {err_y_m:+.2f}m  z={error_z:+.0f}px {err_z_m:+.2f}m  "
        f"v {vy:+.2f} {vz:+.2f}  {depth}"
    )
    (tw, th), _ = cv2.getTextSize(line, _FONT, 0.45, 1)
    cv2.rectangle(img, (6, 6), (10 + tw, 14 + th), (0, 0, 0), -1)
    cv2.putText(img, line, (8, 8 + th), _FONT, 0.45, _GREEN, 1, cv2.LINE_AA)


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
        save_jpg = blackboard.get("save_jpg")

        pass_x = config.obstacle_gate_pass_x
        creep_vx = config.obstacle_gate_creep_vx
        standoff = config.obstacle_gate_standoff
        skip = getattr(config, f"obstacle_gate_{self.position}_skip")

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

        if self.position == "room":
            handler: ImageHandler | None = blackboard.get("image_handler_south")
            if handler is None:
                yasmin.YASMIN_LOG_WARN("Gate room: no south camera, skip.")
                return "skip"
            offset_y_m = config.camera_south_offset_y
            offset_z_m = config.camera_south_offset_z
            hfov = config.camera_south_hfov
            vfov = config.camera_south_vfov
            pass_sign = -1.0
            use_depth = False
            depth_cam = None
        else:
            handler = blackboard.get("image_handler_north")
            offset_y_m = config.camera_north_offset_y
            offset_z_m = config.camera_north_offset_z
            hfov = config.camera_north_hfov
            vfov = config.camera_north_vfov
            pass_sign = 1.0
            use_depth = True
            depth_cam = blackboard.get("depth_cam")
            if depth_cam is None and isinstance(handler.camera, DepthCam):
                depth_cam = handler.camera

        handler.image_processing_callback = blackboard.get("callback_gate")

        lost_tolerance = config.obstacle_gate_lost_tolerance
        aligned_tolerance_m = config.obstacle_gate_aligned_tolerance_m
        aligned_threshold = config.obstacle_gate_aligned_threshold
        class_name = config.model_gate_class_name
        gate_limits = (config.obstacle_gate_output_min, config.obstacle_gate_output_max)
        gate_deadband = config.obstacle_gate_output_deadband

        pid_y = None
        pid_z = None
        lost = 0
        aligned = 0
        phase = "align"
        ever_aligned = False
        color_shape = None
        width = height = 0
        cx_sp = cy_sp = 0.0

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
                fx = ImageCalculus.focal_length_px(width, hfov)
                fy = ImageCalculus.focal_length_px(height, vfov)
                pid_y = PIDController(
                    kp=config.obstacle_gate_kp,
                    kd=config.obstacle_gate_kd,
                    ki=config.obstacle_gate_ki,
                    setpoint=0.0,
                    output_limits=gate_limits,
                    output_deadband=gate_deadband,
                )
                pid_z = PIDController(
                    kp=config.obstacle_gate_kp,
                    kd=config.obstacle_gate_kd,
                    ki=config.obstacle_gate_ki,
                    setpoint=0.0,
                    output_limits=gate_limits,
                    output_deadband=gate_deadband,
                )
                cx_sp = width / 2 + ImageCalculus.meters_to_pixels(
                    offset_y_m, standoff, fx
                )
                cy_sp = height / 2 + ImageCalculus.meters_to_pixels(
                    offset_z_m, standoff, fy
                )
                yasmin.YASMIN_LOG_INFO(
                    f"Gate {self.position}: image {width}x{height}, "
                    f"fx={fx:.0f} fy={fy:.0f} "
                    f"offset y={offset_y_m:.3f} z={offset_z_m:.3f} m "
                    f"standoff={standoff:.2f} m pass_sign={pass_sign:+.0f}."
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
                frame = _frame(result)
                _annotate_align(
                    frame,
                    cx_sp,
                    cy_sp,
                    None,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    aligned,
                    aligned_threshold,
                    f"lost {lost}/{lost_tolerance}",
                    None,
                )
                save_jpg("gate_annotated", "annotated", frame)
                drone.move_velocity()
                if lost >= lost_tolerance:
                    if phase == "creep" and ever_aligned:
                        return self._commit(drone, pass_sign * pass_x, "lost")
                    yasmin.YASMIN_LOG_WARN(
                        f"Gate {self.position}: lost {lost} frames, reacquire."
                    )
                    return "reacquire"
                continue

            lost = 0
            det = max(window, key=lambda d: d.area)
            cx, cy = det.center
            median = None
            depth_frame = None
            ring_rects = None
            if use_depth:
                median, depth_frame, ring_rects = self._frame_median_m(
                    depth_cam, det, color_shape
                )
            range_m = median if median is not None else standoff
            fx = ImageCalculus.focal_length_px(width, hfov)
            fy = ImageCalculus.focal_length_px(height, vfov)
            cx_sp = width / 2 + ImageCalculus.meters_to_pixels(
                offset_y_m, range_m, fx
            )
            cy_sp = height / 2 + ImageCalculus.meters_to_pixels(
                offset_z_m, range_m, fy
            )
            error_y = cx_sp - cx
            error_z = cy_sp - cy
            err_y_m = _px_to_m(error_y, range_m, fx)
            err_z_m = _px_to_m(error_z, range_m, fy)
            output_y = pid_y.update(-err_y_m)
            output_z = pid_z.update(-err_z_m)

            if phase == "align":
                if (
                    abs(err_y_m) < aligned_tolerance_m
                    and abs(err_z_m) < aligned_tolerance_m
                ):
                    aligned += 1
                else:
                    aligned = 0

            frame = _frame(result)
            _overlay_gate_ring(frame, depth_frame, color_shape, ring_rects)
            _annotate_align(
                frame,
                cx_sp,
                cy_sp,
                (cx, cy),
                error_y,
                error_z,
                err_y_m,
                err_z_m,
                output_y,
                output_z,
                aligned,
                aligned_threshold,
                phase,
                median,
            )
            save_jpg("gate_annotated", "annotated", frame)

            if phase == "align":
                yasmin.YASMIN_LOG_INFO(
                    f"Gate {self.position} align ({aligned}/{aligned_threshold}) "
                    f"| error y={error_y:.0f}px {err_y_m:+.3f}m "
                    f"z={error_z:.0f}px {err_z_m:+.3f}m "
                    f"| R={range_m:.2f}m "
                    f"| vel y={output_y:.2f} z={output_z:.2f} m/s"
                )
                drone.move_velocity(vy=output_y, vz=output_z)
                if aligned >= aligned_threshold:
                    if config.obstacle_gate_align_only:
                        continue
                    if not use_depth:
                        return self._commit(drone, pass_sign * pass_x, "standoff")
                    ever_aligned = True
                    phase = "creep"
                    yasmin.YASMIN_LOG_INFO(
                        f"Gate {self.position} aligned. "
                        f"Creep vx={pass_sign * creep_vx:.2f} m/s."
                    )
                continue

            vx = pass_sign * creep_vx
            depth_txt = "none" if median is None else f"{median:.2f} m"
            yasmin.YASMIN_LOG_INFO(
                f"Gate {self.position} creep "
                f"| error y={error_y:.0f}px {err_y_m:+.3f}m "
                f"z={error_z:.0f}px {err_z_m:+.3f}m "
                f"| vel x={vx:.2f} y={output_y:.2f} z={output_z:.2f} m/s "
                f"| depth={depth_txt}"
            )
            drone.move_velocity(vx=vx, vy=output_y, vz=output_z)
            if median is not None and median <= standoff:
                commit_x = pass_sign * max(0.6, min(median + 0.5, 2.0))
                return self._commit(drone, commit_x, "depth")

    def _commit(self, drone: MavlinkDrone, commit_x: float, reason: str):
        yasmin.YASMIN_LOG_INFO(
            f"Gate {self.position} commit ({reason}): "
            f"{commit_x:+.1f} m body-x."
        )
        drone.move_velocity()
        drone.move_to(x=commit_x, y=0.0, z=0.0, yaw=0.0, precision=0.12)
        return SUCCEED

    @staticmethod
    def _frame_median_m(depth_cam, det, color_shape):
        empty = (None, None, None)
        if depth_cam is None or color_shape is None:
            return empty
        try:
            frame = depth_cam.get_depth_frame()
        except Exception:
            return empty
        if frame is None or getattr(frame, "size", 0) == 0:
            return empty
        x1, y1, x2, y2 = (float(v) for v in det.xyxy)
        dh, dw = frame.shape[:2]
        ch, cw = color_shape[:2]
        if cw <= 0 or ch <= 0 or dw <= 0 or dh <= 0:
            return empty
        x1 *= dw / cw
        x2 *= dw / cw
        y1 *= dh / ch
        y2 *= dh / ch
        w = x2 - x1
        h = y2 - y1
        if w < 8.0 or h < 8.0:
            return empty
        ix1 = max(0, int(x1))
        ix2 = min(dw, int(x2))
        iy1 = max(0, int(y1))
        iy2 = min(dh, int(y2))
        ox1 = max(0, int(x1 - _FRAME_EXPAND * w))
        ox2 = min(dw, int(x2 + _FRAME_EXPAND * w))
        oy1 = max(0, int(y1 - _FRAME_EXPAND * h))
        oy2 = min(dh, int(y2 + _FRAME_EXPAND * h))
        if ox2 <= ox1 or oy2 <= oy1 or ix2 <= ix1 or iy2 <= iy1:
            return empty
        rects = (ox1, oy1, ox2, oy2, ix1, iy1, ix2, iy2)
        ring = frame[oy1:oy2, ox1:ox2].copy()
        hx1 = ix1 - ox1
        hx2 = ix2 - ox1
        hy1 = iy1 - oy1
        hy2 = iy2 - oy1
        ring[hy1:hy2, hx1:hx2] = 0.0
        valid = ring[np.isfinite(ring) & (ring > 0.05)]
        if valid.size < 16:
            return (None, frame, rects)
        return float(np.median(valid)), frame, rects

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(
            seconds=self.timeout
        ) or now - self.start_mission > Duration(seconds=self.mission_timeout)
