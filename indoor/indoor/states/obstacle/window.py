from rclpy.time import Time, Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, TIMEOUT

from nectar.control import MavlinkDrone, PIDController
from nectar.vision import DepthCam, ImageCalculus, ImageHandler
from nectar.ai import DetectionResult

from ...config import Config
from ...gate_range import (
    bbox_clipped,
    depth_on_color,
    draw_align_markers,
    draw_hud,
    fmt_z,
    fuse_z,
    overlay_gate,
    px_to_m,
    smooth_z,
    z_from_bbox,
    z_from_frame,
)

_COMMIT_MIN = 0.7
_COMMIT_MAX = 1.8


def _frame(result: DetectionResult):
    img = result.annotated_image
    return img if img is not None else result.image


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
        if self.position == "room":
            creep_vx = config.inspect_gate_creep_vx
            standoff = config.inspect_gate_standoff
            bbox_frac = config.inspect_gate_bbox_frac
            expand = config.inspect_gate_frame_expand
            commit_extra = config.inspect_gate_commit_extra
        else:
            creep_vx = config.obstacle_gate_creep_vx
            standoff = config.obstacle_gate_standoff
            bbox_frac = config.obstacle_gate_bbox_frac
            expand = config.obstacle_gate_frame_expand
            commit_extra = config.obstacle_gate_commit_extra
        gate_width = config.obstacle_gate_width
        z_min = config.obstacle_gate_depth_min
        z_max = config.obstacle_gate_depth_max
        fuse_delta = config.obstacle_gate_fuse_delta
        skip = getattr(config, f"obstacle_gate_{self.position}_skip")
        is_north = self.position != "room"

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
            depth_cam = None
        else:
            handler = blackboard.get("image_handler_north")
            offset_y_m = config.camera_north_offset_y
            offset_z_m = config.camera_north_offset_z
            hfov = config.camera_north_hfov
            vfov = config.camera_north_vfov
            pass_sign = 1.0
            depth_cam = blackboard.get("depth_cam")
            if depth_cam is None and isinstance(handler.camera, DepthCam):
                depth_cam = handler.camera

        handler.image_processing_callback = blackboard.get("callback_gate")

        lost_tolerance = config.obstacle_gate_lost_tolerance
        aligned_tolerance_m = config.obstacle_gate_aligned_tolerance_m
        aligned_threshold = config.obstacle_gate_aligned_threshold
        aim_down_m = config.obstacle_gate_center_z_down_m
        class_name = config.model_gate_class_name
        gate_limits = (config.obstacle_gate_output_min, config.obstacle_gate_output_max)
        gate_deadband = config.obstacle_gate_output_deadband

        pid_y = None
        pid_z = None
        pid_alt = None
        hold_alt = None
        lost = 0
        aligned = 0
        phase = "align"
        close_enough = False
        z_smooth = None
        width = height = 0
        cx_sp = cy_sp = 0.0
        fx = fy = 0.0

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
                    kp=config.obstacle_gate_z_kp,
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
                    offset_z_m - aim_down_m, standoff, fy
                )
                yasmin.YASMIN_LOG_INFO(
                    f"Gate {self.position}: image {width}x{height}, "
                    f"fx={fx:.0f} fy={fy:.0f} "
                    f"offset y={offset_y_m:.3f} z={offset_z_m:.3f} m "
                    f"aim_down={aim_down_m:.3f} m "
                    f"standoff={standoff:.3f} m pass_sign={pass_sign:+.0f}."
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
                draw_align_markers(frame, cx_sp, cy_sp, None)
                draw_hud(
                    frame,
                    [
                        f"lost {lost}/{lost_tolerance}  Z {fmt_z(z_smooth)}",
                    ],
                )
                save_jpg("gate_annotated", "annotated", frame)
                drone.move_velocity()
                if lost >= lost_tolerance:
                    if (
                        phase == "creep"
                        and close_enough
                        and not config.obstacle_gate_align_only
                    ):
                        return self._commit(
                            drone,
                            self._commit_x(pass_sign, z_smooth, commit_extra),
                            "lost",
                        )
                    yasmin.YASMIN_LOG_WARN(
                        f"Gate {self.position}: lost {lost} frames, reacquire."
                    )
                    return "reacquire"
                continue

            lost = 0
            det = max(window, key=lambda d: d.area)
            cx, cy = det.center
            xyxy = det.xyxy
            clipped = bbox_clipped(xyxy, (height, width))
            w_px = float(xyxy[2] - xyxy[0])
            z_box = z_from_bbox(w_px, fx, gate_width)

            depth_color = None
            z_depth = None
            if is_north and depth_cam is not None:
                try:
                    raw = depth_cam.get_depth_frame()
                except Exception:
                    raw = None
                depth_color = depth_on_color(raw, (height, width))
                z_depth = z_from_frame(depth_color, xyxy, z_min, z_max, expand)

            z = fuse_z(z_box, z_depth, fuse_delta)
            z_smooth = smooth_z(z_smooth, z)

            range_m = z_smooth if z_smooth is not None else standoff
            cx_sp = width / 2 + ImageCalculus.meters_to_pixels(
                offset_y_m, range_m, fx
            )
            cy_sp = height / 2 + ImageCalculus.meters_to_pixels(
                offset_z_m - aim_down_m, range_m, fy
            )
            error_y = cx_sp - cx
            error_z = cy_sp - cy
            err_y_m = px_to_m(error_y, range_m, fx)
            err_z_m = px_to_m(error_z, range_m, fy)
            output_y = pass_sign * pid_y.update(-err_y_m)
            y_centered = abs(err_y_m) < aligned_tolerance_m
            centered = y_centered and abs(err_z_m) < aligned_tolerance_m
            agl = drone.get_altitude()
            if phase == "align":
                output_z = pid_z.update(-err_z_m)
            elif pid_alt is not None and agl is not None:
                output_z = pid_alt.update(agl)
            else:
                output_z = 0.0

            too_big = w_px >= bbox_frac * width
            at_standoff = z_smooth is not None and z_smooth <= standoff
            should_stop = too_big or at_standoff
            hud_ctr = centered if phase == "align" else y_centered

            if phase == "align":
                if centered:
                    aligned += 1
                else:
                    aligned = 0

            frame = _frame(result)
            overlay_gate(frame, depth_color, xyxy, z_min, z_max, expand)
            draw_align_markers(frame, cx_sp, cy_sp, (cx, cy))
            if phase == "align":
                hud_z = f"z={error_z:+.0f}px {err_z_m:+.2f}m"
            else:
                hud_z = (
                    f"zbbox={error_z:+.0f}px {err_z_m:+.2f}m ign "
                    f"hold={fmt_z(hold_alt)} agl={fmt_z(agl)}"
                )
            draw_hud(
                frame,
                [
                    f"{phase} {aligned}/{aligned_threshold}  "
                    f"y={error_y:+.0f}px {err_y_m:+.2f}m  "
                    f"{hud_z}  "
                    f"v {output_y:+.2f} {output_z:+.2f}",
                    f"Zb {fmt_z(z_box)} Zd {fmt_z(z_depth)} Z {fmt_z(z_smooth)}  "
                    f"clip={int(clipped)} big={int(too_big)} stop={int(at_standoff)} "
                    f"ctr={int(hud_ctr)}",
                ],
            )
            save_jpg("gate_annotated", "annotated", frame)

            if phase == "align":
                yasmin.YASMIN_LOG_INFO(
                    f"Gate {self.position} align ({aligned}/{aligned_threshold}) "
                    f"| error y={error_y:.0f}px {err_y_m:+.3f}m "
                    f"z={error_z:.0f}px {err_z_m:+.3f}m "
                    f"| Zb={fmt_z(z_box)} Zd={fmt_z(z_depth)} Z={fmt_z(z_smooth)} "
                    f"| vel y={output_y:.2f} z={output_z:.2f} m/s"
                )
                drone.move_velocity(vy=output_y, vz=output_z)
                if aligned >= aligned_threshold:
                    if config.obstacle_gate_align_only:
                        continue
                    hold_alt = agl if agl is not None else config.obstacle_gate_alt
                    pid_alt = PIDController(
                        kp=config.obstacle_alt_kp,
                        kd=0.0,
                        ki=0.0,
                        setpoint=hold_alt,
                        output_limits=gate_limits,
                    )
                    phase = "creep"
                    yasmin.YASMIN_LOG_INFO(
                        f"Gate {self.position} aligned. "
                        f"Creep vx={pass_sign * creep_vx:.2f} m/s "
                        f"hold AGL={hold_alt:.2f} m."
                    )
                continue

            if should_stop:
                close_enough = True
            hold = config.obstacle_gate_align_only and close_enough
            vx = 0.0
            if not hold and not close_enough and y_centered:
                vx = pass_sign * creep_vx
            yasmin.YASMIN_LOG_INFO(
                f"Gate {self.position} {'hold' if hold else 'creep'} "
                f"| hold={fmt_z(hold_alt)} agl={fmt_z(agl)} "
                f"| error y={error_y:.0f}px {err_y_m:+.3f}m "
                f"zbbox={error_z:.0f}px {err_z_m:+.3f}m ignored "
                f"| vel x={vx:.2f} y={output_y:.2f} z={output_z:.2f} m/s "
                f"| Zb={fmt_z(z_box)} Zd={fmt_z(z_depth)} Z={fmt_z(z_smooth)} "
                f"| clip={int(clipped)} ctr={int(y_centered)}"
            )
            drone.move_velocity(vx=vx, vy=output_y, vz=output_z)
            if close_enough and not config.obstacle_gate_align_only:
                if self.position == "room":
                    config.inspect_gate_alignment_alt = drone.get_altitude()
                    yasmin.YASMIN_LOG_INFO(f'Saved alignment alt: {config.inspect_gate_alignment_alt}')
                return self._commit(
                    drone,
                    self._commit_x(pass_sign, z_smooth, commit_extra),
                    "standoff",
                )

    @staticmethod
    def _commit_x(pass_sign: float, z_smooth: float | None, extra: float) -> float:
        remaining = _COMMIT_MIN if z_smooth is None else z_smooth + extra
        return pass_sign * max(_COMMIT_MIN, min(remaining, _COMMIT_MAX))

    def _commit(self, drone: MavlinkDrone, commit_x: float, reason: str):
        yasmin.YASMIN_LOG_INFO(
            f"Gate {self.position} commit ({reason}): "
            f"{commit_x:+.1f} m body-x."
        )
        drone.move_velocity()
        drone.move_to(x=commit_x, y=0.0, z=0.0, yaw=0.0, precision=0.12)
        return SUCCEED

    def check_timeout(self):
        now = self.node.get_clock().now()

        return now - self.start_time > Duration(
            seconds=self.timeout
        ) or now - self.start_mission > Duration(seconds=self.mission_timeout)
