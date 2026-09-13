from rclpy.time import Time, Duration

import numpy as np
import math

import yasmin
from yasmin import State, Blackboard, YASMIN_LOG_INFO
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT

from nectar.control import MavlinkDrone, MavrosDrone, PIDController, MoveReference, NavigationMethod, AltitudeSource
from nectar.vision import ImageHandler, ImageCalculus
from nectar.ai import DetectionResult
from .map_boxes import pixel_to_takeoff_frame   

from ...config import Config

Point2D = tuple[float, float]       # (x, y)
Pose2D = tuple[float, float, float] # (x, y, yaw)

class CenterBox(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, FAIL, TIMEOUT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavlinkDrone | MavrosDrone = blackboard.get('drone')
        config: Config = blackboard.get('config')
        
        camera_down: ImageHandler = blackboard.get('image_handler_down')
        camera_down.image_processing_callback = blackboard.get('callback_box')
        
        target_x, target_y = blackboard["target_coords"]

        pid_cx: PIDController = PIDController(
            kp=config.dropping_box_kp,
            kd=config.dropping_box_kd,
            ki=config.dropping_box_ki,
            setpoint=0.0,
            output_limits=config.dropping_box_limits,
            output_deadband=config.dropping_box_deadband
        )
        
        pid_cy: PIDController = PIDController(
            kp=config.dropping_box_kp,
            kd=config.dropping_box_kd,
            ki=config.dropping_box_ki,
            setpoint=0.0,
            output_limits=config.dropping_box_limits,
            output_deadband=config.dropping_box_deadband,
        )
        
        pid_cz: PIDController = PIDController(
            kp=config.dropping_box_kp_z,
            kd=0.0,
            ki=0.0,
            setpoint=0.0,
            output_limits=config.dropping_box_limits_z,
        )
        
        self.mission_start_time: Time = blackboard.get('start_time')
        self.state_start_time = self.node.get_clock().now()

        yasmin.YASMIN_LOG_INFO(
            f"Navigating to box approx position ({target_x:.2f}, {target_y:.2f})"
        )
        drone.move_to(
            x=target_x,
            y=target_y,
            z=config.safe_alt,
            yaw=0,
            reference=MoveReference.TAKEOFF,
            precision=0.12,
        )

        # ---------- FASE 2: Centralização fina via visão (pinhole model) ----------
        pid_cx.reset()
        pid_cy.reset()
        pid_cx.set_setpoint(0.0)
        pid_cy.set_setpoint(0.0)
        pid_cz.set_setpoint(0.0)

        width, height = config.camera_down_frame
        lost = 0
        aligned_frames = 0

        while True:
            if self.timeout(config):
                yasmin.YASMIN_LOG_ERROR("Centering timed out.")
                return TIMEOUT

            result = camera_down.take_photo()
            if result is None:
                yasmin.YASMIN_LOG_WARN("Failed to get frame from camera, skipping cycle")
                continue

            detections = result.filter_by_class([config.model_dropping_box_name])

            if not detections:
                lost += 1
                aligned_frames = 0
                yasmin.YASMIN_LOG_WARN(f"Not detected ({lost}/{config.dropping_lost_tolerance}) Holding position...")
                drone.move_velocity(0.0, 0.0, 0.0)

                if lost >= config.dropping_lost_tolerance:
                    yasmin.YASMIN_LOG_WARN("Detection lost. Increasing altitude to restart search...")
                    if drone.get_altitude(AltitudeSource.LIDAR) < config.max_altitude:
                        drone.move_to(z=0.2)
                    else:
                        yasmin.YASMIN_LOG_WARN("Detection lost. Max altitude reached, decreasing 30 cm...")
                        drone.move_to(z=-0.3)
                    pid_cx.reset()
                    pid_cy.reset()
                    pid_cz.reset()
                    yasmin.YASMIN_LOG_INFO("Restarting box detection")
                    lost = 0
                    aligned_frames = 0
                continue

            lost = 0
            best_det = max(detections, key=lambda d: d.confidence)
            target_x, target_y = best_det.center

            error_x_px = target_x - width // 2
            error_y_px = target_y - height // 2
            altitude = drone.get_altitude(AltitudeSource.LIDAR)

            error_x = self.ppm(error_x_px, altitude, config.camera_down_hfov, width)
            error_y = self.ppm(error_y_px, altitude, config.camera_down_vfov, height) + config.dropping_cone_offset
            error_z = altitude - config.dropping_center_drop_altitude

            # funil: pixel libera o Z (mais frouxo em altitude alta), metro decide o alinhamento final
            px_aligned = max(abs(error_x_px), abs(error_y_px)) <= config.dropping_centralize_tolerance
            aligned = max(abs(error_x), abs(error_y)) <= config.dropping_center_drop_tolerance

            vx = pid_cy.update(error_y)
            vy = pid_cx.update(error_x)
            vz = pid_cz.update(error_z) if px_aligned else 0.0

            if aligned:
                aligned_frames += 1
                yasmin.YASMIN_LOG_INFO(f"Box aligned ({aligned_frames}/{config.dropping_required_frames})")
            else:
                aligned_frames = 0

            drone.move_velocity(vx, vy, vz)

            if abs(error_z) <= config.dropping_center_drop_altitude_tolerance and aligned_frames >= config.dropping_required_frames:
                drone.move_velocity(0.0, 0.0, 0.0)
                yasmin.YASMIN_LOG_INFO(f"Centering confirmed with {aligned_frames} consecutive aligned frames. Stopping drone before drop.")
                return SUCCEED


    def ppm(self, delta_pixel: int, altitude: float, fov_degrees: float, frame_px: int) -> float:
        angle_rad = math.radians(fov_degrees) / 2
        ratio = (math.tan(angle_rad) * altitude) / (frame_px // 2)
        return delta_pixel * ratio

    def timeout(self, config) -> bool:
        now = self.node.get_clock().now()
        if self.mission_start_time is not None and hasattr(config, "mission_timeout"):
            if now - self.mission_start_time > Duration(seconds=config.mission_timeout):
                return True
        return now - self.state_start_time > Duration(seconds=config.dropping_center_timeout)
