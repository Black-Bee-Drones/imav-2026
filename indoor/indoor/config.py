from argparse import Namespace
from dataclasses import dataclass, field, replace
from math import radians
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory


def _models_path() -> Path:
    return Path(get_package_share_directory("indoor")) / "models"


def _resolve_model(value: str) -> str:
    if value and not Path(value).is_absolute():
        return str(_models_path() / value)
    return value


@dataclass
class Config:
    """Mission parameters. Presets subclass this and override fields."""

    sitl: bool = False
    timeout: int = 360
    safe_alt: float = 3.0
    max_alt: float = 7.0

    drone_type: str = "mavlink"
    drone_connection_string: str = "udp:127.0.0.1:14551"

    model_gate_source: str = "gate-mangalarga.pt"
    model_gate_conf: float = 0.5
    model_gate_class_name: str = "blue"

    model_baby_source: str = "yolo26n.pt"
    model_baby_conf: float = 0.25
    model_baby_overlap_iou: float = 0.3
    model_baby_classes_names: list = field(
        default_factory=lambda: ["person", "teddy bear"]
    )
    model_baby_sample_count: int = 15

    model_box_source: str = "package.pt"
    model_box_conf: float = 0.5

    model_lines_source: str = "lines.pt"
    model_lines_conf: float = 0.25
    model_lines_red_class_name: str = "red"
    model_lines_blue_class_name: str = "blue"

    aruco_marker_dict: int = 5
    aruco_size: float = 1.0
    color_calibration_path: Optional[str] = None

    # North: RealSense D435i RGB (Intel 69.4° x 42.5° x 77° ±3°)
    camera_north_source: str = "ros_depth"
    camera_north_topic: str = "/camera/color/image_raw/compressed"
    camera_north_is_compressed: bool = True
    camera_north_id: int = 2
    camera_north_depth_topic: str = "/camera/aligned_depth_to_color/image_raw"
    camera_north_depth_topic_fallback: str = "/camera/depth/image_rect_raw"
    camera_north_offset_z: float = 0.11
    camera_north_offset_y: float = 0.0
    camera_north_hfov: float = radians(69.4)
    camera_north_vfov: float = radians(42.5)

    # South: Logitech C920, looking aft (70.42° H, 43.3° V, 78° diag)
    camera_south_source: str = "opencv"
    camera_south_topic: str = "/south_camera"
    camera_south_is_compressed: bool = True
    camera_south_id: int = 10
    camera_south_offset_z: float = 0.0
    camera_south_offset_y: float = 0.0
    camera_south_hfov: float = radians(70.42)
    camera_south_vfov: float = radians(43.3)

    # Down: IMX662 (Arducam datasheet 86° H x 47° V)
    camera_down_source: str = "opencv"
    camera_down_topic: str = "/down_camera"
    camera_down_is_compressed: bool = True
    camera_down_id: int = 6
    camera_down_offset_x: float = 0.11
    camera_down_offset_y: float = 0.0
    camera_down_hfov: float = radians(86)
    camera_down_vfov: float = radians(47)

    takeoff_alt: float = 1.1
    skip_takeoff: bool = False
    rtl: bool = False

    obstacle_skip: bool = False
    obstacle_timeout: int = 300
    obstacle_xy_kp: float = 0.00069
    obstacle_xy_kd: float = 0.0
    obstacle_xy_ki: float = 0.0
    obstacle_alt_kp: float = 0.4

    obstacle_start_x: float = -0.48
    obstacle_start_y: float = -2.5

    obstacle_gate_first_skip: bool = False
    obstacle_gate_second_skip: bool = False
    obstacle_gate_room_skip: bool = False
    obstacle_after_first_skip: bool = False
    obstacle_gate_align_only: bool = False
    obstacle_gate_alt: float = 1.1
    obstacle_gate_lost_tolerance: int = 15
    obstacle_gate_aligned_tolerance: int = 35
    obstacle_gate_aligned_threshold: int = 10
    obstacle_gate_aligned_tolerance_m: float = 0.05
    obstacle_gate_center_z_down_m: float = 0.121
    obstacle_gate_kp: float = 0.5874
    obstacle_gate_z_kp: float = 0.731
    obstacle_gate_kd: float = 0.0
    obstacle_gate_ki: float = 0.0
    obstacle_gate_output_min: float = -0.18
    obstacle_gate_output_max: float = 0.18
    obstacle_gate_output_deadband: float = 0.03
    obstacle_gate_pass_x: float = 1.5
    obstacle_gate_creep_vx: float = 0.071
    obstacle_gate_standoff: float = 0.79
    obstacle_gate_width: float = 0.60
    obstacle_gate_height: float = 0.50
    obstacle_gate_bbox_frac: float = 0.75
    obstacle_gate_depth_min: float = 0.45
    obstacle_gate_depth_max: float = 2.2
    obstacle_gate_frame_expand: float = 0.27
    obstacle_gate_fuse_delta: float = 0.4
    obstacle_gate_commit_extra: float = 0.65

    obstacle_red: Optional[int] = 3
    obstacle_red_step_alt_1: float = 1.7
    obstacle_red_step_alt_2: float = 2.1
    obstacle_red_step_alt_3: float = 2.71

    obstacle_blue_1: Optional[int] = 2
    obstacle_blue_step_alt_1: float = 0.2
    obstacle_blue_step_alt_2: float = 0.22
    obstacle_blue_step_alt_3: float = 0.6
    obstacle_bar_red_color: str = "red"
    obstacle_bar_blue_color: str = "blue"
    obstacle_bar_center_skip: bool = True
    obstacle_bar_center_tolerance: float = 20.0
    obstacle_bar_acquire_timeout: float = 30.0
    obstacle_bar_find_vx: float = 0.15
    obstacle_bar_pass_x: float = 1.8
    obstacle_bar_approach_x: float = 1.0
    obstacle_bar_gap: float = 1.0
    obstacle_bar_roi_w: int = 640
    obstacle_bar_roi_h: int = 480

    obstacle_tubes_skip: bool = False
    obstacle_tubes_alt: float = 1.2
    obstacle_tubes_offset: float = 1.5
    obstacle_tubes_end_offset: float = 0.5
    obstacle_tubes_standoff: float = 0.7

    inspect_skip: bool = False
    inspect_timeout: int = 300
    inspect_start_x: float = 10.0
    inspect_start_y: float = 0.0
    inspect_start_z: float = 1.70
    inspect_back_speed: float = -0.5
    inspect_babies_count: Optional[int] = None
    inspect_go_out_x: float = 1.5

    droping_skip: bool = False

    precise_skip: bool = False
    precise_timeout: int = 300
    precise_fixed: bool = True
    precise_fixed_x: float = 0.0
    precise_fixed_y: float = 2.0
    precise_mobile_x: float = 0.0
    precise_mobile_y: float = -2.0
    center_threshold_xy: float = 0.2
    center_threshold_z: float = 0.2
    center_threshold_yaw: float = 5.0
    lost_tolerance: int = 10
    land_altitude: float = 1.0
    precise_reacquire_vz: float = 0.3

    controller_xy_kp: float = 0.000511
    controller_xy_kd: float = 0.0
    controller_xy_ki: float = 0.0
    controller_xy_output_min: float = -0.1
    controller_xy_output_max: float = 0.1
    controller_xy_integral_min: float = -0.1
    controller_xy_integral_max: float = 0.1
    controller_yaw_kp: float = 0.0000111
    controller_yaw_kd: float = 0.0
    controller_yaw_ki: float = 0
    controller_yaw_output_min: float = -0.1
    controller_yaw_output_max: float = 0.1
    controller_yaw_integral_min: float = -0.1
    controller_yaw_integral_max: float = 0.1

    inspect_babies_output_path: Path = field(
        default_factory=lambda: Path.home() / "ros2_ws"
    )
    obstacle_start_time: object = None
    inspect_start_time: object = None
    precise_start_time: object = None

    def __post_init__(self):
        self.model_gate_source = _resolve_model(self.model_gate_source)
        self.model_baby_source = _resolve_model(self.model_baby_source)
        self.model_box_source = _resolve_model(self.model_box_source)
        self.model_lines_source = _resolve_model(self.model_lines_source)

    @classmethod
    def list_preset(cls) -> list:
        from indoor import presets

        return presets.list_presets()

    def apply_args(self, args: Namespace) -> "Config":
        config = self

        if args.preset is not None:
            if args.preset == "custom":
                from indoor.customization import run_customization_wizard

                config = replace(config, **run_customization_wizard())
            else:
                from indoor import presets

                config = presets.get_preset(args.preset)()

        if args.sitl:
            calib = None
            try:
                calib = str(
                    Path(get_package_share_directory("indoor"))
                    / "config"
                    / "sitl_color_calibration.json"
                )
            except Exception:
                calib = None
            config = replace(
                config,
                sitl=True,
                drone_connection_string="tcp:127.0.0.1:5762",
                camera_north_source="ros_depth",
                camera_north_topic="/front_camera/image",
                camera_north_is_compressed=False,
                camera_north_depth_topic="/front_camera/depth_image",
                camera_north_offset_z=-0.02,
                camera_north_offset_y=0.0,
                camera_south_source="skip",
                camera_down_source="ros",
                camera_down_topic="/down_camera",
                camera_down_is_compressed=False,
                camera_down_offset_x=0.0,
                camera_down_offset_y=0.0,
                model_gate_conf=0.2,
                color_calibration_path=calib,
            )

        if args.obstacle_skip:
            config = replace(config, obstacle_skip=True)
        if args.inspect_skip:
            config = replace(config, inspect_skip=True)
        if args.droping_skip:
            config = replace(config, droping_skip=True)
        if args.precise_skip:
            config = replace(config, precise_skip=True)
        if getattr(args, "no_takeoff", False):
            config = replace(config, skip_takeoff=True)

        return config
