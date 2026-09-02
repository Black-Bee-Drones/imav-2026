from argparse import Namespace
from dataclasses import dataclass, field, replace
import pathlib
from typing import Optional

from ament_index_python.packages import get_package_share_directory


def _models_path() -> pathlib.Path:
    return pathlib.Path(get_package_share_directory('indoor')) / 'models'


@dataclass
class Config:
    """Plain dataclass holding every mission parameter. Presets (in
    presets.py) are subclasses that override just the fields they care
    about - everything else falls back to the defaults declared here."""

    ### Global ###
    timeout: int = 360          # seconds
    safe_alt: float = 3.0        # meters
    max_alt: float = 7.0         # meters

    ### Initialize ###
    # Drone
    drone_type: str = 'mavlink'
    drone_connection_string: str = 'udp:127.0.0.1:14551'

    # Detector - gate
    model_gate_source: str = 'gate2.pt'
    model_gate_conf: float = 0.5
    model_gate_class_name: str = 'blue'

    # Detector - baby
    model_baby_source: str = 'yolo26n.pt'
    model_baby_conf: float = 0.25
    model_baby_overlap_iou: float = 0.3   # IoU threshold
    model_baby_classes_names: list = field(default_factory=lambda: ['person', 'teddy bear'])
    model_baby_sample_count: int = 15

    # Detector - box
    model_box_source: str = 'package.pt'
    model_box_conf: float = 0.5

    # Aruco
    aruco_marker_dict: int = 5    # 5x5
    aruco_size: float = 1.0        # meters

    # ImageHandler - north (D435i)
    camera_north_source: str = 'ros'
    camera_north_topic: str = '/camera/color/image_raw/compressed'
    camera_north_is_compressed: bool = True
    camera_north_id: int = 2

    # ImageHandler - south (C920)
    camera_south_source: str = 'opencv'
    camera_south_topic: str = '/camera/color/image_raw/compressed'
    camera_south_is_compressed: bool = True
    camera_south_id: int = 2

    # ImageHandler - down
    camera_down_source: str = 'opencv'
    camera_down_topic: str = '/down_camera'
    camera_down_is_compressed: bool = True
    camera_down_id: int = 0

    ### Takeoff ###
    takeoff_alt: float = 1.2   # meters

    ### Land ###
    rtl: bool = False

    ### Obstacle ###
    obstacle_skip: bool = False
    obstacle_timeout: int = 300   # seconds

    obstacle_xy_kp: float = 0.005
    obstacle_xy_kd: float = 0.0
    obstacle_xy_ki: float = 0.0

    obstacle_z_kp: float = 0.003
    obstacle_z_kd: float = 0.0
    obstacle_z_ki: float = 0.0

    # GoToObstacle
    obstacle_start_x: float = 0.0     # meters
    obstacle_start_y: float = -3.5    # meters

    # Window
    obstacle_gate_first_skip: bool = False
    obstacle_gate_second_skip: bool = False
    obstacle_gate_room_skip: bool = False
    obstacle_gate_alt: float = 1.2
    obstacle_gate_lost_tolerance: int = 15
    obstacle_gate_aligned_tolerance: int = 15
    obstacle_gate_aligned_threshold: int = 50   # pixels

    # RedBar
    obstacle_red: Optional[int] = 3   # step or jump
    obstacle_red_step_alt_1: float = 1.7
    obstacle_red_step_alt_2: float = 2.1
    obstacle_red_step_alt_3: float = 2.5

    # BlueBar
    obstacle_blue_1: Optional[int] = 3   # step or jump
    obstacle_blue_2: Optional[int] = 3   # step or jump
    obstacle_blue_step_alt_1: float = 0.2
    obstacle_blue_step_alt_2: float = 0.4
    obstacle_blue_step_alt_3: float = 0.6

    # Tubes
    obstacle_tubes_skip: bool = False
    obstacle_tubes_alt: float = 1.2
    obstacle_tubes_offset: float = 1.5   # meters

    ### Inspect ###
    inspect_skip: bool = False
    inspect_timeout: int = 300   # seconds

    # GoToWindow
    inspect_start_x: float = 10.0   # meters
    inspect_start_y: float = 0.0     # meters
    inspect_start_z: float = 1.70    # meters

    # FindWindow
    inspect_back_speed: float = -0.5

    # Count Babies
    inspect_babies_count: Optional[int] = None

    # GoOut
    inspect_go_out_x: float = 1.5

    ### Dropping ###
    drop_cone_enabled: bool = True
    droping_skip: bool = False

    ### Precise landing ###
    precise_skip: bool = False
    precise_timeout: int = 300   # seconds

    # GoToLandingBase
    precise_fixed: bool = True
    precise_fixed_x: float = 0.0
    precise_fixed_y: float = 2.0
    precise_mobile_x: float = 0.0
    precise_mobile_y: float = -2.0

    # Center
    center_threshold_xy: float = 0.2     # meters
    center_threshold_z: float = 0.2      # meters
    center_threshold_yaw: float = 5.0    # degrees
    lost_tolerance: int = 10
    land_altitude: float = 1.0           # meters

    # Reacquire
    precise_reacquire_vz: float = 0.3

    ### PIDController ###
    # PID xy
    controller_xy_kp: float = 0.000511
    controller_xy_kd: float = 0.0
    controller_xy_ki: float = 0.0
    controller_xy_output_min: float = -0.1
    controller_xy_output_max: float = 0.1
    controller_xy_integral_min: float = -0.1
    controller_xy_integral_max: float = 0.1

    # PID z
    controller_z_kp: float = 0.000711
    controller_z_kd: float = 0.0
    controller_z_ki: float = 0.0
    controller_z_output_min: float = -0.1
    controller_z_output_max: float = 0.1
    controller_z_integral_min: float = -0.1
    controller_z_integral_max: float = 0.1

    # PID yaw
    controller_yaw_kp: float = 0.0000111
    controller_yaw_kd: float = 0.0
    controller_yaw_ki: float = 0
    controller_yaw_output_min: float = -0.1
    controller_yaw_output_max: float = 0.1
    controller_yaw_integral_min: float = -0.1
    controller_yaw_integral_max: float = 0.1

    ### Runtime-only fields ###
    # Not real config - populated/consumed during a mission run. Kept as
    # dataclass fields (rather than set in __post_init__) so `replace()`
    # and preset subclassing keep working uniformly for every field.
    inspect_babies_output_path: pathlib.Path = field(
        default_factory=lambda: pathlib.Path.home() / 'ros2_ws')
    obstacle_start_time: object = None
    inspect_start_time: object = None
    precise_start_time: object = None
    inspect_babies_boxes: Optional[list] = None

    def __post_init__(self):
        # model_*_source fields hold bare filenames in every preset/default
        # (e.g. "gate2.pt"); resolve them once, here, into full paths under
        # the package's models/ dir so every state just reads
        # config.model_gate_source etc. as a ready-to-use path.
        models_path = _models_path()
        for attr in ('model_gate_source', 'model_baby_source', 'model_box_source'):
            value = getattr(self, attr)
            if value and not pathlib.Path(value).is_absolute():
                setattr(self, attr, str(models_path / value))

    @classmethod
    def list_preset(cls) -> list:
        from indoor import presets
        return presets.list_presets()


    def apply_args(self, args: Namespace) -> 'Config':
        """Returns a new Config (possibly a different subclass, if a
        preset was requested) with CLI overrides applied. Does not mutate
        self, since presets are chosen by picking a different dataclass."""
        config = self

        if args.preset is not None:
            if args.preset == 'custom':
                from indoor.customization import run_customization_wizard
                overrides = run_customization_wizard()
                config = replace(config, **overrides)
            else:
                from indoor import presets
                preset_cls = presets.get_preset(args.preset)
                config = preset_cls()

        if args.sitl:
            config = replace(
                config,
                drone_connection_string='tcp:127.0.0.1:5762',
                camera_north_source='ros',
                camera_north_topic='/north_camera/image',
                camera_north_is_compressed=False,
                camera_down_source='ros',
                camera_down_topic='/down_camera',
                camera_down_is_compressed=False,
            )

        if args.obstacle_skip:
            config = replace(config, obstacle_skip=args.obstacle_skip)
        if args.inspect_skip:
            config = replace(config, inspect_skip=args.inspect_skip)
        if args.droping_skip:
            config = replace(config, droping_skip=args.droping_skip)
        if args.precise_skip:
            config = replace(config, precise_skip=args.precise_skip)

        return config