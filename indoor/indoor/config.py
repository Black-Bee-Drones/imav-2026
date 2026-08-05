import pathlib
from dataclasses import dataclass
from enum import Enum

from ament_index_python import get_package_share_directory

models_path = pathlib.Path(get_package_share_directory('indoor')) / 'models'


class Mission(str, Enum):
    OBSTACLES = 'OBSTACLESM'
    INSPECT = 'INSPECTSM'
    DROPPING = 'DROPPINGSM'


class LandingMode(str, Enum):
    LAND = 'LAND'                        
    RTL = 'RTL'                          
    PRECISION_FIXED = 'PRECISION_FIXEDSM'
    PRECISION_MOVING = 'PRECISION_MOVINGSM'


@dataclass(frozen=True)
class Config:
    ### Missions ###
    missions: tuple[Mission, ...] = (
        Mission.OBSTACLES,
        Mission.INSPECT,
        Mission.DROPPING,
    )

    ### Landing behavior  ###
    landing_mode: LandingMode = LandingMode.LAND

    ### Global ###
    timeout: int = 1800  # seconds
    timeout_per_state: int = 300  # seconds
    safe_altitude: float = 3.0  # meters
    max_altitude: float = 7.0  # meters

    ### Initialize ###
    # Drone
    drone_type: str = 'mavlink'
    connection_string: str = 'udp:127.0.0.1:14551'

    # Detector
    gate_model_source: str = str(models_path / 'gate.pt')
    gate_conf: float = 0.5
    baby_model_source: str = str(models_path / 'best.pt')
    baby_conf: float = 0.5
    box_model_source: str = str(models_path / 'package.pt')
    box_conf: float = 0.5

    # Aruco
    marker_dict: int = 5  # 5x5
    aruco_size: float = 1.0  # meters

    # ImageHandler
    front_image_source: str = 'ros'
    front_ros_topic: str = '/camera/color/image_raw'
    down_image_source: str = 'webcam'
    down_ros_topic: str = '/donw_camera/image'

    ### Takeoff ###
    takeoff_altitude: float = 1.2  # meters

    ### Obstacle ###
    # GoToWindow
    start_y : float = -3.5

    # Window
    first_color_window: str | None = 'blue'  # 'blue' or 'red' or None
    second_color_window: str | None = 'blue'  # 'blue' or 'red' or None
    room_color_window: str | None = 'blue'  # 'blue' or 'red' or None
    window_threshold: int = 50  # pixels

    # RedBar
    red_step: int | None = 3

    # BlueBar
    blue_step1: int | None = 3
    blue_step2: int | None = 3

    # Obstacle (tube passage) - set False to skip this checker entirely
    obstacle_avoid_enabled: bool = True

    ### Inspect ###
    # GoToWindow
    room_x: float = 10.0  # meters
    room_y: float = 2.0  # meters

    room_entry_color: str | None = 'blue'  # 'blue' or 'red' or None
    room_exit_color: str | None = 'blue'   # 'blue' or 'red' or None

    run_baby_inference: bool = True

    # FindWindow
    find_tolerance: int = 2
    back_speed: float = -0.5  # meters per second

    ### Dropping ###
    drop_cone_enabled: bool = True

    ### Precise landing ###
    # GoToLandingBase
    fixed_base: bool = True
    fixed_base_x: float = 0.0  # meters
    fixed_base_y: float = 2.0  # meters
    mobile_base_x: float = 0.0  # meters
    mobile_base_y: float = -2.0  # meters

    # Center
    center_threshold_xy: float = 0.2  # meters
    center_threshold_z: float = 0.2  # meters
    center_threshold_yaw: float = 5.0  # degrees
    lost_tolerance: int = 10
    land_altitude: float = 1.0  # meters

    # Reacquire
    reacquire_step: float = 0.5  # meters

    ### PIDController ###
    # PID xy
    controller_xy_kp: float = 1.0
    controller_xy_kd: float = 1.0
    controller_xy_ki: float = 1.0
    controller_xy_output_min: float = -1.0
    controller_xy_output_max: float = 1.0
    controller_xy_integral_min: float = -1.0
    controller_xy_integral_max: float = 1.0

    # PID z
    controller_z_kp: float = 1.0
    controller_z_kd: float = 1.0
    controller_z_ki: float = 1.0
    controller_z_output_min: float = -1.0
    controller_z_output_max: float = 1.0
    controller_z_integral_min: float = -1.0
    controller_z_integral_max: float = 1.0

    # PID yaw
    controller_yaw_kp: float = 1.0
    controller_yaw_kd: float = 1.0
    controller_yaw_ki: float = 0
    controller_yaw_output_min: float = -1.0
    controller_yaw_output_max: float = 1.0
    controller_yaw_integral_min: float = -1.0
    controller_yaw_integral_max: float = 1.0


@dataclass(frozen=True)
class SITLConfig(Config):
    connection_string: str = 'tcp:127.0.0.1:5762'


