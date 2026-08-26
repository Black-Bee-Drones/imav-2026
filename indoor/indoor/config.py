from argparse import Namespace

from rclpy.time import Time

import yasmin
from yasmin import Blackboard

from indoor import presets


class Config(Blackboard):
    def __init__(self):
        super().__init__()

        ### Global ###
        self.timeout: int = 360  # seconds
        self.safe_alt: float = 3.0  # meters
        self.max_alt: float = 7.0  # meters

        ### Initialize ###
        # Drone
        self.drone_type: str = 'mavlink'
        self.drone_connection_string: str = 'udp:127.0.0.1:14551'

        # Detector
        self.model_gate_source: str = 'gate2.pt'
        self.model_gate_conf: float = 0.5
        self.model_gate_class_name: str = 'blue'

        self.model_baby_source: str = 'yolo26n.pt'
        self.model_baby_conf: float = 0.25
        self.model_baby_overlap_iou: float = 0.3  # IoU threshold
        self.model_baby_classes_names = ['person', 'teddy bear']

        self.model_box_source: str = 'package.pt'
        self.model_box_conf: float = 0.5

        # Aruco
        self.aruco_marker_dict: int = 5  # 5x5
        self.aruco_size: float = 1.0  # meters

        # ImageHandler
        self.camera_front_source: str = 'ros'
        self.camera_front_topic: str = '/camera/color/image_raw/compressed'
        self.camera_front_is_compressed: bool = True
        self.camera_front_id: int = 6
        self.camera_down_source: str = 'opencv'
        self.camera_down_topic: str = '/down_camera'
        self.camera_down_is_compressed: bool = True
        self.camera_down_id: int = 0

        ### Takeoff ###
        self.takeoff_alt: float = 1.2  # meters

        ### LAND ###
        self.rtl: bool = False

        ### Obstacle ###
        self.obstacle_skip: bool = False
        self.obstacle_timeout: int = 300  # seconds
        self.obstacle_start_time: Time | None = None

        self.obstacle_xy_kp: float = 0.005
        self.obstacle_xy_kd: float = .0
        self.obstacle_xy_ki: float = .0

        self.obstacle_z_kp: float = .003
        self.obstacle_z_kd: float = .0
        self.obstacle_z_ki: float = .0

        # GoToObstacle
        self.obstacle_start_x: float = 0.0  # meters
        self.obstacle_start_y: float = -3.2  # meters

        # Window
        self.obstacle_gate_first_skip: bool = False
        self.obstacle_gate_second_skip: bool = False
        self.obstacle_gate_room_skip: bool = False
        self.obstacle_gate_alt: float = 1.2
        self.obstacle_gate_lost_tolerance = 15
        self.obstacle_gate_aligned_tolerance = 15
        self.obstacle_gate_aligned_threshold = 50  # pixels

        # RedBar
        self.obstacle_red: int | None = 3  # step or jump
        self.obstacle_red_step_alt_1: float = 1.7
        self.obstacle_red_step_alt_2: float = 2.1
        self.obstacle_red_step_alt_3: float = 2.5

        # BlueBar
        self.obstacle_blue_1: int | None = 3  # step or jump
        self.obstacle_blue_2: int | None = 3  # step or jump
        self.obstacle_blue_step_alt_1: float = 0.2
        self.obstacle_blue_step_alt_2: float = 0.4
        self.obstacle_blue_step_alt_3: float = 0.6

        # Tubes
        self.obstacle_tubes_skip = False
        self.obstacle_tubes_alt = 1.2
        self.obstacle_tubes_offset = 1.5  # meters

        ### Inspect ###
        self.inspect_skip: bool = False
        self.inspect_timeout: int = 300  # seconds
        self.inspect_start_time: Time | None = None

        # GoToWindow
        self.inspect_start_x: float = 9.30  # meters
        self.inspect_start_y: float = 1.40  # meters

        # FindWindow
        self.inspect_back_speed: float = -0.5

        # Count Babies
        self.inspect_babies_count: int | None = None
        self.inspect_babies_boxes: list[list[int]] | None = None

        # GoOut
        self.inspect_go_out_x: float = 1.5

        ### Dropping ###
        self.drop_cone_enabled: bool = True
        self.droping_skip: bool = False

        ### Precise landing ###
        self.precise_skip: bool = False
        self.precise_timeout: int = 300  # seconds
        self.precise_start_time: Time | None = None

        # GoToLandingBase
        self.precise_fixed: bool = True
        self.precise_fixed_x: float = 0.0 
        self.precise_fixed_y: float = 2.0 
        self.precise_mobile_x: float = 0.0 
        self.precise_mobile_y: float = -2.0 

        # Center
        self.center_threshold_xy: float = 0.2  # meters
        self.center_threshold_z: float = 0.2  # meters
        self.center_threshold_yaw: float = 5.0  # degrees
        self.lost_tolerance: int = 10
        self.land_altitude: float = 1.0  # meters

        # Reacquire
        self.precise_reacquire_vz: float = 0.3

        ### PIDController ###
        # PID xy
        self.controller_xy_kp: float = 0.000511
        self.controller_xy_kd: float = 0.0
        self.controller_xy_ki: float = 0.0
        self.controller_xy_output_min: float = -0.1
        self.controller_xy_output_max: float = 0.1
        self.controller_xy_integral_min: float = -0.1
        self.controller_xy_integral_max: float = 0.1

        # PID z
        self.controller_z_kp: float = 0.000711
        self.controller_z_kd: float = 0.0
        self.controller_z_ki: float = 0.0
        self.controller_z_output_min: float = -0.1
        self.controller_z_output_max: float = 0.1
        self.controller_z_integral_min: float = -0.1
        self.controller_z_integral_max: float = 0.1

        # PID yaw
        self.controller_yaw_kp: float = 0.0000111
        self.controller_yaw_kd: float = 0.0
        self.controller_yaw_ki: float = 0
        self.controller_yaw_output_min: float = -0.1
        self.controller_yaw_output_max: float = 0.1
        self.controller_yaw_integral_min: float = -0.1
        self.controller_yaw_integral_max: float = 0.1

    def aplly_preset(self, preset: dict[str, any]):
        for k, v in getattr(presets, preset).items():
            self.set(k, v)

    def apply_args(self, args: Namespace):
        if args.sitl:
            ### Initialize ###
            # Drone
            self.drone_connection_string: str = 'tcp:127.0.0.1:5762'

            # ImageHandler
            self.camera_front_source: str = 'ros'
            self.camera_front_topic: str = '/front_camera/image'
            self.camera_front_is_compressed: bool = False
            self.camera_down_source: str = 'ros'
            self.camera_down_topic: str = '/down_camera'
            self.camera_down_is_compressed: bool = False

        if args.preset is not None:
            if args.preset == 'custom':
                from indoor.customization import run_customization_wizard

                overrides = run_customization_wizard()
                for k, v in overrides.items():
                    self.set(k, v)
            else:
                self.aplly_preset(args.preset)

        if args.obstacle_skip:
            self.obstacle_skip = args.obstacle_skip
        if args.inspect_skip:
            self.inspect_skip = args.inspect_skip
        if args.droping_skip:
            self.droping_skip = args.droping_skip
        if args.precise_skip:
            self.precise_skip = args.precise_skip

    @staticmethod
    def list_preset():
        return [p for p in dir(presets) if p.isupper()] + ['custom']