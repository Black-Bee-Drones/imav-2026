import yasmin
from yasmin_ros.yasmin_node import YasminNode


class Config:
    def __init__(self, node: YasminNode):
        self._node = node

    def declare_parameters(self):
        yasmin.YASMIN_LOG_INFO('Declaring parameters...')

        self._declare_global()
        self._declare_initialize()
        self._declare_takeoff()
        self._declare_obstacle()
        self._declare_inspect()
        self._declare_precise_landing()
        self._declare_dropping()
        self._declare_pid()

        yasmin.YASMIN_LOG_INFO('Parameters successfully declared...')

    def _declare_global(self):
        self._node.declare_parameter('timeout', 1800)  # seconds
        self._node.declare_parameter('timeout_per_state', 300)  # seconds
        self._node.declare_parameter('safe_altitude', 3.0)  # meters
        self._node.declare_parameter('max_altitude', 7.0)  # meters

    def _declare_initialize(self):
        # Drone
        self._node.declare_parameter('drone_type', 'mavlink')
        self._node.declare_parameter('connection_string', 'udp:127.0.0.1:14551')

        # Detector
        self._node.declare_parameter('gate_model_source', 'gate.pt')
        self._node.declare_parameter('gate_conf', 0.5)
        self._node.declare_parameter('baby_model_source', 'best.pt')
        self._node.declare_parameter('baby_conf', 0.5)
        self._node.declare_parameter('box_model_source', 'package.pt')
        self._node.declare_parameter('box_conf', 0.5)

        # Aruco
        self._node.declare_parameter('marker_dict', 5)  # 5x5

        # ImageHandler
        self._node.declare_parameter('front_image_source', 'ros')
        self._node.declare_parameter('front_ros_topic', '/camera/color/image_raw')
        self._node.declare_parameter('down_image_source', 'webcam')
        self._node.declare_parameter('down_ros_topic', '/donw_camera/image')

    def _declare_takeoff(self):
        self._node.declare_parameter('takeoff_altitude', 1.2)  # meters

    def _declare_obstacle(self):
        # Window
        self._node.declare_parameter('window_threshold', 50)  # pixels

    def _declare_inspect(self):
        # GoToWindown
        self._node.declare_parameter('room_x', 10.0)  # meters
        self._node.declare_parameter('room_y', 2.0)  # meters

        # FindWindow
        self._node.declare_parameter('find_tolerance', 2)
        self._node.declare_parameter('back_speed', -0.5)  # meters per second

    def _declare_precise_landing(self):
        # GoToLandingBase
        self._node.declare_parameter('fixed_base_x', 0.0)  # meters
        self._node.declare_parameter('fixed_base_y', 2.0)  # meters
        self._node.declare_parameter('mobile_base_x', 0.0)  # meters
        self._node.declare_parameter('mobile_base_y', -2.0)  # meters

        # Center
        self._node.declare_parameter('center_threshold', 50)  # pixels
        self._node.declare_parameter('lost_tolerance', 10)
        self._node.declare_parameter('land_altitude', 1.0)  # meters
        self._node.declare_parameter('land_speed', -0.5)  # meters per second

        # Reacquire
        self._node.declare_parameter('reacquire_step', 0.5)  # meters

    def _declare_dropping(self):
        # GoToBox
        self._node.declare_parameter('box_x', 8.5)  # meters
        self._node.declare_parameter('box_y', 0.0)  # meters

        # Drop
        self._node.declare_parameter('drop_index', 0)  # PWM port
        self._node.declare_parameter('drop_value', 500)  # PWM value

    def _declare_pid(self):
        # PID xy
        self._node.declare_parameter('controller_xy_kp', 1.0)
        self._node.declare_parameter('controller_xy_kd', 1.0)
        self._node.declare_parameter('controller_xy_ki', 1.0)
        self._node.declare_parameter('controller_xy_output_min', -1.0)
        self._node.declare_parameter('controller_xy_output_max', 1.0)
        self._node.declare_parameter('controller_xy_integral_min', -1.0)
        self._node.declare_parameter('controller_xy_integral_max', 1.0)

        # PID z
        self._node.declare_parameter('controller_z_kp', 1.0)
        self._node.declare_parameter('controller_z_kd', 1.0)
        self._node.declare_parameter('controller_z_ki', 1.0)
        self._node.declare_parameter('controller_z_output_min', -1.0)
        self._node.declare_parameter('controller_z_output_max', 1.0)
        self._node.declare_parameter('controller_z_integral_min', -1.0)
        self._node.declare_parameter('controller_z_integral_max', 1.0)

