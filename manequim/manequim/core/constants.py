import os
from math import radians
from pathlib import Path


# ── Simulation ───────────────────────────────────────────────────────────────
SIM_MODE = True


# ── Flight Configuration ─────────────────────────────────────────────────────
RTL_ALTITUDE   = 2.0   # Return-to-launch altitude (meters)
TAKEOFF_HEIGHT = 2.0   # Default take-off height   (meters)
MAX_ALTITUDE_REFERENCE_LOSS = 10 # Total of tries drone can take to verify altitude source


# ── Image Handler ────────────────────────────────────────────────────────────
IMAGE_COMPRESSED = False

# Image source per mode:
#   SIM  → "/down_camera"                   (tópico ROS do Gazebo)
#   REAL → "webcam"                         (câmera onboard via OpenCV)
#          "/mavros/camera/image_captured"  (tópico ROS do drone real)
SIM_IMAGE_SOURCE  = "/down_camera"
REAL_IMAGE_SOURCE = "webcam"

IMAGE_SOURCE = SIM_IMAGE_SOURCE if SIM_MODE else REAL_IMAGE_SOURCE

# ── Drone Configuration ─────────────────────────────────────────────────────
CONNECTION_STRING = "/dev/ttyAMA1"
DRONE_TYPE = "mavlink" if not SIM_MODE else "mavlink"

# ── Detector ─────────────────────────────────────────────────────────────────
if SIM_MODE:
    DETECTOR_MODEL_SOURCE = os.environ.get(
        "MANEQUIM_DETECTOR_MODEL",
        str(Path.home() / "ros2_ws" / "yolo26n.pt"),
    )
else:
    DETECTOR_MODEL_SOURCE = "yolo26n.pt"
DETECTOR_CONFIDENCE_THRESHOLD  = 0.5

DETECTOR_CLASS: list[str] = ["person", "kite"] if SIM_MODE else ["person"]

# ── Package delivery configuration ───────────────────────────────────────────
SERVO_CHANNEL: int = 2
SERVO_OPEN_PWM: int = 1600
SERVO_CLOSED_PWM: int = 2200
DROP_MAX_RETRIES: int = 3
RETRY_DELAY: float = 1.0
DROP_HEIGHT: float = 0.55

# ── Package PID gains ─────────────────────────────────────────────────────────
X_KP: float = 0.123
X_KI: float = 0.0
X_KD: float = 0.02

Y_KP: float = 0.123
Y_KI: float = 0.0
Y_KD: float = 0.02

XY_OUTPUT_LIM: tuple[float, float] = (-0.5, 0.5)
XY_INTEGRAL_LIM: tuple[float, float] = (-1.0, 1.0)
XY_OUTPUT_DEADBAND: float = 0.05

CAMERA_SOURCE = "opencv"
CAMERA_MODEL = "C920"  # Opções disponíveis: "C920" ou "IMX"

if CAMERA_MODEL == "C920":
    # Logitech C920 (70.42° H, 43.3° V)
    CAMERA_HFOV = radians(70.42)
    CAMERA_VFOV = radians(43.3)
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 640
    CAMERA_OFFSET_X = 0.0
    CAMERA_OFFSET_Y = 0.0
    CAMERA_OFFSET_Z = 0.0
elif CAMERA_MODEL == "IMX":
    # IMX662 (Arducam datasheet 86° H x 47° V)
    CAMERA_HFOV = radians(86.0)
    CAMERA_VFOV = radians(47.0)
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 640
    CAMERA_OFFSET_X = 0.11
    CAMERA_OFFSET_Y = 0.0
    CAMERA_OFFSET_Z = 0.0
else:
    CAMERA_HFOV = radians(70.42)
    CAMERA_VFOV = radians(43.3)
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    CAMERA_OFFSET_X = 0.0
    CAMERA_OFFSET_Y = 0.0
    CAMERA_OFFSET_Z = 0.0
