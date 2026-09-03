# =============================================================================
#  Core Constants
#  General configuration shared across the hang_the_hook system.
# =============================================================================


# ── Simulation ───────────────────────────────────────────────────────────────
SIM_MODE = False


# ── Flight Configuration ─────────────────────────────────────────────────────
RTL_ALTITUDE   = 2.0   # Return-to-launch altitude (meters)
TAKEOFF_HEIGHT = 2.0   # Default take-off height   (meters)
MAX_ALTITUDE_REFERENCE_LOSS = 10 # Total of tries drone can take to verify altitude source
PWM_VALUE_CLOSE = 1000


# ── Image Handler ────────────────────────────────────────────────────────────
IMAGE_WIDTH      = 640
IMAGE_HEIGHT     = 480
IMAGE_COMPRESSED = False

# Image source per mode:
#   SIM  → "/down_camera"                   (tópico ROS do Gazebo)
#   REAL → "webcam"                         (câmera onboard via OpenCV)
#          "/mavros/camera/image_captured"  (tópico ROS do drone real)
SIM_IMAGE_SOURCE  = "/down_camera"
REAL_IMAGE_SOURCE = "webcam"

IMAGE_SOURCE = SIM_IMAGE_SOURCE if SIM_MODE else REAL_IMAGE_SOURCE