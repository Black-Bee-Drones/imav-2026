from indoor import Config
from yasmin import YASMIN_LOG_INFO
import math

from nectar.control import MavlinkDrone, MavrosDrone
from nectar.control.types import MoveReference, AltitudeSource
from nectar.ai import DetectionResult
from nectar.vision import ImageHandler

Point2D = tuple[float, float]        # (x_meters, y_meters)
Pose2D = tuple[float, float, float]  # (x_meters, y_meters, yaw_radians)

def pixel_to_takeoff_frame(
    pixel_point: Point2D,
    altitude: float,
    drone_pose: Pose2D,
    config: Config,
) -> Point2D:
    """
    Projects a 2D image pixel location to absolute (X, Y) coordinates
    in the takeoff reference frame using camera parameters from Config.

    Parameters
    ----------
    pixel_point : Point2D
        Target pixel position (u, v) in the image frame.
    altitude : float
        Current Drone altitude above ground level (AGL) in meters.
    drone_pose : Pose2D
        Current Drone pose as (x_meters, y_meters, yaw_rad) relative to takeoff origin.
    config : Config
        Mission configuration instance containing downward camera FOV settings.

    Returns
    -------
    Point2D
        Absolute (X, Y) coordinates of the target on the ground plane in meters.
    """
    u, v = pixel_point
    width, height = config.camera_down_frame
    x_drone, y_drone, yaw = drone_pose

    hfov_rad = config.camera_down_hfov
    vfov_rad = config.camera_down_vfov

    du_px = u - (width / 2.0)
    dv_px = (height / 2.0) - v

    dx_cam = dv_px * (math.tan(vfov_rad / 2.0) * altitude) / (height / 2.0)
    dy_cam = du_px * (math.tan(hfov_rad / 2.0) * altitude) / (width / 2.0)

    dx_world = dx_cam * math.cos(yaw) - dy_cam * math.sin(yaw)
    dy_world = dx_cam * math.sin(yaw) + dy_cam * math.cos(yaw)

    return (x_drone + dx_world, y_drone + dy_world)

def map_and_choose_box(
    handler: ImageHandler,
    drone: MavlinkDrone | MavrosDrone,
    config: Config,
    drone_pose: Pose2D = (0.0, 0.0, 0.0),
    option: str = "auto",
) -> tuple[Point2D, Point2D]:
    """
    Runs handler callback, sorts detected boxes from left[0] to right[n],
    and transforms selected box centers into takeoff frame coordinates (X, Y) in meters
    using camera parameters from Config.

    Parameters
    ----------
    handler : ImageHandler
        Handler instance providing the '.take_photo' method.
    altitude : float
        Current Drone altitude above ground level in meters.
    config : Config
        Mission configuration instance containing downward camera FOV settings.
    drone_pose : Pose2D, optional
        Current Drone pose as (x, y, yaw_rad) relative to takeoff frame.
        Defaults to (0.0, 0.0, 0.0).
    option : str, optional
        Selects target boxes: 'left', 'middle', 'right', or 'auto'.
        Defaults to 'auto' (selects left box for LED, right box for Cone).

    Returns
    -------
    tuple[Point2D, Point2D]
        Takeoff frame coordinates (X, Y) in meters for LED box and Cone box.

    Raises
    ------
    TypeError
        If callback result is not a DetectionResult instance.
    ValueError
        If fewer than 3 boxes are detected.
    """
    while True:
        result = handler.take_photo()

        if not isinstance(result, DetectionResult):
            raise TypeError(f"Expected DetectionResult, got {type(result).__name__}, try reviewing image_handler_callback")

        boxes = sorted(result.detections, key=lambda det: det.center[0])

        if len(boxes) == 3:
            break
        elif len(boxes) < 3:
            YASMIN_LOG_INFO('Less than three boxes detected, ascending...')
            altitude = drone.get_altitude()
            if altitude is not None and altitude + 0.5 < config.max_alt:
                drone.move_to(z=0.5, reference=MoveReference.BODY)
            else:
                YASMIN_LOG_INFO('Max altitude reached... Returning TIMEOUT')
                return (float('nan'), float('nan')), (float('nan'), float('nan'))
            continue
        else:
            YASMIN_LOG_INFO('More than three boxes detected, retrying detection')
            continue

    box_map = {
        "left": (0, 0),
        "middle": (1, 1),
        "right": (-1, -1),
        "auto": (0, -1),
    }

    if option not in box_map:
        YASMIN_LOG_INFO(f"Unexpected box option '{option}', setting to default: 'auto'")
        option = "auto"

    idx_led, idx_cone = box_map[option]

    altitude_read_counter = 0

    while altitude_read_counter < 10:
        altitude = drone.get_altitude(AltitudeSource.LIDAR)
        if altitude is not None:
            led_pos = pixel_to_takeoff_frame(
                boxes[idx_led].center, altitude, drone_pose, config
            )
            cone_pos = pixel_to_takeoff_frame(
                boxes[idx_cone].center, altitude, drone_pose, config
            )
            return led_pos, cone_pos
        else:
            drone.delay(0.05)
            altitude_read_counter += 1
    return (float('nan'), float('nan')), (float('nan'), float('nan'))
