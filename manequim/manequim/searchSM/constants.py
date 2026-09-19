import argparse
import os

LATITUDE: float | None = None
LONGITUDE: float | None = None
MANEQUIM_NUMBER: int = 1


def configure_coordinates(latitude: float | None = None, longitude: float | None = None) -> None:
    global LATITUDE, LONGITUDE

    if latitude is None or longitude is None:
        raise ValueError("Latitude and longitude must be provided before starting the search.")

    LATITUDE = float(latitude)
    LONGITUDE = float(longitude)

    if not (-90.0 <= LATITUDE <= 90.0):
        raise ValueError(f"Invalid latitude: {LATITUDE}")

    if not (-180.0 <= LONGITUDE <= 180.0):
        raise ValueError(f"Invalid longitude: {LONGITUDE}")


def configure_manequim_number(manequim_number: int = 1) -> None:
    global MANEQUIM_NUMBER

    if manequim_number not in (1, 2, 3):
        raise ValueError("Manequim number must be between 1 and 3.")

    MANEQUIM_NUMBER = int(manequim_number)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fly the drone to a specific coordinate for manequim search")
    parser.add_argument("--latitude", type=float, default=None, help="Initial Latitude")
    parser.add_argument("--longitude", type=float, default=None, help="Initial Longitude")
    parser.add_argument(
        "--manequim-number",
        type=int,
        default=1,
        choices=(1, 2, 3),
        help="Manequim to drop on, according to detection order (1-3)",
    )
    return parser


def parse_args(args=None):
    parser = build_parser()
    parsed, _ = parser.parse_known_args(args)
    configure_manequim_number(parsed.manequim_number)

    if (parsed.latitude is None) != (parsed.longitude is None):
        raise ValueError("Latitude and longitude must be provided together.")

    if parsed.latitude is None and parsed.longitude is None:
        return parsed

    parsed.latitude = float(parsed.latitude)
    parsed.longitude = float(parsed.longitude)

    if not (-90.0 <= parsed.latitude <= 90.0):
        raise ValueError(f"Invalid latitude: {parsed.latitude}")

    if not (-180.0 <= parsed.longitude <= 180.0):
        raise ValueError(f"Invalid longitude: {parsed.longitude}")

    return parsed


# ── Search Navigation ─────────────────────────────────────────────────────────
ASCEND_HEIGHT   = 10.0  # altitude de varredura inicial (m)
SEARCH_ALTITUDE = 7.0   # altitude do espiral quadrado de busca (m)
SEARCH_RADIUS   = 25.0  # raio máximo da área de busca (m)

# ── YASMIN OUTCOMES ───────────────────────────────────────────────────────────
SQUARE_SEARCH = "SQUARE_SEARCH" 
MANEQUIM_FOUND = "MANEQUIM_FOUND"
POSITION_REACHED = "POSITION_REACHED"
