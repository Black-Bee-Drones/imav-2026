import argparse
import os

LATITUDE: float | None = None
LONGITUDE: float | None = None


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


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fly the drone to a specific coordinate for manequim search")
    parser.add_argument("--latitude", type=float, default=None, help="Initial Latitude")
    parser.add_argument("--longitude", type=float, default=None, help="Initial Longitude")
    return parser


def parse_args(args=None):
    parser = build_parser()
    parsed, _ = parser.parse_known_args(args)

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
ASCEND_HEIGHT   = 7.0  # altitude de varredura inicial (m)
SEARCH_ALTITUDE = 5.0   # altitude do espiral quadrado de busca (m)
SEARCH_RADIUS   = 25.0  # raio máximo da área de busca (m)

# ── YASMIN OUTCOMES ───────────────────────────────────────────────────────────
SQUARE_SEARCH = "SQUARE_SEARCH" 
MANEQUIM_FOUND = "MANEQUIM_FOUND"
POSITION_REACHED = "POSITION_REACHED"
