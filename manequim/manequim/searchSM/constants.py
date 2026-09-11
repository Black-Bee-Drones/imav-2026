import argparse
import os

def configure_coordinates(latitude: float, longitude: float) -> None:
    global LATITUDE, LONGITUDE
    LATITUDE = float(latitude)
    LONGITUDE = float(longitude)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Fly the drone to a specific coordinate for manequim search")
    parser.add_argument("--latitude", type=float, default=LATITUDE, help="Initial Latitude")
    parser.add_argument("--longitude", type=float, default=LONGITUDE, help="Initial Longitude")
    return parser


def parse_args(args=None):
    parser = build_parser()
    parsed, _ = parser.parse_known_args(args)
    return parsed

LATITUDE  = 0.0
LONGITUDE = 0.0


# ── Search Navigation ─────────────────────────────────────────────────────────
ASCEND_HEIGHT   = 10.0  # altitude de varredura inicial (m)
SEARCH_ALTITUDE = 7.0   # altitude do espiral quadrado de busca (m)
SEARCH_RADIUS   = 25.0  # raio máximo da área de busca (m)

# ── YASMIN OUTCOMES ───────────────────────────────────────────────────────────
SQUARE_SEARCH = "SQUARE_SEARCH" 
MANEQUIM_FOUND = "MANEQUIM_FOUND"
POSITION_REACHED = "POSITION_REACHED"
