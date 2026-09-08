import time

import cv2
import numpy as np

_CYAN = (255, 255, 0)
_MAGENTA = (255, 0, 255)
_GREEN = (0, 255, 0)
_FONT = cv2.FONT_HERSHEY_SIMPLEX
_FRAME_EXPAND = 0.25
_MIN_PX = 8.0
_MIN_BAND = 16
Z_EMA = 0.3


def fmt_z(value: float | None) -> str:
    return "—" if value is None else f"{value:.2f}m"


def pick_depth_topic(node, preferred: str, fallback: str | None, retries: int = 10) -> str:
    for _ in range(max(1, retries)):
        topics = {name for name, _ in node.get_topic_names_and_types()}
        if preferred in topics:
            return preferred
        if fallback and fallback in topics:
            return fallback
        time.sleep(0.2)
    return preferred


def depth_on_color(depth, color_hw):
    if depth is None or color_hw is None:
        return None
    ch, cw = int(color_hw[0]), int(color_hw[1])
    if ch <= 0 or cw <= 0 or depth.size == 0:
        return None
    if depth.shape[0] == ch and depth.shape[1] == cw:
        return depth
    return cv2.resize(depth, (cw, ch), interpolation=cv2.INTER_NEAREST)


def bbox_clipped(xyxy, image_hw, margin_px: float = 4.0) -> bool:
    h, w = float(image_hw[0]), float(image_hw[1])
    x1, y1, x2, y2 = (float(v) for v in xyxy)
    return (
        x1 <= margin_px
        or y1 <= margin_px
        or x2 >= w - 1.0 - margin_px
        or y2 >= h - 1.0 - margin_px
    )


def z_from_bbox(w_px: float, fx: float, width_m: float) -> float | None:
    if w_px < _MIN_PX or fx <= 0.0 or width_m <= 0.0:
        return None
    return fx * width_m / w_px


def px_to_m(px: float, range_m: float | None, focal: float) -> float:
    if focal <= 0.0 or range_m is None or range_m <= 0.0:
        return 0.0
    return px * range_m / focal


def smooth_z(z_smooth: float | None, z: float | None, alpha: float = Z_EMA) -> float | None:
    if z is None:
        return z_smooth
    if z_smooth is None:
        return z
    return alpha * z + (1.0 - alpha) * z_smooth


def _band_rects(xyxy, image_hw, expand: float = _FRAME_EXPAND):
    h, w = int(image_hw[0]), int(image_hw[1])
    x1, y1, x2, y2 = (float(v) for v in xyxy)
    bw = x2 - x1
    bh = y2 - y1
    if bw < _MIN_PX or bh < _MIN_PX or w <= 0 or h <= 0:
        return None
    ix1 = max(0, int(x1))
    iy1 = max(0, int(y1))
    ix2 = min(w, int(x2))
    iy2 = min(h, int(y2))
    ox1 = max(0, int(x1 - expand * bw))
    oy1 = max(0, int(y1 - expand * bh))
    ox2 = min(w, int(x2 + expand * bw))
    oy2 = min(h, int(y2 + expand * bh))
    if ox2 <= ox1 or oy2 <= oy1 or ix2 <= ix1 or iy2 <= iy1:
        return None
    return ox1, oy1, ox2, oy2, ix1, iy1, ix2, iy2


def z_from_frame(
    depth_color,
    xyxy,
    z_min: float,
    z_max: float,
    expand: float = _FRAME_EXPAND,
) -> float | None:
    if depth_color is None or xyxy is None:
        return None
    rects = _band_rects(xyxy, depth_color.shape, expand)
    if rects is None:
        return None
    ox1, oy1, ox2, oy2, ix1, iy1, ix2, iy2 = rects
    band = depth_color[oy1:oy2, ox1:ox2]
    if band.size == 0:
        return None
    valid = np.isfinite(band) & (band >= z_min) & (band <= z_max)
    valid[iy1 - oy1 : iy2 - oy1, ix1 - ox1 : ix2 - ox1] = False
    samples = band[valid]
    if samples.size < _MIN_BAND:
        return None
    return float(np.median(samples))


def fuse_z(z_box: float | None, z_depth: float | None, max_delta: float = 0.3) -> float | None:
    if z_box is None:
        return z_depth
    if z_depth is None:
        return z_box
    if abs(z_depth - z_box) < max_delta:
        return 0.5 * (z_box + z_depth)
    return z_depth


def _colorize_depth(depth, z_min: float, z_max: float) -> np.ndarray:
    vis = np.zeros((*depth.shape[:2], 3), dtype=np.uint8)
    valid = np.isfinite(depth) & (depth >= z_min) & (depth <= z_max)
    if not np.any(valid):
        return vis
    span = max(z_max - z_min, 1e-6)
    gray = np.zeros(depth.shape[:2], dtype=np.uint8)
    gray[valid] = np.clip((depth[valid] - z_min) / span * 255.0, 0, 255).astype(np.uint8)
    vis = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    vis[~valid] = 0
    return vis


def overlay_gate(
    img,
    depth_color,
    xyxy,
    z_min: float,
    z_max: float,
    expand: float = _FRAME_EXPAND,
) -> None:
    if img is None or xyxy is None:
        return
    rects = _band_rects(xyxy, img.shape, expand)
    if rects is None:
        return
    ox1, oy1, ox2, oy2, ix1, iy1, ix2, iy2 = rects
    if depth_color is not None:
        crop = _colorize_depth(depth_color[oy1:oy2, ox1:ox2], z_min, z_max)
        crop[iy1 - oy1 : iy2 - oy1, ix1 - ox1 : ix2 - ox1] = 0
        roi = img[oy1:oy2, ox1:ox2]
        mask = crop.any(axis=2)
        if np.any(mask):
            blended = cv2.addWeighted(roi, 0.45, crop, 0.55, 0)
            roi[mask] = blended[mask]
    cv2.rectangle(img, (ox1, oy1), (ox2, oy2), _CYAN, 1)
    cv2.rectangle(img, (ix1, iy1), (ix2, iy2), _MAGENTA, 1)


def draw_align_markers(img, sx: float, sy: float, det_xy) -> None:
    if img is None:
        return
    sx, sy = int(round(sx)), int(round(sy))
    cv2.drawMarker(img, (sx, sy), _CYAN, cv2.MARKER_TILTED_CROSS, 14, 1)
    if det_xy is None:
        return
    cx, cy = int(round(det_xy[0])), int(round(det_xy[1]))
    cv2.drawMarker(img, (cx, cy), _MAGENTA, cv2.MARKER_CROSS, 12, 1)
    if abs(cx - sx) > 3 or abs(cy - sy) > 3:
        cv2.arrowedLine(
            img, (cx, cy), (sx, sy), _GREEN, 1, tipLength=0.12, line_type=cv2.LINE_AA
        )


def draw_hud(img, lines: list[str]) -> None:
    if img is None or not lines:
        return
    y = 6
    for line in lines:
        (tw, th), _ = cv2.getTextSize(line, _FONT, 0.45, 1)
        cv2.rectangle(img, (6, y), (10 + tw, y + 8 + th), (0, 0, 0), -1)
        cv2.putText(img, line, (8, y + 4 + th), _FONT, 0.45, _GREEN, 1, cv2.LINE_AA)
        y += 8 + th
