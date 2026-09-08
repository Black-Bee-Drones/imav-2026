import argparse
import logging
import uuid
from argparse import Namespace

import cv2
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

import nectar
from nectar.ai import Detector
from nectar.vision import ImageCalculus, ImageHandler, ROSDepthConfig

from indoor.config import Config
from indoor.gate_range import (
    bbox_clipped,
    depth_on_color,
    draw_align_markers,
    draw_hud,
    fmt_z,
    fuse_z,
    overlay_gate,
    pick_depth_topic,
    px_to_m,
    smooth_z,
    z_from_bbox,
    z_from_frame,
)

log = logging.getLogger("gate_depth_check")

_PUBLISH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class GateDepthCheck:
    def __init__(self, args: argparse.Namespace, config: Config) -> None:
        self.config = config
        self.confidence = args.confidence
        self.jpeg_quality = args.jpeg_quality
        self.frame_count = 0
        self.fx = 0.0
        self.fy = 0.0
        self.z_smooth = None

        log.info("Loading model: %s", config.model_gate_source)
        self.detector = Detector(
            model_source=config.model_gate_source,
            confidence_threshold=self.confidence,
        )
        self.detector.load()

        self._pub_node: Node | None = None
        self._pub = None
        if args.publish:
            self._pub_node = Node(
                f"indoor_gate_depth_pub_{uuid.uuid4().hex[:8]}",
                start_parameter_services=False,
            )
            nectar.add_node(self._pub_node)
            self._pub = self._pub_node.create_publisher(
                CompressedImage, args.publish_topic, _PUBLISH_QOS
            )
            log.info("Publishing annotated frames on %s", args.publish_topic)

        cam_cfg = ROSDepthConfig(
            topic=config.camera_north_topic,
            compressed=config.camera_north_is_compressed,
            depth_topic=config.camera_north_depth_topic,
            depth_compressed=False,
            enable_depth=True,
        )
        self.handler = ImageHandler(
            image_source="ros_depth",
            config=cam_cfg,
            show_result="Gate depth" if args.show_result else None,
            image_processing_callback=self.process_frame,
        )
        self.handler.run()
        log.info(
            "color=%s depth=%s  press q to quit",
            config.camera_north_topic,
            config.camera_north_depth_topic,
        )

    def process_frame(self, frame) -> None:
        if frame is None:
            return
        self.frame_count += 1
        cfg = self.config
        h, w = frame.shape[:2]
        if self.fx <= 0.0:
            self.fx = ImageCalculus.focal_length_px(w, cfg.camera_north_hfov)
            self.fy = ImageCalculus.focal_length_px(h, cfg.camera_north_vfov)

        result = self.detector.detect(frame, conf=self.confidence)
        annotated = self.detector.draw_detections(
            image=frame,
            result=result,
            show_labels=True,
            show_confidence=True,
            show_class=True,
            thickness=2,
            text_scale=0.5,
        )
        if annotated is not frame:
            frame[:] = annotated

        raw = None
        cam = self.handler.camera
        if cam is not None:
            try:
                raw = cam.get_depth_frame()
            except Exception:
                raw = None
        dh, dw = (raw.shape[0], raw.shape[1]) if raw is not None else (0, 0)
        depth_color = depth_on_color(raw, (h, w))

        window = result.filter_by_class([cfg.model_gate_class_name])
        z_box = z_depth = z = None
        clipped = too_big = at_standoff = centered = False
        w_px = 0.0
        det_xy = None
        error_y = error_z = err_y_m = err_z_m = 0.0
        range_m = self.z_smooth if self.z_smooth is not None else cfg.obstacle_gate_standoff
        z_aim_m = cfg.camera_north_offset_z - cfg.obstacle_gate_center_z_down_m
        cx_sp = w / 2 + ImageCalculus.meters_to_pixels(
            cfg.camera_north_offset_y, range_m, self.fx
        )
        cy_sp = h / 2 + ImageCalculus.meters_to_pixels(
            z_aim_m, range_m, self.fy
        )

        if window:
            det = max(window, key=lambda d: d.area)
            xyxy = det.xyxy
            det_xy = det.center
            clipped = bbox_clipped(xyxy, (h, w))
            w_px = float(xyxy[2] - xyxy[0])
            if not clipped:
                z_box = z_from_bbox(w_px, self.fx, cfg.obstacle_gate_width)
            z_depth = z_from_frame(
                depth_color,
                xyxy,
                cfg.obstacle_gate_depth_min,
                cfg.obstacle_gate_depth_max,
                cfg.obstacle_gate_frame_expand,
            )
            z = fuse_z(z_box, z_depth, cfg.obstacle_gate_fuse_delta)
            self.z_smooth = smooth_z(self.z_smooth, z)
            range_m = (
                self.z_smooth
                if self.z_smooth is not None
                else cfg.obstacle_gate_standoff
            )
            cx_sp = w / 2 + ImageCalculus.meters_to_pixels(
                cfg.camera_north_offset_y, range_m, self.fx
            )
            cy_sp = h / 2 + ImageCalculus.meters_to_pixels(
                z_aim_m, range_m, self.fy
            )
            cx, cy = det_xy
            error_y = cx_sp - cx
            error_z = cy_sp - cy
            err_y_m = px_to_m(error_y, range_m, self.fx)
            err_z_m = px_to_m(error_z, range_m, self.fy)
            too_big = w_px >= cfg.obstacle_gate_bbox_frac * w
            at_standoff = (
                self.z_smooth is not None
                and self.z_smooth <= cfg.obstacle_gate_standoff
            )
            centered = (
                abs(err_y_m) < cfg.obstacle_gate_aligned_tolerance_m
                and abs(err_z_m) < cfg.obstacle_gate_aligned_tolerance_m
            )
            overlay_gate(
                frame,
                depth_color,
                xyxy,
                cfg.obstacle_gate_depth_min,
                cfg.obstacle_gate_depth_max,
                cfg.obstacle_gate_frame_expand,
            )

        draw_align_markers(frame, cx_sp, cy_sp, det_xy)
        should_stop = too_big or at_standoff
        draw_hud(
            frame,
            [
                f"topic {cfg.camera_north_depth_topic}  color {w}x{h} depth {dw}x{dh}",
                f"y={error_y:+.0f}px {err_y_m:+.2f}m  z={error_z:+.0f}px {err_z_m:+.2f}m  "
                f"box {w_px:.0f}px",
                f"Zb {fmt_z(z_box)} Zd {fmt_z(z_depth)} Z {fmt_z(z)} Zs {fmt_z(self.z_smooth)}",
                f"clip={int(clipped)} big={int(too_big)} stop={int(at_standoff)} "
                f"ctr={int(centered)} would_stop={int(should_stop)}",
            ],
        )

        if self.frame_count % 15 == 1:
            log.info(
                "frame %d dets=%d box=%.0fpx clip=%s big=%s stop=%s "
                "Zb=%s Zd=%s Z=%s Zs=%s  y=%+.0fpx %+.3fm z=%+.0fpx %+.3fm",
                self.frame_count,
                len(result),
                w_px,
                clipped,
                too_big,
                at_standoff,
                fmt_z(z_box),
                fmt_z(z_depth),
                fmt_z(z),
                fmt_z(self.z_smooth),
                error_y,
                err_y_m,
                error_z,
                err_z_m,
            )

        if self._pub is not None:
            ok, buf = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            )
            if ok:
                msg = CompressedImage()
                msg.header.stamp = self._pub_node.get_clock().now().to_msg()
                msg.header.frame_id = "north_camera"
                msg.format = "jpeg"
                msg.data = buf.tobytes()
                self._pub.publish(msg)

    def cleanup(self) -> None:
        self.handler.cleanup()
        if self._pub_node is not None:
            nectar.remove_node(self._pub_node)
            try:
                self._pub_node.destroy_node()
            except Exception:
                pass


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Gate detection + depth ranging overlay")
    p.add_argument("--sitl", action="store_true")
    p.add_argument("--model", default="")
    p.add_argument("--confidence", type=float, default=None)
    p.add_argument("--publish", action="store_true")
    p.add_argument("--publish-topic", default="/inference/compressed")
    p.add_argument("--jpeg-quality", type=int, default=80)
    p.add_argument("--show-result", action="store_true", default=True)
    p.add_argument("--no-show", dest="show_result", action="store_false")
    args, _ = p.parse_known_args()
    return args


def _config_from_args(args: argparse.Namespace) -> Config:
    ns = Namespace(
        preset=None,
        sitl=args.sitl,
        obstacle_skip=False,
        inspect_skip=False,
        droping_skip=False,
        precise_skip=False,
        no_takeoff=False,
    )
    config = Config().apply_args(ns)
    if args.model:
        config.model_gate_source = args.model
    if args.confidence is not None:
        config.model_gate_conf = args.confidence
    else:
        args.confidence = config.model_gate_conf
    if args.sitl:
        return config
    probe = Node("gate_depth_check_probe", start_parameter_services=False)
    nectar.add_node(probe)
    picked = pick_depth_topic(
        probe,
        config.camera_north_depth_topic,
        config.camera_north_depth_topic_fallback,
    )
    if picked != config.camera_north_depth_topic:
        log.warning(
            "depth topic %s missing, using %s",
            config.camera_north_depth_topic,
            picked,
        )
    config.camera_north_depth_topic = picked
    nectar.remove_node(probe)
    try:
        probe.destroy_node()
    except Exception:
        pass
    return config


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    nectar.init()
    args = parse_args()
    stream = None
    try:
        config = _config_from_args(args)
        stream = GateDepthCheck(args, config)
        nectar.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if stream is not None:
            stream.cleanup()
        nectar.shutdown()


if __name__ == "__main__":
    main()
