from collections import defaultdict
from datetime import datetime
from pathlib import Path

import cv2
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED

from nectar.vision import ImageHandler
from nectar.ai import DetectionResult


class CountBabies(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED])

        self.node = YasminNode.get_instance()

    def configure(self):
        self.add_output_key('inspect_babies_count')

        self.add_input_key('image_handler_down')
        self.add_input_key('callback_baby')

        self.add_input_key('model_baby_classes_names')
        self.add_input_key('model_baby_overlap_iou')

        self.add_input_key('model_baby_sample_count')
        self.add_input_key('inspect_babies_output_path')

    def execute(self, blackboard: Blackboard):
        handler: ImageHandler = blackboard.get('image_handler_down')
        handler.image_processing_callback = blackboard.get('callback_baby')

        classes_names = blackboard.get('model_baby_classes_names')
        iou_threshold = blackboard.get('model_baby_overlap_iou')
        sample_count = blackboard.get('model_baby_sample_count')
        output_path = blackboard.get('inspect_babies_output_path')

        samples = []  # list of (count, boxes, confidence, raw_result)

        while len(samples) < sample_count:
            result: DetectionResult = handler.take_photo()

            if result is None:
                continue

            babies = result.filter_by_class(classes_names)
            boxes, confidence = self._merge_overlapping(babies, iou_threshold)

            yasmin.YASMIN_LOG_INFO(
                f'Sample {len(samples)}: number of babies = {len(boxes)} '
                f'(confidence={confidence:.3f}).'
            )
            for i, box in enumerate(boxes):
                yasmin.YASMIN_LOG_INFO(f'Baby {i}: bbox={box}')

            samples.append((len(boxes), boxes, confidence, result))

        matching = [s for s in samples if s[0] == final_count]
        best_count, best_boxes, best_confidence, best_result = max(
            matching, key=lambda s: s[2]
        )

        yasmin.YASMIN_LOG_INFO(f'BABIES AMOUNT: {best_count}')

        saved_path = self._save_labeled_image(best_result, best_boxes, output_path)
        yasmin.YASMIN_LOG_INFO(f'Saved labeled image to {saved_path}.')

        final_count = blackboard.get('inspect_babies_count')

        return SUCCEED

    @staticmethod
    def _save_labeled_image(result: DetectionResult, boxes: list[list[int]], output_path: str) -> str:
        """
        Draw the merged bounding boxes onto the raw image associated with
        `result` and save it to `output_path`.

        NOTE: this assumes `result` exposes the raw frame as `result.image`
        (a numpy/cv2 BGR array). Adjust the attribute name if your
        DetectionResult exposes the image differently.
        """
        image = result.image.copy()

        for i, (x1, y1, x2, y2) in enumerate(boxes):
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'baby_{i}'
            cv2.putText(
                image,
                label,
                (x1, max(0, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )

        path = Path(output_path)

        image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif', '.webp'}

        if path.is_dir() or path.suffix.lower() not in image_extensions:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = path / f'babies_{timestamp}.jpg'

        path.parent.mkdir(parents=True, exist_ok=True)

        if not cv2.imwrite(str(path), image):
            raise RuntimeError(f'Failed to write labeled image to {path}')

        return str(path)

    @staticmethod
    def _iou(box_a, box_b) -> float:
        """Intersection-over-union between two [x1, y1, x2, y2] boxes."""
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b

        inter_x1 = max(ax1, bx1)
        inter_y1 = max(ay1, by1)
        inter_x2 = min(ax2, bx2)
        inter_y2 = min(ay2, by2)

        inter_w = max(0.0, inter_x2 - inter_x1)
        inter_h = max(0.0, inter_y2 - inter_y1)
        inter_area = inter_w * inter_h

        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union_area = area_a + area_b - inter_area

        return inter_area / union_area if union_area > 0 else 0.0

    @classmethod
    def _merge_overlapping(
        cls, result: DetectionResult, iou_threshold: float
    ) -> tuple[list[list[int]], float]:
        """
        Group detections whose boxes overlap by at least `iou_threshold` (union-find)
        and collapse each group into a single enclosing bounding box, so that
        overlapping person/teddy_bear detections of the same baby count once.

        Also returns an overall confidence score for this sample: the mean
        of each merged group's average detection confidence. Assumes each
        detection exposes a `.conf` (float) attribute; adjust if your
        DetectionResult uses a different name (e.g. `.confidence`).
        """
        boxes = [tuple(float(v) for v in det.xyxy) for det in result]
        confidences = [float(det.confidence) for det in result]
        n = len(boxes)
        if n == 0:
            return [], 0.0

        parent = list(range(n))

        def find(i: int) -> int:
            while parent[i] != i:
                parent[i] = parent[parent[i]]
                i = parent[i]
            return i

        def union(i: int, j: int) -> None:
            ri, rj = find(i), find(j)
            if ri != rj:
                parent[ri] = rj

        for i in range(n):
            for j in range(i + 1, n):
                if cls._iou(boxes[i], boxes[j]) >= iou_threshold:
                    union(i, j)

        groups: dict[int, list[int]] = {}
        for idx in range(n):
            groups.setdefault(find(idx), []).append(idx)

        merged = []
        group_confidences = []
        for indices in groups.values():
            group_boxes = [boxes[i] for i in indices]
            x1 = min(b[0] for b in group_boxes)
            y1 = min(b[1] for b in group_boxes)
            x2 = max(b[2] for b in group_boxes)
            y2 = max(b[3] for b in group_boxes)
            merged.append([int(x1), int(y1), int(x2), int(y2)])

            group_confidences.append(
                sum(confidences[i] for i in indices) / len(indices)
            )

        sample_confidence = sum(group_confidences) / len(group_confidences)

        return merged, sample_confidence