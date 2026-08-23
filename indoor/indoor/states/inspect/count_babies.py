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
        self.add_output_key('inspect_babies_boxes')

        self.add_input_key('image_handler_front')
        self.add_input_key('callback_baby')

        self.add_input_key('model_baby_classes_names')
        self.add_input_key('model_baby_overlap_iou')

    def execute(self, blackboard: Blackboard):
        handler: ImageHandler = blackboard.get('image_handler_front')
        handler.image_processing_callback = blackboard.get('callback_baby')

        result: DetectionResult = handler.take_photo()

        if result is not None:
            babies = result.filter_by_class(
                [blackboard.get('model_baby_classes_names')])
            boxes = self._merge_overlapping(
                babies,
                blackboard.get('model_baby_overlap_iou'),
            )

            yasmin.YASMIN_LOG_INFO(f'Number of babies: {len(boxes)}.')
            for i, box in enumerate(boxes):
                yasmin.YASMIN_LOG_INFO(f'Baby {i}: bbox={box}')

            blackboard.set('inspect_babies_count', len(boxes))
            blackboard.set('inspect_babies_boxes', boxes)

        return SUCCEED

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
    def _merge_overlapping(cls, result: DetectionResult, iou_threshold: float) -> list[list[int]]:
        """
        Group detections whose boxes overlap by at least `iou_threshold` (union-find)
        and collapse each group into a single enclosing bounding box, so that
        overlapping person/teddy_bear detections of the same baby count once.
        """
        boxes = [tuple(float(v) for v in det.xyxy) for det in result]
        n = len(boxes)
        if n == 0:
            return []

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

        groups: dict[int, list[tuple]] = {}
        for idx in range(n):
            groups.setdefault(find(idx), []).append(boxes[idx])

        merged = []
        for group_boxes in groups.values():
            x1 = min(b[0] for b in group_boxes)
            y1 = min(b[1] for b in group_boxes)
            x2 = max(b[2] for b in group_boxes)
            y2 = max(b[3] for b in group_boxes)
            merged.append([int(x1), int(y1), int(x2), int(y2)])

        return merged
