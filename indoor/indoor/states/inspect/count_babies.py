import yasmin
from yasmin import State, Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED

from nectar.vision import ImageHandler
from nectar.ai import DetectionResult

from indoor import Config


class CountBabies(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED])

        self.config = config
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        image_handler_down: ImageHandler = blackboard.get('image_handler_down')
        image_handler_down.image_processing_callback = blackboard.get(
            'callback_detector_baby')

        result: DetectionResult = image_handler_down.take_photo()
        yasmin.YASMIN_LOG_INFO(f'Number of babies: {len(result)}.')

        yasmin.YASMIN_LOG_INFO('Completed successfully!!!')
        return SUCCEED
