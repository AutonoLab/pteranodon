from typing import List, Tuple, Union, Any, Optional

import tensorflow as tf
import numpy as np
import cv2


class MobileNetV1:
    """
    Class which wraps the CNN used for detecting quadcopters in a common interface.
    """

    def __init__(
        self,
        model_path="./models/networks/mobilenet_v1/frozen_inference_graph.pb",
        frame_shape=(480, 640),
        min_score_threshold=0.25,
        diff_threshold=0.2,
        filter_frames=False,
        filter_jumps=False,
    ):
        # try:
        #     tf.enable_eager_execution()
        # except AttributeError:
        #     pass
        self.model_path = model_path
        self.frame_shape = frame_shape
        self.min_score = min_score_threshold
        self.diff_threshold = diff_threshold
        self.filter_frames = filter_frames
        self.filter_jumps = filter_jumps
        self.mobilenet = None
        self.initialized = False
        self.success = False
        self.frame = np.ndarray
        self.tensor = np.ndarray
        self.bounding_boxes: Any = None
        self.bounding_box: Optional[Tuple[int, int, int, int]] = None
        self.previous_box: Any = None
        self.scores = []

    def init(self) -> None:
        """Initialize the network"""
        self.mobilenet = MobileNetV1.load_frozen_model(
            self.model_path,
            inputs=[
                "image_tensor:0",
            ],
            outputs=[
                "detection_boxes:0",
                "detection_scores:0",
            ],
        )
        self.initialized = True

    def reset(self) -> None:
        """Reset the instance attributes, BUT does not un-initialize the network"""
        self.success = False
        self.frame = np.ndarray
        self.tensor = np.ndarray
        self.bounding_boxes = None
        self.bounding_box = None
        self.previous_box = None
        self.scores = []

    @staticmethod
    def wrap_frozen_graph(graph_def, inputs, outputs):
        """
        A function required to load in a tf v1 frozen graph in tf v2
        """

        def _imports_graph_def():
            tf.compat.v1.import_graph_def(graph_def, name="")

        wrapped_import = tf.compat.v1.wrap_function(_imports_graph_def, [])
        import_graph = wrapped_import.graph
        return wrapped_import.prune(
            tf.nest.map_structure(import_graph.as_graph_element, inputs),
            tf.nest.map_structure(import_graph.as_graph_element, outputs),
        )

    @staticmethod
    def load_frozen_model(model_path: str, inputs: List[str], outputs: List[str]):
        """Loads a frozen graph and creates a function for processing image inputs"""
        graph_def = tf.compat.v1.GraphDef()
        with open(model_path, "rb") as f:
            file = f.read()
            graph_def.ParseFromString(file)
            return MobileNetV1.wrap_frozen_graph(
                graph_def, inputs=inputs, outputs=outputs
            )

    def update_frame(
        self, frame
    ) -> None:  # TODO, can port color conversion to VPI, but slower
        """Update the frame and tensor attributes"""
        self.frame = cv2.resize(
            cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),
            (self.frame_shape[1], self.frame_shape[0]),
        )
        input_frame = np.expand_dims(frame, axis=0)
        self.tensor = tf.convert_to_tensor(input_frame, dtype=tf.uint8)

    def run(
        self, frame: Union[np.ndarray, None] = None
    ) -> Tuple[bool, Optional[Tuple[int, int, int, int]]]:
        """Perform an inference on a given frame. If frame is not given, performs inference on the last frame given."""
        assert self.initialized
        if frame is not None:
            self.update_frame(frame)
        self.bounding_boxes, self.scores = self.mobilenet(self.tensor)
        self.bounding_box, self.previous_box = self._process_bounding_boxes()
        self.success = True
        if self.bounding_box is None:
            self.success = False
        else:
            self.bounding_box = (
                self.bounding_box[1],
                self.bounding_box[0],
                self.bounding_box[3],
                self.bounding_box[2],
            )
        return self.success, self.bounding_box

    def _adjust_noisy_boxes(self, placements, boxes, chosen_index):
        noisy = False
        bad_box = False

        if len(placements) > 0:
            noisy = True
            for index in placements:
                bad_box = False
                curr_box = boxes[0, index, 0:4]
                diff = 0

                # checking size of bounding box compared to the previous box
                if self.previous_box.any():
                    curr_area = (curr_box[2] - curr_box[1]) * (
                        curr_box[3] - curr_box[0]
                    )
                    prev_area = (self.previous_box[2] - self.previous_box[1]) * (
                        self.previous_box[3] - self.previous_box[0]
                    )
                    diff = abs(curr_area - prev_area)

                # Justin Davis:
                # this check was present in the legacy code (before move to object oriented) (most likely to filter
                # jumps, however when only running
                # the CNN sporadically and filling in with other trackers, this results in no additional detections
                # thus the filter jumps flag was added which allows you to disable this check
                if diff > self.diff_threshold and self.filter_jumps:
                    continue
                curr_box = curr_box - self.previous_box

                # checking the position of the bounding box ( at all points) compared to the previous box
                for element in curr_box:
                    if abs(element) > 0.1 and self.previous_box.any():
                        bad_box = True
                if not bad_box:
                    chosen_index = index

            # using the selected box, check for noise (box is super close to the previous box, if so just use the
            # previous box)
            # default is the first box since it will have the best accuracy
            if chosen_index > -1:
                for element in boxes[0, chosen_index, 0:4]:
                    if abs(element) > 0.01:
                        noisy = False

        return placements, boxes, chosen_index, noisy

    def _filter_frames_and_jumps(self):
        """Filter out frames that are too different to the previous frame"""
        placements = []
        for index, score in enumerate(self.scores[0]):
            if score > self.min_score:
                # OPTIONAL: Running average
                placements.append(index)

        boxes = self.bounding_boxes.numpy()
        chosen_index = -1

        placements, boxes, chosen_index, noisy = self._adjust_noisy_boxes(
            placements, boxes, chosen_index
        )

        # check if we chose a box and score mets threshold
        # OPTIONAL: Finish merging properly
        best_bound = None
        if self.scores[0][0] > self.min_score and chosen_index > -1:
            # Not noise: then use tnew box, otherwise: use previous box
            if not noisy:
                best_bound = np.multiply(
                    boxes[0, chosen_index, 0:4],
                    np.array(
                        [
                            self.frame_shape[0],
                            self.frame_shape[1],
                            self.frame_shape[0],
                            self.frame_shape[1],
                        ]
                    ),
                )
                self.previous_box = boxes[0, chosen_index, 0:4]
            else:
                best_bound = np.multiply(
                    self.previous_box,
                    np.array(
                        [
                            self.frame_shape[0],
                            self.frame_shape[1],
                            self.frame_shape[0],
                            self.frame_shape[1],
                        ]
                    ),
                )
            self.bounding_box = best_bound.astype(int)  # type: ignore
            return self.bounding_box, self.previous_box
        return None, self.previous_box

    def _process_bounding_boxes(
        self,
    ) -> Tuple[
        Optional[Tuple[int, int, int, int]], Optional[Tuple[int, int, int, int]]
    ]:
        """Processes the bounding boxes produced by the inference from the network"""
        if self.previous_box is None:
            self.previous_box = np.array(0)
        if self.filter_frames:
            return self._filter_frames_and_jumps()

        self.previous_box = self.bounding_box
        if self.scores[0][0] > self.min_score:
            boxes = self.bounding_boxes.numpy()
            self.bounding_box = np.multiply(  # type: ignore
                boxes[0, 0, 0:4],
                np.array(
                    [
                        self.frame_shape[0],
                        self.frame_shape[1],
                        self.frame_shape[0],
                        self.frame_shape[1],
                    ]
                ),
            ).astype(int)
            return self.bounding_box, self.previous_box
        return None, self.previous_box

    def draw(self, show=False):
        """Draws the bounding box on the frame and displays if show is True"""
        # make a copy of the frame
        drawing = self.frame.copy()

        if self.bounding_box is not None:
            cv2.rectangle(
                drawing,
                (self.bounding_box[1], self.bounding_box[0]),
                (self.bounding_box[3], self.bounding_box[2]),
                (255, 0, 0),
                3,
            )

        if show:
            cv2.imshow("Detected Drone By Mobilenet", drawing)
            cv2.waitKey(1)

        return drawing
