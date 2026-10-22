# Majd
"""Detect cups in RGB frames using the bundled COCO model."""

from __future__ import annotations

import hashlib
from pathlib import Path

from aurora.paths import MODEL_DIRECTORY

MODEL_PATH = MODEL_DIRECTORY / "cup_yolo11n.onnx"
CUP_CLASS = 41


def prepare_frame(frame, input_size, cv2, np):
    height, width = frame.shape[:2]
    target_height, target_width = input_size
    scale = min(target_width / width, target_height / height)
    resized_width, resized_height = round(width * scale), round(height * scale)
    left, top = (target_width - resized_width) // 2, (target_height - resized_height) // 2
    canvas = np.full((target_height, target_width, 3), 114, dtype=np.uint8)
    canvas[top : top + resized_height, left : left + resized_width] = cv2.resize(
        frame, (resized_width, resized_height)
    )
    tensor = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)
    return tensor / 255.0, scale, (left, top)


def decode_cups(output, frame_size, scale, padding, threshold, cv2, np):
    predictions = output[0].T
    classes = np.argmax(predictions[:, 4:], axis=1)
    scores = predictions[:, 4 + CUP_CLASS]
    selected = predictions[(classes == CUP_CLASS) & (scores >= threshold)]
    width, height = frame_size
    boxes, confidences = [], []
    for prediction in selected:
        cx, cy, box_width, box_height = prediction[:4]
        confidence = float(prediction[4 + CUP_CLASS])
        if (
            not np.isfinite(prediction[:4]).all()
            or not np.isfinite(confidence)
            or box_width <= 0
            or box_height <= 0
        ):
            continue
        x1 = max(0.0, min(width - 1.0, (cx - box_width / 2 - padding[0]) / scale))
        y1 = max(0.0, min(height - 1.0, (cy - box_height / 2 - padding[1]) / scale))
        x2 = max(0.0, min(width - 1.0, (cx + box_width / 2 - padding[0]) / scale))
        y2 = max(0.0, min(height - 1.0, (cy + box_height / 2 - padding[1]) / scale))
        if not all(np.isfinite(value) for value in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
            continue
        boxes.append([float(x1), float(y1), float(x2 - x1), float(y2 - y1)])
        confidences.append(confidence)
    if not boxes:
        return []
    retained = cv2.dnn.NMSBoxes(boxes, confidences, threshold, 0.45)
    return [
        {
            "label": "cup",
            "confidence": round(confidences[index], 4),
            "bbox_xyxy": [
                round(boxes[index][0], 2),
                round(boxes[index][1], 2),
                round(boxes[index][0] + boxes[index][2], 2),
                round(boxes[index][1] + boxes[index][3], 2),
            ],
        }
        for index in np.asarray(retained).reshape(-1)[:5]
    ]


class CupDetector:
    name = "yolo11n_coco"

    def __init__(self, cv2, np, model_path: Path = MODEL_PATH, threshold: float = 0.25):
        self.cv2, self.np = cv2, np
        self.threshold = threshold
        self.session = None
        self.model_sha256 = ""
        self.status = "model missing"
        if not model_path.is_file():
            return
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        model = model_path.read_bytes()
        self.model_sha256 = hashlib.sha256(model).hexdigest()
        self.session = ort.InferenceSession(
            model, sess_options=options, providers=["CPUExecutionProvider"]
        )
        source = self.session.get_inputs()[0]
        self.input_name = source.name
        self.input_size = tuple(source.shape[2:])
        self.status = "ready"

    def detect(self, frame) -> list[dict]:
        if self.session is None:
            return []
        tensor, scale, padding = prepare_frame(frame, self.input_size, self.cv2, self.np)
        output = self.session.run(None, {self.input_name: tensor})[0]
        return decode_cups(
            output,
            (frame.shape[1], frame.shape[0]),
            scale,
            padding,
            self.threshold,
            self.cv2,
            self.np,
        )

    def close(self) -> None:
        self.session = None


def add_cup_tracking(frame, output, observation, detector, cv2) -> None:
    observation["cup_visible"] = False
    observation["cup_detections"] = []
    observation["frame_size"] = [frame.shape[1], frame.shape[0]]
    if detector is None:
        observation["cup_detector_status"] = "not enabled"
        return
    detections = detector.detect(frame)
    observation["cup_detector"] = detector.name
    observation["cup_detector_status"] = detector.status
    observation["cup_model_sha256"] = detector.model_sha256
    observation["cup_confidence_threshold"] = detector.threshold
    observation["cup_detections"] = detections
    observation["cup_visible"] = bool(detections)
    width = frame.shape[1]
    for detection in detections:
        x1, y1, x2, y2 = detection["bbox_xyxy"]
        left, right = width - 1 - round(x2), width - 1 - round(x1)
        cv2.rectangle(output, (left, round(y1)), (right, round(y2)), (255, 180, 40), 2)
        cv2.putText(
            output,
            f"Cup {detection['confidence']:.0%}",
            (left, max(18, round(y1) - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 180, 40),
            1,
        )
