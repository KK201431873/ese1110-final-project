# read camera images, do apriltag detection, run object detector
from utils.pi_thread import PiThread
from picamera2 import Picamera2
import onnxruntime as ort
import numpy as np
import numpy.typing as npt
import time
import cv2

class CameraThread(PiThread):
    model_path: str = "threads/vision/detection_models/pong_11-2-25_1226AM_small.onnx"
    input_size: tuple[int, int] = (256, 256)
    min_score: float = 0.65

    def _on_created_impl(self) -> None:
        # Initialize camera
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": "XRGB8888", "size": (640, 480)}
        )
        self.picam2.configure(preview_config)
        
        # Initialize ONNX Runtime
        self.session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        # Get input size from model
        input_shape = self.session.get_inputs()[0].shape  # e.g., [1, 3, 256, 256]
        self.input_size = (input_shape[2], input_shape[3])

        # Detector class names
        self.class_names = ["ping-pong-ball"]  # update with your classes

    def _on_start_impl(self) -> None:
        self.picam2.start()
        time.sleep(1)
        self.picam2.set_controls({
            "AeEnable": False,
            "ExposureTime": 5000 # microseconds
        })
        self.print("Alive!")

    def _loop_impl(self) -> None:
        # Capture full-res frame
        frame = self.picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        # Run inference
        img = self._preprocess(frame)
        outputs: npt.NDArray[np.float32] = self.session.run(
            [self.output_name], {self.input_name: img}
        )[0]  # type: ignore[assignment]

        # --- Parse outputs (depends on model export) ---
        # For YOLOv8 ONNX models, outputs shape is (1, N, 85)
        #   x, y, w, h, conf, class_scores[80]
        boxes, confidences, class_ids = self._postprocess(outputs, frame.shape[:2])

        # Draw results
        annotated = frame.copy()
        detection_points = []
        for (x1, y1, x2, y2), score, cls in zip(boxes, confidences, class_ids):
            if score < self.min_score:
                continue
            color = (0, 255, 0)
            label = f"{self.class_names[int(cls)] if int(cls) < len(self.class_names) else 'obj'} {score:.2f}"
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            detection_points.append(((x1 + x2) / 2, min(y1, y2)))

        # Share detection data
        self["detection.frame"] = annotated
        self["detection.points"] = detection_points


    def _preprocess(self, img: np.ndarray) -> np.ndarray:
        # Resize to 256x256 (your model’s expected input)
        img_resized = cv2.resize(img, (256, 256))

        # Convert to float32 and normalize 0–1
        img = img_resized.astype(np.float32) / 255.0

        # Convert HWC → CHW (channels first)
        img = np.transpose(img, (2, 0, 1))

        # Add batch dimension
        img = np.expand_dims(img, axis=0)

        return img

    def _postprocess(self, outputs: np.ndarray, orig_shape: tuple[int, int]):
        """
        Convert YOLOv8 ONNX outputs to usable boxes in the original image space.
        Args:
            outputs: raw model outputs (1, N, 85)
            orig_shape: (height, width) of the original frame
        """
        predictions = np.squeeze(outputs)  # (N, 85)
        boxes, confidences, class_ids = [], [], []

        # YOLO outputs (x, y, w, h) are in 256x256 space (since you resized)
        input_w, input_h = 256, 256
        orig_h, orig_w = orig_shape

        scale_x = orig_w / input_w
        scale_y = orig_h / input_h

        for pred in predictions:
            x, y, w, h = pred[:4]
            object_conf = pred[4]
            class_scores = pred[5:]
            cls_id = np.argmax(class_scores)
            cls_conf = class_scores[cls_id]
            score = object_conf * cls_conf
            if score < self.min_score:
                continue

            # Convert to corner coords and scale back to full-res image
            x1 = int((x - w / 2) * scale_x)
            y1 = int((y - h / 2) * scale_y)
            x2 = int((x + w / 2) * scale_x)
            y2 = int((y + h / 2) * scale_y)

            boxes.append([x1, y1, x2 - x1, y2 - y1])
            confidences.append(float(score))
            class_ids.append(int(cls_id))

        # --- Apply NMS to remove duplicates ---
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.min_score, 0.45)
        final_boxes, final_confs, final_cls = [], [], []

        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                final_boxes.append((x, y, x + w, y + h))
                final_confs.append(confidences[i])
                final_cls.append(class_ids[i])

        return final_boxes, final_confs, final_cls

    def _on_shutdown_impl(self) -> None:
        self.picam2.stop()