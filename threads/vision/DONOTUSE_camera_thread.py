# read camera images, do apriltag detection, run object detector
from utils.pi_thread import PiThread
from picamera2 import Picamera2
import onnxruntime as ort
import numpy.typing as npt
import numpy as np
import time
import cv2

class CameraThread(PiThread):
    # model_path: str = "threads/vision/detection_models/pong_11-2-25_1226AM_small.onnx"
    model_path: str = "threads/vision/detection_models/pong_11-2-25_145AM_pruned50pct.onnx"
    input_size: tuple[int, int] = (256, 256)
    min_score: float = 0.02

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
        predictions = outputs.squeeze()     # (5, 1344)
        predictions = predictions.T         # (1344, 5)

        # --- Parse outputs (depends on model export) ---
        # For YOLOv8 ONNX models, outputs shape is (1, N, 85)
        #   x, y, w, h, conf, class_scores[80]
        orig_shape: tuple[int, int] = (int(frame.shape[0]), int(frame.shape[1]))
        boxes, confidences, class_ids = self._postprocess(outputs, orig_shape)

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

    def _postprocess(
        self,
        outputs: np.ndarray,
        orig_shape: tuple[int, int],
    ) -> tuple[list[tuple[int, int, int, int]], list[float], list[int]]:
        """
        Postprocess model outputs (1, 5, 1344) → boxes, confidences, class_ids.
        Args:
            outputs: raw model output (1, 5, 1344) where each detection = [x, y, w, h, conf]
            orig_shape: (height, width) of the original frame
        """
        # Remove batch dimension and transpose → (1344, 5)
        preds = np.squeeze(outputs).T

        boxes, confidences, class_ids = [], [], []

        input_w, input_h = self.input_size
        orig_h, orig_w = orig_shape
        scale_x = orig_w / input_w
        scale_y = orig_h / input_h

        for x, y, w, h, conf in preds:
            # Skip low-confidence boxes early
            if conf < self.min_score:
                continue

            # Convert YOLO center-based boxes to pixel corner coordinates
            x1 = int((x - w / 2) * scale_x)
            y1 = int((y - h / 2) * scale_y)
            x2 = int((x + w / 2) * scale_x)
            y2 = int((y + h / 2) * scale_y)

            boxes.append([x1, y1, x2 - x1, y2 - y1])
            confidences.append(float(conf))
            class_ids.append(0)  # single class: ping-pong-ball

        # --- Apply NMS to clean duplicates ---
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.min_score, 0.45)
        final_boxes, final_confs, final_cls = [], [], []

        if len(indices) > 0:
            # Flatten and extract final kept boxes
            indices = np.array(indices).flatten()
            for i in indices:
                x, y, w, h = boxes[i]
                final_boxes.append((x, y, x + w, y + h))
                final_confs.append(confidences[i])
                final_cls.append(class_ids[i])

        return final_boxes, final_confs, final_cls

    def _on_shutdown_impl(self) -> None:
        self.picam2.stop()



# read camera images, do apriltag detection, run object detector
from utils.pi_thread import PiThread
from threads.arduino_serial.arduino_serial_thread import ArduinoSerialThread
from picamera2 import Picamera2
from ultralytics.models.yolo import YOLO
import time
import cv2

class CameraThread(PiThread):
    model_path: str = "threads/vision/detection_models/pong_11-2-25_1226AM_small.onnx"
    min_score: float = 0.65

    def _on_created_impl(self) -> None:
        # Initialize camera
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
        self.picam2.configure(preview_config)
        self.model = YOLO(self.model_path, task="detect", verbose=False)

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
        h, w = frame.shape[:2]

        # Resize frame for YOLO inference
        small_frame = cv2.resize(frame, (160, 120))
        small_h, small_w = small_frame.shape[:2]

        # Run YOLO inference on small frame
        results = self.model(small_frame, verbose=False)

        # Scale factor from small frame to full frame
        scale_x = w / small_w
        scale_y = h / small_h

        # Draw detection boxes on full-res frame
        annotated = frame.copy()
        detection_points = []
        for r in results:
            boxes = r.boxes.xyxy.cpu().numpy()  # get bounding boxes
            class_ids = r.boxes.cls.cpu().numpy()
            scores = r.boxes.conf.cpu().numpy()

            for (x1, y1, x2, y2), cls, score in zip(boxes, class_ids, scores):
                # Scale box coordinates
                x1 = int(x1 * scale_x)
                y1 = int(y1 * scale_y)
                x2 = int(x2 * scale_x)
                y2 = int(y2 * scale_y)
                detection_points.append(((x1 + x2) / 2, min(y1, y2)))

                # Draw rectangle and label on full-res frame
                above_min_score: bool = score >= self.min_score
                color = (0, 255, 0) if above_min_score else (0, 0, 255)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                label = f"{self.model.names[int(cls)]} {score:.2f}"
                cv2.putText(annotated, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        self["detection.frame"] = annotated
        self["detection.points"] = detection_points
    
    def _on_shutdown_impl(self) -> None:
        self.picam2.stop()