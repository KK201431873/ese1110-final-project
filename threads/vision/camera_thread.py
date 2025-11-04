# read camera images, do apriltag detection, run object detector
from utils import PiThread
from threads.arduino_serial import ArduinoSerialThread
from picamera2 import Picamera2
from ultralytics import YOLO
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