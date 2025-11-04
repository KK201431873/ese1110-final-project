from picamera2 import Picamera2
import cv2
from ultralytics import YOLO
import numpy as np
import time

# Activate YOLO environment before running:
# source ~/yolo-env/bin/activate

# Load your ONNX model
# model_path = "./threads/vision/detection_models/pong_10-31-25_1033PM.onnx"
model_path = "./threads/vision/detection_models/pong_11-2-25_1226AM_small.onnx"
# model_path = "./threads/vision/detection_models/pong_11-2-25_145AM_pruned50pct.onnx"
model = YOLO(model_path)

# Initialize camera
picam2 = Picamera2()
preview_config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
picam2.configure(preview_config)
picam2.start()

# camera warms up
time.sleep(1)

picam2.set_controls({
    "AeEnable": False,
    "ExposureTime": 5000 # microseconds
})


print("Camera started. Press 'q' to quit.")

while True:
    # Cluttered images
    # # Capture a frame as a NumPy array (OpenCV-friendly)
    # frame = picam2.capture_array()
    # frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    # frame = cv2.resize(frame, (160, 120))

    # # Run YOLO inference
    # results = model(frame, stream=True)

    # # Draw detection boxes on the frame
    # for r in results:
    #     annotated = r.plot()  # Draw boxes and labels

    #     # Show frame in a window
    #     cv2.imshow("Ping Pong Detection", annotated)


    # Capture full-res frame
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    h, w = frame.shape[:2]

    # Resize frame for YOLO inference
    small_frame = cv2.resize(frame, (160, 120))
    small_h, small_w = small_frame.shape[:2]

    # Run YOLO inference on small frame
    results = model(small_frame)

    # Scale factor from small frame to full frame
    scale_x = w / small_w
    scale_y = h / small_h

    # Draw detection boxes on full-res frame
    annotated = frame.copy()
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

            # Draw rectangle and label on full-res frame
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{model.names[int(cls)]} {score:.2f}"
            cv2.putText(annotated, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show full-res annotated frame
    cv2.imshow("Ping Pong Detection", annotated)

    # Exit when pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    # time.sleep(1)

# Cleanup
cv2.destroyAllWindows()
picam2.stop()
print("Camera stopped.")