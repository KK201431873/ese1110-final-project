# read camera images, do apriltag detection, run object detector
from utils.load_settings import load_settings
from utils.pi_thread import PiThread
from picamera2 import Picamera2
import onnxruntime as ort
import numpy.typing as npt
import numpy as np
import math
import time
import cv2

settings = load_settings()["camera_thread"]

# --- Model config ---
model_path: str = settings["model_path"]
frame_size: tuple[int, int] = tuple(settings["frame_size"])
model_input_size: tuple[int, int] = tuple(settings["model_input_size"])
min_score: float = settings["min_score"]
exposure_time: int = settings["exposure_time"]

# --- Pose estimation config---
FOV: float = settings["FOV"]
# fov_x: float = math.radians(settings["fov_x"])
# fov_y: float = math.radians(settings["fov_y"])
lens_incline: float = math.radians(settings["lens_incline"])
lens_height: float = settings["lens_height"]
lens_lateral_offset: float = settings["lens_lateral_offset"]
lens_forward_offset: float = settings["lens_forward_offset"]
lens_angular_offset: float = math.radians(settings["lens_angular_offset"])
pong_ball_diameter: float = settings["pong_ball_diameter"]

class CameraThread(PiThread):
    _model_path: str = model_path
    _model_input_size: tuple[int, int] = model_input_size
    _min_score: float = min_score

    def _on_created_impl(self) -> None:
        # Initialize camera
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": "XRGB8888", "size": frame_size}
        )
        self.picam2.configure(preview_config)
        
        # Initialize ONNX Runtime
        self.session = ort.InferenceSession(self._model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        # Get input size from model
        input_shape = self.session.get_inputs()[0].shape  # e.g., [1, 3, 256, 256]
        self._model_input_size = (input_shape[2], input_shape[3])

        # Detector class names
        self.class_names = ["ping-pong-ball"]  # update with your classes

    def _on_start_impl(self) -> None:
        self.picam2.start()
        time.sleep(1)
        self.picam2.set_controls({
            "AeEnable": False,
            "ExposureTime": exposure_time # microseconds
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
            if score < self._min_score:
                continue
            color = (0, 255, 0)
            label = f"{self.class_names[int(cls)] if int(cls) < len(self.class_names) else 'obj'} {score:.2f}"
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            ball_px = (x1 + x2) / 2
            ball_py = min(y1, y2)
            detection_point = relative_ball_position((ball_px, ball_py))

            if detection_point is None:
                continue

            real_x, real_y = detection_point

            # show ball pos
            cv2.putText(annotated, f"({real_x:.1f}cm, {real_y:.1f}cm)", (x1, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            cv2.circle(annotated, (int(ball_px), int(ball_py)), 5, (0,0,0), cv2.FILLED)

            detection_points.append(detection_point)

        # Share detection data
        self["detection.frame"] = annotated
        self["detection.points"] = detection_points

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """
        1) convert BGR->RGB (ultralytics uses RGB internally)
        2) letterbox to self.input_size (preserve aspect)
        3) normalize to 0..1 float32
        4) CHW and batch dim
        """
        # convert BGR -> RGB
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # letterbox to model size — returns padded square and scale & pad offsets,
        # store pad/scale for mapping back in postprocess (store as instance attrs)
        padded, scale, (pad_x, pad_y) = letterbox(img_rgb, new_shape=self._model_input_size)
        # save for postprocess
        self._last_pre_scale = scale
        self._last_pre_pad = (pad_x, pad_y)

        # normalize to 0..1
        img = padded.astype(np.float32) / 255.0

        # HWC -> CHW
        img = np.transpose(img, (2, 0, 1))

        # add batch
        img = np.expand_dims(img, axis=0).astype(np.float32)

        return img

    def _postprocess(self, outputs: np.ndarray, orig_shape: tuple[int, int]):
        """
        Postprocess for model output layout (1,5,N) -> each detection [x,y,w,h,conf].
        Uses last _last_pre_scale and _last_pre_pad to map back to original image coords.
        """
        # outputs -> (1,5,N) or (5,N) depending on squeeze; normalize to (N,5)
        preds = np.squeeze(outputs)
        if preds.ndim == 2 and preds.shape[0] == 5:   # (5, N)
            preds = preds.T                          # (N, 5)
        elif preds.ndim == 3:  # maybe (1,5,N)
            preds = preds.squeeze().T

        boxes, confidences, class_ids = [], [], []

        input_w, input_h = self._model_input_size
        orig_h, orig_w = orig_shape

        # get scale & pad saved during preprocess
        scale = getattr(self, "_last_pre_scale", None)
        pad_x, pad_y = getattr(self, "_last_pre_pad", (0, 0))

        # If not available, fallback to center scaling (legacy)
        if scale is None:
            scale_x = orig_w / input_w
            scale_y = orig_h / input_h
            pad_x = pad_y = 0
        else:
            # padded coordinates are in padded (input_w,input_h) space
            # to remove padding: (coord - pad) / scale -> original image px
            inv_scale = 1.0 / scale
            scale_x = inv_scale
            scale_y = inv_scale

        for x, y, w, h, conf in preds:
            if conf < self._min_score:
                continue

            # x,y,w,h are in padded model space (0..input_w/input_h)
            # convert center-format padded -> remove pad -> scale back to original image
            cx = (x - pad_x) * scale_x
            cy = (y - pad_y) * scale_y
            bw = w * scale_x
            bh = h * scale_y

            x1 = int(round(cx - bw / 2))
            y1 = int(round(cy - bh / 2))
            x2 = int(round(cx + bw / 2))
            y2 = int(round(cy + bh / 2))

            # clamp
            x1 = max(0, min(orig_w - 1, x1))
            y1 = max(0, min(orig_h - 1, y1))
            x2 = max(0, min(orig_w - 1, x2))
            y2 = max(0, min(orig_h - 1, y2))

            boxes.append([x1, y1, x2 - x1, y2 - y1])  # for NMSBoxes
            confidences.append(float(conf))
            class_ids.append(0)

        # Apply NMS
        final_boxes, final_confs, final_cls = [], [], []
        if boxes:
            indices = cv2.dnn.NMSBoxes(boxes, confidences, self._min_score, 0.45)
            if len(indices) > 0:
                indices = np.array(indices).flatten()
                for i in indices:
                    x, y, w, h = boxes[i]
                    final_boxes.append((x, y, x + w, y + h))
                    final_confs.append(confidences[i])
                    final_cls.append(class_ids[i])

        return final_boxes, final_confs, final_cls

    def _on_shutdown_impl(self) -> None:
        self.picam2.stop()

def letterbox(img: np.ndarray, new_shape=(256, 256), color=(114,114,114)):
    """
    Resize image to new_shape while keeping aspect ratio, pad with `color`.
    Returns: resized_padded_img, scale, (pad_left, pad_top)
    This mirrors the typical YOLO letterbox behaviour.
    """
    orig_h, orig_w = img.shape[:2]
    new_w, new_h = new_shape
    # compute scale
    scale = min(new_w / orig_w, new_h / orig_h)
    # compute new unpadded size
    resized_w, resized_h = int(round(orig_w * scale)), int(round(orig_h * scale))
    # resize
    resized = cv2.resize(img, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    # compute padding
    pad_w = new_w - resized_w
    pad_h = new_h - resized_h
    pad_left = pad_w // 2
    pad_right = pad_w - pad_left
    pad_top = pad_h // 2
    pad_bottom = pad_h - pad_top
    padded = cv2.copyMakeBorder(resized, pad_top, pad_bottom, pad_left, pad_right,
                                borderType=cv2.BORDER_CONSTANT, value=color)
    return padded, scale, (pad_left, pad_top)

def relative_ball_position(ball_point: tuple[float, float]) -> tuple[float, float] | None:
    """
    Estimates the ping-pong ball's position relative to the robot given the 
    midpoint of its bounding box's top edge. Returns (x, y) in cm, in accordance
    with the robot's coordinate system defined in settings.yaml.
    """
    # Get pixel coordinates of midpoint of top edge of bounding box.
    px, py = ball_point

    # Get camera frame midpoint
    mid_x, mid_y = frame_size[0]/2, frame_size[1]/2

    # Back-solve for projected coordinates
    c_px = (px - mid_x) / FOV;
    c_py = (mid_y - py) / FOV;

    print(c_px, c_py)

    projectedX_numer = -c_py*math.sin(lens_incline) - math.cos(lens_incline);
    projectedX_denom = c_py*math.cos(lens_incline) - math.sin(lens_incline);
    if (projectedX_denom == 0): 
        return None

    height_above_ball = lens_height - pong_ball_diameter
    projectedX = height_above_ball * projectedX_numer / projectedX_denom;
    projectedY = -(projectedX*math.cos(lens_incline) + height_above_ball*math.sin(lens_incline)) * c_px;

    return (projectedX, projectedY)

# import math

# def relative_ball_position(ball_point: tuple[float, float]) -> tuple[float, float] | None:
#     px, py = ball_point
#     mid_x, mid_y = frame_size[0] / 2, frame_size[1] / 2

#     # Convert pixel offsets to tangent of angles
#     c_px = math.tan((px - mid_x) / frame_size[0] * fov_x)
#     c_py = math.tan((mid_y - py) / frame_size[1] * fov_y)

#     h = lens_height - pong_ball_diameter

#     # Compute ray intersection with ground
#     down_angle = lens_incline + math.atan(c_py)
#     if abs(math.tan(down_angle)) < 1e-6:
#         return None

#     X_cam = h / math.tan(down_angle)
#     Y_cam = X_cam * c_px

#     return (X_cam, Y_cam)
