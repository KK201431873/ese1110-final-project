from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.pose2 import Pose2
from utils.vector2 import Vector2
from threads.vision.camera_thread import CameraThread
from threads.peripherals.sensor_thread import SensorThread
import numpy.typing as npt
import onnxruntime as ort
import numpy as np
import math
import cv2

settings = load_settings()
inference_settings = settings["inference_thread"]

# --- Pi camera frame size ---
frame_size: tuple[int, int] = tuple(settings["camera_thread"]["frame_size"])

# --- Model config ---
model_path: str = inference_settings["model_path"]
model_input_size: tuple[int, int] = tuple(inference_settings["model_input_size"])
min_score: float = inference_settings["min_score"]

# --- Pose estimation config---
FOV: float = inference_settings["FOV"]
fov_x: float = math.radians(inference_settings["fov_x"])
fov_y: float = math.radians(inference_settings["fov_y"])
lens_incline: float = math.radians(inference_settings["lens_incline"])
lens_height: float = inference_settings["lens_height"]
lens_x_offset: float = inference_settings["lens_x_offset"]
lens_y_offset: float = inference_settings["lens_y_offset"]
lens_angular_offset: float = math.radians(inference_settings["lens_angular_offset"])
pong_ball_diameter: float = inference_settings["pong_ball_diameter"]

class InferenceThread(PiThread):
    """
    Runs ONNX inference asynchronously on frames pushed by CameraThread.
    """
    ROBOT_POSE: Pose2 | None
    
    _model_path: str = model_path
    _model_input_size: tuple[int, int] = model_input_size
    _min_score: float = min_score

    # Camera freeze detection
    _last_frame_checksum: int = 0

    def _on_created_impl(self) -> None:
        
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
        self.print("Alive!")

    def _loop_impl(self) -> None:
        # Get robot pose
        self.ROBOT_POSE = SensorThread["localization.pose"]

        # Get frame and run inference
        frame, ok = self._get_frame()
        if (not ok) or (frame is None):
            self["detection.frame"] = None
            detection_points: dict[str, list[Vector2]] = { "relative_points": [], "absolute_points": [] }
            self["detection.points"] = detection_points
            return

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
        relative_points: list[Vector2] = []
        absolute_points: list[Vector2] = []
        for (x1, y1, x2, y2), score, cls in zip(boxes, confidences, class_ids):
            if score < self._min_score:
                continue
            
            # Calculate relative ball position
            ball_px = (x1 + x2) / 2
            ball_py = min(y1, y2)
            relative_pos: Vector2 | None = self.relative_ball_position((ball_px, ball_py))
            if relative_pos is None:
                continue
            relative_points.append(relative_pos)

            real_x = relative_pos.x
            real_y = relative_pos.y
            if real_x <= 0:
                continue

            # Calculate absolute ball position
            absolute_pos = self.absolute_ball_position(relative_pos)
            if absolute_pos is not None:
                absolute_points.append(absolute_pos)

            # Annotate frame
            color = (0, 255, 0)
            label = f"{self.class_names[int(cls)] if int(cls) < len(self.class_names) else 'obj'} {score:.2f}"
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            cv2.putText(annotated, label, (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(annotated, f"({real_x:.1f}m, {real_y:.1f}m)", (x1, y2 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            cv2.circle(annotated, (int(ball_px), int(ball_py)), 5, (0,0,0), cv2.FILLED)

        # Share detection data
        self["detection.frame"] = annotated
        detection_points: dict[str, list[Vector2]] = {
            "relative_points": relative_points,
            "absolute_points": absolute_points
        }
        self["detection.points"] = detection_points

    def _get_frame(self) -> tuple[npt.NDArray, bool]:
        """Returns the camera's frame and whether or not the returned frame should be used."""
        frame: npt.NDArray | None = CameraThread["frame"]
        if frame is None:
            return np.zeros((1)), False

        # Check if camera froze
        checksum = int(np.sum(frame))
        ok = checksum != self._last_frame_checksum
        self._last_frame_checksum = checksum
        return frame, ok

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
        padded, scale, (pad_x, pad_y) = self.letterbox(img_rgb, new_shape=self._model_input_size)
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

    def letterbox(self, img: np.ndarray, new_shape=(256, 256), color=(114,114,114)):
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

    def relative_ball_position(self, ball_point: tuple[float, float]) -> Vector2 | None:
        px, py = ball_point
        mid_x, mid_y = frame_size[0] / 2, frame_size[1] / 2

        # Convert pixel offsets to tangent of angles
        c_px = math.tan((px - mid_x) / frame_size[0] * fov_x)
        c_py = math.tan((py - mid_y) / frame_size[1] * fov_y)   # <-- flipped sign

        h = lens_height - pong_ball_diameter

        # Compute ray intersection with ground
        down_angle = lens_incline + math.atan(c_py)
        if abs(math.tan(down_angle)) < 1e-6:
            return None

        X_cam = h / math.tan(down_angle)
        Y_cam = -X_cam * c_px

        return Vector2(X_cam, Y_cam)

    def absolute_ball_position(self, relative_pos: Vector2) -> Vector2 | None:
        if self.ROBOT_POSE is None:
            return None

        # Copy the relative position
        absolute_pos: Vector2 = Vector2(relative_pos.x, relative_pos.y)

        # Correct for camera angle
        absolute_pos = absolute_pos.rotate(lens_angular_offset)

        # Correct for camera translation
        absolute_pos = absolute_pos + Vector2(lens_x_offset, lens_y_offset)

        # Correct for robot heading
        absolute_pos = absolute_pos.rotate(self.ROBOT_POSE.h)

        # Correct for robot translation
        absolute_pos = absolute_pos + self.ROBOT_POSE.COORDS

        return absolute_pos
        
    def _on_shutdown_impl(self) -> None:
        return