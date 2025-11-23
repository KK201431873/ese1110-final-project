from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.websocket_interface import WebSocketInterface
from threads.vision.inference_thread import InferenceThread
import cv2

class IoTCameraFeedThread(PiThread):
    _resized_size: tuple[int, int]

    def _on_created_impl(self) -> None:
        # Load settings
        settings = load_settings()["iot_camera_feed_thread"]
        self._resized_size = tuple(settings["resized_size"])

        # # Connect to WebSocket
        # WebSocketInterface._ensure_socket(self)

    def _on_start_impl(self) -> None:
        self.print("Alive!")

    def _loop_impl(self) -> None:
        # Preprocess and send detection frame through WebSocket
        detection_frame = InferenceThread["detection.frame"]
        if detection_frame is None:
            return
        detection_frame = cv2.resize(detection_frame, self._resized_size)
        WebSocketInterface.send_frame(self, detection_frame)

    def _on_shutdown_impl(self) -> None:
        WebSocketInterface.close()
