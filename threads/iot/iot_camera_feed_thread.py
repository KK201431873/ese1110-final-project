from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.websocket_interface import WebSocketInterface
from threads.vision.camera_thread import CameraThread
import cv2

class IoTCameraFeedThread(PiThread):
    _resized_size: tuple[int, int]

    def _on_created_impl(self) -> None:
        # Load settings
        settings = load_settings()["iot_camera_feed_thread"]
        self._resized_size = tuple(settings["resized_size"])

        # Connect to WebSocket
        WebSocketInterface._ensure_socket(self)

    def _on_start_impl(self) -> None:
        self.print("Alive!")

    def _loop_impl(self) -> None:
        # Preprocess and send camera frame through WebSocket
        frame = CameraThread["detection.frame"]
        if frame is None:
            return
        frame = cv2.resize(frame, self._resized_size)
        WebSocketInterface.send_frame(self, frame)

    def _on_shutdown_impl(self) -> None:
        WebSocketInterface.close()
