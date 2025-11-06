from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.websocket_interface import WebSocketInterface
from threads.vision.camera_thread import CameraThread
import cv2

class IoTThread(PiThread):
    resized_size: tuple[int, int]

    def _on_created_impl(self) -> None:
        # Load settings
        settings = load_settings()["iot_thread"]
        self.resized_size = tuple(settings["resized_size"])

        # Connect to WebSocket
        WebSocketInterface._ensure_socket(self)

    def _on_start_impl(self) -> None:
        pass

    def _loop_impl(self) -> None:
        # Preprocess frame
        frame = CameraThread["detection.frame"]
        if frame is None:
            return
        frame = cv2.resize(frame, self.resized_size)
        WebSocketInterface.send_frame(self, frame)

    def _on_shutdown_impl(self) -> None:
        WebSocketInterface.close()
