from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from threads.vision.camera_thread import CameraThread
import websocket
import cv2

class IoTThread(PiThread):
    ws: websocket.WebSocket
    server_ws: str
    resized_size: tuple[int, int]

    def _on_created_impl(self) -> None:
        # Load settings
        settings = load_settings()["iot_thread"]
        self.server_ws = settings["server_ws"]
        self.resized_size = tuple(settings["resized_size"])

        # Initialize WebSocket
        self.ws = websocket.WebSocket()
        self.ws.connect(self.server_ws)

    def _on_start_impl(self) -> None:
        pass

    def _loop_impl(self) -> None:
        frame = CameraThread["detection.frame"]
        if frame is None:
            return
        frame = cv2.resize(frame, self.resized_size)
        _, jpeg = cv2.imencode(".jpg", frame)
        self.ws.send(jpeg.tobytes(), opcode=websocket.ABNF.OPCODE_BINARY)

    def _on_shutdown_impl(self) -> None:
        self.ws.close()