from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from threads.vision.camera_thread import CameraThread
import websocket
import cv2

import time

class IoTThread(PiThread):
    ws: websocket.WebSocket
    server_ws: str
    pi_stream_password: str
    resized_size: tuple[int, int]

    def _on_created_impl(self) -> None:
        # Load settings
        settings = load_settings()["iot_thread"]
        self.server_ws = settings["server_ws"]
        self.pi_stream_password = settings["pi_stream_password"]
        self.resized_size = tuple(settings["resized_size"])

        # Initialize WebSocket
        self.ws = websocket.WebSocket()
        self.connect_ws()

    def _on_start_impl(self) -> None:
        pass

    def _loop_impl(self) -> None:
        # Preprocess frame
        frame = CameraThread["detection.frame"]
        if frame is None:
            return
        frame = cv2.resize(frame, self.resized_size)
        _, jpeg = cv2.imencode(".jpg", frame)

        # Try sending the frame
        try:
            self.ws.send(jpeg.tobytes(), opcode=websocket.ABNF.OPCODE_BINARY)
        except Exception as e:
            self.print("Failed to send frame.")
            self.connect_ws()
        

    def _on_shutdown_impl(self) -> None:
        self.ws.close()

    def connect_ws(self):
        try:
            self.ws.close()
            self.ws.connect(self.server_ws)
            self.ws.send(self.pi_stream_password)
            self.print(f"Reconnecting... pass: {self.pi_stream_password}")
        except Exception as e:
            self.print(f"Failed to connect to server.")