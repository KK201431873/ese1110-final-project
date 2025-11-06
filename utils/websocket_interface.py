from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.debug import print_from, raise_error_from
import numpy as np
import threading
import websocket
import cv2

settings = load_settings()["iot_thread"]
server_ws = settings["server_ws"]
pi_stream_password = settings["pi_stream_password"]

class WebSocketInterface():
    _ws: websocket.WebSocket | None = None
    """WebSocket communication channel."""

    _server_ws: str = server_ws
    """WebSocket server address."""

    _pi_stream_password: int = pi_stream_password
    """Password for connecting with Pi Stream route."""

    _lock: threading.Lock = threading.Lock()
    """Lock for accessing WebSocket."""

    @classmethod
    def _ensure_socket(cls, which_thread: type[PiThread] | PiThread | str) -> None:
        """Ensure WebSocket is connected with Flask server."""
        with cls._lock:
            if cls._ws is None or not cls._ws.connected:
                try:
                    # Initialize WebSocket
                    cls._ws = websocket.WebSocket()
                    cls._ws.close()
                    cls._ws.connect(server_ws)
                    cls._ws.send(pi_stream_password)
                    print_from(which_thread, f"Reconnecting... pass: {cls._pi_stream_password}")
                except Exception as e:
                    print_from(
                        which_thread, f"Failed to connect to server {cls._server_ws}: {e}"
                    )

    @classmethod
    def send_frame(cls, which_thread: type[PiThread] | PiThread | str, frame: np.ndarray) -> None:
        """Thread-safe send a frame to the WebSocket server."""
        # Check websocket status
        cls._ensure_socket(which_thread)
        if cls._ws is None:
            raise_error_from(
                which_thread, 
                RuntimeError, f"Failed _ensure_socket() in send_frame()"
            )
            return
        
        # Try sending the frame
        with cls._lock:
            try:
                _, jpeg = cv2.imencode(".jpg", frame)
                cls._ws.send(b"\x01" + jpeg.tobytes(), opcode=websocket.ABNF.OPCODE_BINARY)
            except Exception as e:
                print_from(which_thread, f"Error sending frame: {e}")
                cls._ws = None
                return
        
    @classmethod
    def send_variable(cls, which_thread: type[PiThread] | PiThread | str, name: str, value: str) -> None:
        """Thread-safe send a variable to the WebSocket server."""
        # Variable name cannot contain a colon
        if ":" in name:
            raise_error_from(which_thread, RuntimeError, f"Variable name {name} cannot contain a colon ':'")

        # Check websocket status
        cls._ensure_socket(which_thread)
        if cls._ws is None:
            raise_error_from(
                which_thread, 
                RuntimeError, f"Failed _ensure_socket() in send_variable()"
            )
            return
        
        # Try sending the variable
        with cls._lock:
            try:
                encoded_message = f"{name}:{value}".encode()
                cls._ws.send(b"\x02" + encoded_message, opcode=websocket.ABNF.OPCODE_BINARY)
            except Exception as e:
                print_from(which_thread, f"Error sending variable {name}: {e}")
                cls._ws = None
                return
    
    @classmethod
    def close(cls) -> None:
        """Close the WebSocket connection safely."""
        with cls._lock:
            if cls._ws and cls._ws.connected:
                cls._ws.close()
            cls._ws = None