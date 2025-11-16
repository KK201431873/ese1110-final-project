from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.debug import print_from, raise_error_from
import numpy as np
import threading
import websocket
import cv2

settings = load_settings()["websocket_interface"]
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

    _connect_thread: threading.Thread | None = None
    _connecting: bool = False

    @classmethod
    def _connect_in_background(cls, which_thread: type[PiThread] | PiThread | str):
        """Background thread function to connect to WebSocket."""
        if PiThread.has_crashed():
            return
        
        with cls._lock:
            if cls._connecting:
                return
            cls._connecting = True

        try:
            print_from(which_thread, f"Connecting to {cls._server_ws}...")
            ws = websocket.WebSocket()
            ws.connect(cls._server_ws, timeout=2)

            if PiThread.has_crashed():
                return
            
            ws.send(str(cls._pi_stream_password))

            if PiThread.has_crashed():
                return

            with cls._lock:
                cls._ws = ws
            print_from(which_thread, f"Connected successfully to {cls._server_ws}")
        except Exception as e:
            print_from(which_thread, f"Connection failed: {e}")
            with cls._lock:
                cls._ws = None
        finally:
            with cls._lock:
                cls._connecting = False

    @classmethod
    def _ensure_socket(cls, which_thread: type[PiThread] | PiThread | str) -> None:
        """Ensure WebSocket is connected with Flask server (async version)."""
        with cls._lock:
            ws_connected = cls._ws is not None and cls._ws.connected
            connecting = cls._connecting

        if ws_connected:
            return  # Already connected

        # If not connected and not currently connecting, start a background thread
        if not connecting:
            cls._connect_thread = threading.Thread(
                target=cls._connect_in_background,
                args=(which_thread,),
                daemon=True
            )
            cls._connect_thread.start()

    @classmethod
    def _safe_send(cls, which_thread, prefix: bytes, payload: bytes) -> None:
        """Helper to safely send data to the WebSocket if connected."""
        with cls._lock:
            ws = cls._ws
            connecting = cls._connecting

        if ws is None or not ws.connected:
            if not connecting:
                print_from(which_thread, "WebSocket not connected — skipping send")
            return

        try:
            ws.send(prefix + payload, opcode=websocket.ABNF.OPCODE_BINARY)
        except Exception as e:
            print_from(which_thread, f"Send failed: {e}")
            with cls._lock:
                cls._ws = None

    # === Public send functions ===

    @classmethod
    def send_frame(cls, which_thread: type[PiThread] | PiThread | str, frame: np.ndarray) -> None:
        """Thread-safe send a frame to the WebSocket server."""
        cls._ensure_socket(which_thread)
        try:
            _, jpeg = cv2.imencode(".jpg", frame)
            cls._safe_send(which_thread, b"\x01", jpeg.tobytes())
        except Exception as e:
            print_from(which_thread, f"send_frame error: {e}")
        
    @classmethod
    def send_variable(cls, which_thread: type[PiThread] | PiThread | str, name: str, value: str) -> None:
        """Thread-safe send a variable to the WebSocket server."""
        # Variable name cannot contain a colon
        if ":" in name:
            raise_error_from(which_thread, RuntimeError, f"Variable name {name} cannot contain a colon ':'")
        
        # Try sending the variable
        cls._ensure_socket(which_thread)
        encoded = f"{name}:{value}".encode()
        cls._safe_send(which_thread, b"\x02", encoded)
    
    @classmethod
    def send_minimap(cls, which_thread: type[PiThread] | PiThread | str, minimap: np.ndarray) -> None:
        """Thread-safe send the minimap image to the WebSocket server."""
        cls._ensure_socket(which_thread)
        try:
            _, jpeg = cv2.imencode(".jpg", minimap)
            cls._safe_send(which_thread, b"\x03", jpeg.tobytes())
        except Exception as e:
            print_from(which_thread, f"send_minimap error: {e}")
    
    @classmethod
    def close(cls):
        """Close the WebSocket connection safely."""
        with cls._lock:
            if cls._ws and cls._ws.connected:
                try:
                    cls._ws.close()
                except:
                    pass
            cls._ws = None
            cls._connecting = False