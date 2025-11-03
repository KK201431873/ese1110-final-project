from utils import PiThread
import threading
import serial

class ArduinoSerialInterface():
    _serial_port: str = "/dev/ttyACM0"
    """USB on Raspberry Pi that Arduino is connected to."""

    _baudrate: int = 9600
    """Sync with Arduino."""

    _ser: serial.Serial | None = None
    """Serial communication channel."""

    _lock: threading.Lock = threading.Lock()
    """Lock for accessing serial channel."""

    @classmethod
    def _ensure_serial(cls, which_thread: type[PiThread] | PiThread) -> None:
        """Ensure serial port is open."""
        with cls._lock:
            if cls._ser is None or not cls._ser.is_open:
                try:
                    cls._ser = serial.Serial(port=cls._serial_port,
                                            baudrate=cls._baudrate,
                                            timeout=1)
                except serial.SerialException as e:
                    cls._raise_error_from(
                        which_thread, 
                        RuntimeError, f"Failed to open serial port {cls._serial_port}: {e}"
                    )

    @classmethod
    def read_line(cls, which_thread: type[PiThread] | PiThread) -> str:
        """Thread-safe read a line from Arduino serial channel."""
        # Check serial channel status
        cls._ensure_serial(which_thread)
        if cls._ser is None:
            cls._raise_error_from(
                which_thread, 
                RuntimeError, f"Failed _ensure_serial() in read_line()"
            )
            return ""
        
        # Try reading data
        with cls._lock:
            try:
                data = cls._ser.readline().decode(errors="ignore").strip()
            except Exception as e:
                cls._print_from(which_thread, f"Error reading from serial: {e}")
                cls._ser = None  # force reinit on next read/write
                data = ""
            return data
        
    @classmethod
    def write_line(cls, which_thread: type[PiThread] | PiThread, data: str) -> None:
        """Thread-safe write a line to Arduino serial channel."""
        # Check serial channel status
        cls._ensure_serial(which_thread)
        if cls._ser is None:
            cls._raise_error_from(
                which_thread, 
                RuntimeError, f"Failed _ensure_serial() in write_line()"
            )
            return
        
        # Try writing data
        with cls._lock:
            try:
                cls._ser.write((data + "\n").encode())
            except Exception as e:
                cls._print_from(which_thread, f"Error writing to serial: {e}")
                cls._ser = None  # force reinit on next read/write
    
    @classmethod
    def _print_from(cls, which_thread: type[PiThread] | PiThread, *values: object) -> None:
        """Thread-safe print with thread name prefix."""
        if isinstance(which_thread, PiThread):
            which_thread.print(*values)
        elif isinstance(which_thread, type) and issubclass(which_thread, PiThread):
            which_thread.print_cls(*values)
        else:
            print("[ArduinoSerialInterface]", *values)

    @classmethod
    def _raise_error_from(cls, which_thread: type[PiThread] | PiThread, exc_type: type[Exception], message: str) -> None:
        """Raise error with thread name prefix."""
        if isinstance(which_thread, PiThread):
            which_thread.raise_error(exc_type, message)
        elif isinstance(which_thread, type) and issubclass(which_thread, PiThread):
            which_thread.raise_error_cls(exc_type, message)
        else:
            raise exc_type(f"[ArduinoSerialInterface] {message}")
    
    @classmethod
    def close(cls):
        """Close the serial port safely."""
        with cls._lock:
            if cls._ser and cls._ser.is_open:
                cls._ser.close()
            cls._ser = None