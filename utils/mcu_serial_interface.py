from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.debug import print_from, raise_error_from
from serial import SerialTimeoutException
import serial
import threading
import time

settings = load_settings()["mcu_serial_interface"]
baudrate: int = settings["baudrate"]

class _SerialChannel:
    def __init__(self, port: str, baudrate: int):
        self._port = port
        """Path ending with an MCU ID."""

        self._baudrate = baudrate
        """Sync with MCUs, usually 115200."""

        self._ser: serial.Serial | None = None
        """Serial communication channel."""

        self._lock = threading.RLock()
        """Reentrant lock for accessing serial channel."""

        self._buffer = b""
        """Accumulate bytes until a newline is read"""

        self._ensure_open("_SerialChannel")

    def _ensure_open(self, which_thread: type[PiThread] | PiThread | str) -> None:
        """Ensure serial port is open."""
        with self._lock:
            if self._ser is None or not self._ser.is_open:
                try:
                    self._ser = serial.Serial(port=self._port,
                                            baudrate=self._baudrate,
                                            timeout=0,
                                            write_timeout=1)  # non-blocking
                    self._buffer = b""
                    time.sleep(0.1)
                except serial.SerialException as e:
                    raise_error_from(
                        which_thread, 
                        RuntimeError, f"Failed to open serial port {self._port}: {e}"
                    )

    def read_lines(self, 
                   which_thread: type[PiThread] | PiThread | str, 
                   max_lines: int = 10) -> list[str]:
        """Read up to `max_lines` complete lines from peripheral MCU (ending with '\n')."""
        lines: list[str] = []
        with self._lock:
            # Check serial channel status
            self._ensure_open(which_thread)
            if self._ser is None:
                raise_error_from(which_thread, RuntimeError, f"Failed _ensure_open() in read_lines() for port {self._port}")
                return []
            
            # Try reading data
            try:
                # Read all available bytes (non-blocking)
                available = self._ser.in_waiting
                if available > 0:
                    self._buffer += self._ser.read(available)

                # Split into lines based on newline terminator
                self._buffer = self._buffer.replace(b"\r", b"")
                if b"\n" in self._buffer:
                    parts = self._buffer.split(b"\n")
                    complete_lines = parts[:-1]  # all full lines
                    self._buffer = parts[-1]      # leftover partial line

                    for raw_line in complete_lines:
                        decoded = raw_line.decode(errors="ignore").strip()
                        if decoded:
                            lines.append(decoded)
                            if len(lines) >= max_lines:
                                break

            except Exception as e:
                print_from(which_thread, f"Error reading from serial: {e}")
                self._ser = None
                self._buffer = b""
                return []

        return lines
        
    def write_line(self, which_thread: type[PiThread] | PiThread | str, data: str) -> None:
        """Thread-safe write a line to actuator MCU serial channel."""
        with self._lock:
            # Check serial channel status
            self._ensure_open(which_thread)
            if self._ser is None:
                raise_error_from(which_thread, RuntimeError, f"Failed _ensure_open() in write_line()")
                return
            
            # Try writing data
            try:
                self._ser.write((data + "\n").encode())
            except SerialTimeoutException as e:
                raise_error_from(which_thread, SerialTimeoutException, f"Serial write timed out! (is MCU connected?): {e}")
            except Exception as e:
                print_from(which_thread, f"Error writing to serial: {e}")
                self._ser = None  # force reinit on next read/write

    def close(self) -> None:
        """Close the serial port safely."""
        with self._lock:
            if self._ser and self._ser.is_open:
                self._ser.close()
            self._ser = None
    


class MCUSerialInterface():
    _peripheral: _SerialChannel | None = None
    """Peripheral MCU reads & sends sensor data, Pi receives it here."""

    _actuator: _SerialChannel | None = None
    """Pi sends commands to actuator MCU, which controls motors & servos."""

    _init_lock = threading.Lock()
    """Lock for initializing serial channels."""

    @classmethod
    def _ensure_init(cls):
        with cls._init_lock:
            if cls._peripheral is None:
                cls._peripheral = _SerialChannel(settings["sensor_port"], baudrate)
            if cls._actuator is None:
                cls._actuator = _SerialChannel(settings["actuator_port"], baudrate)

    @classmethod
    def read_lines(cls, which_thread: type[PiThread] | PiThread | str, max_lines: int = 5) -> list[str]:
        cls._ensure_init()
        if cls._peripheral:
            return cls._peripheral.read_lines(which_thread, max_lines)
        else:
            return []

    @classmethod
    def write_line(cls, which_thread: type[PiThread] | PiThread | str, data: str) -> None:
        cls._ensure_init()
        if cls._actuator:
            cls._actuator.write_line(which_thread, data)

    @classmethod
    def close_all(cls) -> None:
        cls._ensure_init()
        if cls._peripheral:
            cls._peripheral.close()
        if cls._actuator:
            cls._actuator.close()