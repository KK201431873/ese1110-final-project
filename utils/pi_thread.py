from abc import ABCMeta, abstractmethod
from typing import Any, TypeAlias
import threading
import time

DataValue: TypeAlias = Any | None
ThreadData: TypeAlias = dict[str, DataValue]

class PiThreadMeta(ABCMeta):
    """Custom metaclass to enable class-level __getitem__ and __setitem__ functionality."""

    def __getitem__(cls, key: str) -> DataValue:
        """Thread-safe read from global data value using key."""
        with PiThread._lock:
            return PiThread._global_data.get(cls.__name__, {}).get(key, None)

    def __setitem__(cls, key: str, value: DataValue) -> None:
        """Thread-safe write a key-value pair to global data."""
        with PiThread._lock:
            if cls.__name__ not in PiThread._global_data:
                PiThread._global_data[cls.__name__] = {}
            PiThread._global_data[cls.__name__][key] = value
    
    def __contains__(cls, key: str) -> bool:
        """Return True if key exists in this thread's global data."""
        with PiThread._lock:
            return key in PiThread._global_data.get(cls.__name__, {})


class PiThread(threading.Thread, metaclass=PiThreadMeta):
    """
    Base class for singleton threads with shared, class-scoped global data.

    Each subclass can expose and access its own global data via class or instance syntax:
        CameraThread["frame"] = image
        frame = CameraThread["frame"]
        self["status"] = "ready"
        status = self["status"]

    All subclasses share one synchronized global data registry but each class has its own namespace.
    """

    _instance_registry: dict[str, bool] = {}
    """Track singleton instances, mapping subclass names to instance existence."""

    _global_data: dict[str, ThreadData] = {}
    """Shared global data across all threads."""

    _lock: threading.Lock = threading.Lock()
    """Lock for accessing global data."""

    def __init__(self, 
                 frequency: int = 100,
                 exit_on_error: bool = True,
                 raise_on_error: bool = True
                ) -> None:
        """
        Creates a singleton thread with name and global data reference.
        Global data is of type Dict[str, Dict[str, Any]]. The outer
        Dict maps thread names to the inner Dict. The inner Dict is
        the shared global data, mapping variable names to their values.
        """
        class_name = self.__class__.__name__

        # Enforce singleton invariant
        with PiThread._lock:
            if class_name in PiThread._instance_registry and PiThread._instance_registry[class_name]:
                self.raise_error(RuntimeError, f"Only one instance of {class_name} allowed")
            PiThread._instance_registry[class_name] = True
            PiThread._global_data[class_name] = {}
        
        # Create thread
        super().__init__(daemon=True, name=class_name)
        self.set_thread_frequency(frequency)
        self._exit_on_error: bool = exit_on_error
        self._raise_on_error: bool = raise_on_error
        self._alive: bool = False
        self._on_created_impl()
    
    # --- Initialization and thread life cycle ---
    @abstractmethod
    def _on_created_impl(self) -> None:
        """
        Initialization actions implemented by subclass. Called at the
        end of __init__, before thread starts.
        """
        pass

    @abstractmethod
    def _on_start_impl(self) -> None:
        """
        Actions to perform when thread starts, implemented by subclass.
        Called once at the start of run(), inside new thread.
        """
        pass

    @abstractmethod
    def _loop_impl(self) -> None:
        """
        Main thread loop implemented by subclass. Called repeatedly
        in runtime at target frequency.
        """
        pass

    @abstractmethod
    def _on_shutdown_impl(self) -> None:
        """Optional cleanup before thread exits."""
        pass

    def run(self) -> None:
        """Runs in a separate thread after .start() is called."""
        # First loop logic
        if not self._alive:
            try:
                self._on_start_impl()
                self._alive = True
            except Exception as e:
                if self._exit_on_error:
                    try:
                        self.kill()
                    except Exception as shutdown_err:
                        self.print(f"Error in shutdown after start failure: {shutdown_err}")
                        self._alive = False
                        self.join()
                if self._raise_on_error:
                    self.raise_error(RuntimeError, f"Error on start: {e}")
                else:
                    self.print(f"Error on start: {e}")

        # Main loop logic
        next_loop_time = time.perf_counter()
        while self._alive:
            try:
                self._loop_impl()
            except Exception as e:
                if self._exit_on_error:
                    self.kill()
                if self._raise_on_error:
                    self.raise_error(RuntimeError, f"Error in loop: {e}")
                else:
                    self.print(f"Error in loop: {e}")

            now = time.perf_counter()
            blocking_time = next_loop_time - now
            if blocking_time < 0:
                next_loop_time = now
                blocking_time = 0
            
            next_loop_time += 1.0 / self._frequency
            if blocking_time > 0:
                # Don't call sleep(0)
                time.sleep(blocking_time)

    def kill(self, join: bool = True) -> None:
        """Permanently stop this thread. Cannot be restarted or recreated."""
        self._on_shutdown_impl()
        self._alive = False
        if join and self.is_alive():
            self.join()
        
    # --- Thread API ---
    def set_thread_frequency(self, frequency: int) -> None:
        """Set this thread's loop frequency in Hz."""
        if frequency <= 0:
            self.raise_error(ValueError, "Frequency must be positive")
        self._frequency = frequency

    def get_name(self) -> str:
        """Get thread name."""
        return self.name
    
    def get_alive(self) -> bool:
        """Get whether thread is alive or not."""
        return self._alive
    
    def __setitem__(self, key: str, value: DataValue) -> None:
        """Thread-safe write a key-value pair to global data."""
        with PiThread._lock:
            if self.name not in PiThread._global_data:
                PiThread._global_data[self.name] = {}
            PiThread._global_data[self.name][key] = value
    
    def __getitem__(self, key: str) -> DataValue:
        """Thread-safe read from this thread's global data value using key."""
        with PiThread._lock:
            return PiThread._global_data.get(self.name, {}).get(key, None)
    
    def __contains__(self, key: str) -> bool:
        """Thread-safe check if key exists in this thread's global data."""
        with PiThread._lock:
            return key in PiThread._global_data.get(self.name, {})
    
    @classmethod
    def get_global_data(cls) -> dict[str, Any]:
        """Return this thread’s entire global data dictionary (read-only)."""
        with PiThread._lock:
            return dict(PiThread._global_data.get(cls.__name__, {}))

    # --- Debug Functions ---
    def print(self, *values: object) -> None:
        """Thread-safe print with thread name prefix."""
        print(f"[{self.name}]", *values)
    
    def raise_error(self, exc_type: type[Exception], message: str) -> None:
        """Raise an exception with thread name prefix."""
        raise exc_type(f"[{self.name}] {message}")
