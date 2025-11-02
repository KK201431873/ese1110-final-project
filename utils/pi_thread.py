from abc import ABC, abstractmethod
from typing import Dict, Any, Type
import threading
import time

class PiThread(threading.Thread, ABC):
    def __init__(self, 
                 global_data: Dict[str, Dict[str, Any]],
                 lock: threading.Lock,
                 frequency: int = 100
                ) -> None:
        """
        Creates a singleton thread with name and global data reference.
        Global data is of type Dict[str, Dict[str, Any]]. The outer
        Dict maps thread names to the inner Dict. The inner Dict is
        the shared global data, mapping variable names to their values.
        """
        if frequency <= 0:
            raise ValueError("Frequency must be positive")
        super().__init__(daemon=True, name=self.__class__.__name__)
        self._global_data: Dict[str, Dict[str, Any]] = global_data
        self._lock: threading.Lock = lock
        self._frequency: int = frequency
        self._active: bool = True
        
        # Initialize this thread's global data
        with self._lock:
            self._global_data[self.name] = {}
    
    # --- Abstract main loop ---
    @abstractmethod
    def _loop_impl(self) -> None:
        """Main thread loop implemented by subclass."""
        pass

    # --- Thread API ---
    def run(self) -> None:
        """Runs in a separate thread after .start() is called."""
        next_loop_time = time.perf_counter()
        while self._active:
            try:
                self._loop_impl()
            except Exception as e:
                self.print(f"Error in loop: {e}")
                self._active = False

            now = time.perf_counter()
            blocking_time = next_loop_time - now
            if blocking_time < 0:
                next_loop_time = now
                blocking_time = 0
            
            next_loop_time += 1.0 / self._frequency
            time.sleep(blocking_time)

    def stop(self, join: bool = True) -> None:
        """Set running to False and blocks this thread's execution."""
        self._active = False
        if join and self.is_alive():
            self.join()

    def get_name(self) -> str:
        """Get thread name."""
        return self.name
    
    def get_active(self) -> bool:
        """Get whether thread is active or not."""
        return self._active
    
    def fetch_global_data(self, other_thread: Type) -> Dict[str, Any]:
        """Thread-safely returns a copy of the given thread's data."""
        with self._lock:
            return dict(self._global_data.get(other_thread.__name__, {}))
    
    def fetch_my_global_data(self, key: str) -> Any:
        """Thread-safe read from this thread's own global data."""
        with self._lock:
            return self._global_data[self.name].get(key)
        
    def set_thread_frequency(self, frequency: int) -> None:
        """Set this thread's loop frequency in Hz."""
        if frequency <= 0:
            raise ValueError("Frequency must be positive")
        self._frequency = frequency
    
    def write_global_data(self, key: str, value: Any) -> None:
        """Thread-safe write to this thread's global data."""
        with self._lock:
            self._global_data[self.name][key] = value

    # --- Util Functions ---
    def print(self, s: str) -> None:
        """Thread-safe print with thread name prefix."""
        print(f"[{self.name}] {s}")