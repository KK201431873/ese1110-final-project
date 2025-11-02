from abc import ABC, abstractmethod
from typing import Dict, Any
import threading

class PiThread(ABC, threading.Thread):
    def __init__(self, 
                 name: str,
                 global_data: Dict[str, Dict[str, Any]],
                 lock: threading.Lock
                ) -> None:
        """
        Creates a new thread with name and global data reference.
        Global data is of type Dict[str, Dict[str, Any]]. The outer
        Dict maps thread names to the inner Dict. The inner Dict is
        the shared global data, mapping variable names to their values.
        """
        super().__init__(daemon=True)
        self._name: str = name
        self._global_data: Dict[str, Dict[str, Any]] = global_data
        self._lock: threading.Lock = lock
        self._active: bool = True
        
        # Initialize this thread's global data
        with self._lock:
            self._global_data[self._name] = {}
    
    # --- Abstract main loop ---
    @abstractmethod
    def _loop_impl(self) -> None:
        """Main thread loop implemented by subclass"""
        pass

    # --- Thread API ---
    def run(self) -> None:
        """Runs in a separate thread after .start() is called."""
        while self._active:
            self._loop_impl()

    def stop(self) -> None:
        """Set running to False"""
        self._active = False

    def get_name(self) -> str:
        """Get thread name"""
        return self._name
    
    def get_active(self) -> bool:
        """Get whether thread is active or not"""
        return self._active
    
    def fetch_global_data(self, name: str) -> Dict[str, Any]:
        """Read the given thread's data (thread-safe)"""
        with self._lock:
            return dict(self._global_data.get(name, {}))
    
    def write_global_data(self, key: str, value: Any) -> None:
        """Write to this thread's global data (thread-safe)"""
        with self._lock:
            self._global_data[self._name][key] = value

    # --- Util Functions ---
    def print(self, s: str) -> None:
        print(f"[{self._name}] {s}")