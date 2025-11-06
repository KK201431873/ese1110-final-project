from utils.pi_thread import PiThread

def print_from(which_thread: type[PiThread] | PiThread | str, *values: object) -> None:
    """Thread-safe print with thread name prefix."""
    if isinstance(which_thread, str):
        print(f"[{which_thread}]", *values)
    elif isinstance(which_thread, PiThread):
        which_thread.print(*values)
    elif isinstance(which_thread, type) and issubclass(which_thread, PiThread):
        which_thread.print_cls(*values)
    else:
        print("[Debug]", *values)

def raise_error_from(which_thread: type[PiThread] | PiThread | str, exc_type: type[Exception], message: str) -> None:
    """Raise error with thread name prefix."""
    if isinstance(which_thread, str):
        raise exc_type(f"[{which_thread}] {message}")
    elif isinstance(which_thread, PiThread):
        which_thread.raise_error(exc_type, message)
    elif isinstance(which_thread, type) and issubclass(which_thread, PiThread):
        which_thread.raise_error_cls(exc_type, message)
    else:
        raise exc_type(f"[Debug] {message}")
    
__all__ = ["print_from", "raise_error_from"]