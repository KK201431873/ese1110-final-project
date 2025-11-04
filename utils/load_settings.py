import yaml
from pathlib import Path

_settings_cache = None

def load_settings() -> dict:
    """
    Loads settings.yaml from the project root (same dir as main.py).
    Uses caching to avoid reloading every time.
    """
    global _settings_cache
    if _settings_cache is not None:
        return _settings_cache

    # Find the project root — assumed to be where main.py is
    root_dir = Path(__file__).resolve().parent.parent
    settings_path = root_dir / "settings.yaml"

    if not settings_path.exists():
        raise FileNotFoundError(f"settings.yaml not found at {settings_path}")

    with open(settings_path, "r") as f:
        _settings_cache = yaml.safe_load(f)

    return _settings_cache
