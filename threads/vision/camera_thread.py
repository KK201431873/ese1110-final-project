from utils.load_settings import load_settings
from utils.pi_thread import PiThread
from picamera2 import Picamera2
import time
import cv2

settings = load_settings()["camera_thread"]
frame_size: tuple[int, int] = tuple(settings["frame_size"])
exposure_time: int = settings["exposure_time"]

class CameraThread(PiThread):
    def _on_created_impl(self) -> None:
        # Initialize camera
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": "XRGB8888", "size": frame_size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()

    def _on_start_impl(self) -> None:
        # Wait until camera is warmed up
        exposure_set_start_time = time.perf_counter()
        while True:
            try:
                frame = self.picam2.capture_metadata()
                if frame is not None:
                    break
            except Exception:
                pass
            if time.perf_counter() - exposure_set_start_time > 1:
                self.raise_error(RuntimeError, "Failed to set camera exposure.")
            time.sleep(0.05)
        
        # Set exposure
        self.picam2.set_controls({
            "AeEnable": False,
            "ExposureTime": exposure_time
        })

        # Telemetry
        self.print("Alive!")

    def _loop_impl(self) -> None:
        # Capture frame and update global state
        frame = cv2.cvtColor(self.picam2.capture_array(), cv2.COLOR_BGRA2BGR)
        self["frame"] = frame

    def _on_shutdown_impl(self) -> None:
        self.picam2.stop()