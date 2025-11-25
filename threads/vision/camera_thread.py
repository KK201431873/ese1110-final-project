from utils.load_settings import load_settings
from utils.pi_thread import PiThread
from picamera2 import Picamera2
import numpy as np
import time
import cv2

settings = load_settings()["camera_thread"]
frame_size: tuple[int, int] = tuple(settings["frame_size"])
exposure_time: int = settings["exposure_time"]

class CameraThread(PiThread):
    def _on_created_impl(self) -> None:
        # Initialize camera
        self.picam2 = Picamera2()
        self.preview_config = self.picam2.create_preview_configuration(
            main={"format": "XRGB8888", "size": frame_size}
        )

    def _on_start_impl(self) -> None:
        self.picam2.configure(self.preview_config)
        self.picam2.start()

        # Wait until camera is warmed up
        time.sleep(0.2)
        self.picam2.capture_metadata()
        
        # Set exposure
        self.picam2.set_controls({"AeEnable": False})
        time.sleep(0.05)
        self.picam2.set_controls({"ExposureTime": exposure_time})

        # Telemetry
        self.print("Alive!")

    def _loop_impl(self) -> None:
        # Capture frame and update global state
        try:
            req = self.picam2.capture_request()
        except Exception as e:
            self.raise_error(RuntimeError, f"capture_request failed: {e}")
            return

        try:
            # Convert buffer to numpy safely
            frame = req.make_array("main")
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            # Camera is mounted upside-down due to space constraints
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            self["frame"] = frame

        finally:
            # This is CRITICAL: releases the buffer back to camera
            req.release()

    def _on_shutdown_impl(self) -> None:
        self.picam2.stop()
        self.picam2.close()