import time
start_time = time.perf_counter()

from threads.iot.iot_camera_feed_thread import IoTCameraFeedThread
from threads.iot.iot_minimap_thread import IoTMinimapThread
from threads.iot.iot_receive_thread import IoTReceiveThread
from threads.peripherals.sensor_thread import SensorThread
from threads.vision.camera_thread import CameraThread
from threads.vision.inference_thread import InferenceThread

from utils.load_settings import load_settings
from utils.pi_thread import PiThread

import numpy as np
import threading
import cv2


def normalize_angle(degrees: float) -> float:
    """Normalize an angle to the range [-180, 180)."""
    while degrees >= 180:
        degrees -= 2 * 180
    while degrees < -180:
        degrees += 2 * 180
    return degrees
import threads.controller.controller_serial_interface as controller
class ControllerThread(PiThread):
    running: bool = False
    """Whether or not the robot is running, controlled via IoTReceiveThread."""

    def _on_created_impl(self) -> None:
        settings = load_settings()
        controller_settings = settings["controller_thread"]
        self.USE_IOT_COMMANDS = controller_settings["USE_IOT_COMMANDS"]
        self.INTAKE_UP_POS = controller_settings["INTAKE_UP_POS"]
        pass

    def _on_start_impl(self) -> None:
        controller.stop_drive()
        self.print("Alive!")

    def _loop_impl(self) -> None:
        # === IoT Control ===
        if self.USE_IOT_COMMANDS:
            running = IoTReceiveThread["running"]
            if not running:
                controller.stop_drive()
                controller.set_intake_position(self.INTAKE_UP_POS)
                controller.set_intake_power(0.0)
                return
        
        yaw = SensorThread["imu.yaw"]
        if yaw is not None:
            target = 180.0 # degrees
            error = normalize_angle(target - yaw)
            if error > 0:
                controller.drive_speed_angle(0, 0.5)
            else:
                controller.stop_drive()
        

    def _on_shutdown_impl(self) -> None:
        controller.stop_drive()




def main(with_watchdog: bool = True, show_camera: bool = False, show_detections: bool = False):
    if show_camera:
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    if show_detections:
        cv2.namedWindow("Ping Pong Detections", cv2.WINDOW_NORMAL)

    # --- Init threads ---
    settings = load_settings()
    controller_thread_frequency = settings["controller_thread"]["frequency"]
    iot_camera_feed_thread_frequency = settings["iot_camera_feed_thread"]["frequency"]
    iot_minimap_thread_frequency = settings["iot_minimap_thread"]["frequency"]
    iot_receive_thread_frequency = settings["iot_receive_thread"]["frequency"]
    sensor_thread_frequency = settings["sensor_thread"]["frequency"]
    camera_thread_frequency = settings["camera_thread"]["frequency"]
    inference_thread_frequency = settings["inference_thread"]["frequency"]

    controller_thread = ControllerThread(frequency=controller_thread_frequency)
    iot_camera_feed_thread = IoTCameraFeedThread(frequency=iot_camera_feed_thread_frequency)
    iot_minimap_thread = IoTMinimapThread(frequency=iot_minimap_thread_frequency)
    iot_receive_thread = IoTReceiveThread(frequency=iot_receive_thread_frequency)
    sensor_thread = SensorThread(frequency=sensor_thread_frequency)
    camera_thread = CameraThread(frequency=camera_thread_frequency)
    inference_thread = InferenceThread(frequency=inference_thread_frequency)

    # --- Start threads ---
    print("[MAIN] Starting up robot...")
    controller_thread.start()
    iot_camera_feed_thread.start()
    iot_minimap_thread.start()
    iot_receive_thread.start()
    sensor_thread.start()
    camera_thread.start()
    inference_thread.start()

    elapsed_time = time.perf_counter() - start_time
    print(f"[MAIN] All threads started up in {elapsed_time:.3f} seconds.")

    # --- Main loop ---
    try:
        while True:
            crashed = False
            if not any(t.is_alive() for t in threading.enumerate() if isinstance(t, PiThread)):
                print("[MAIN] All PiThreads have stopped unexpectedly.")
                crashed = True
            if with_watchdog and PiThread.has_crashed():
                print(f"[MAIN] Detected thread crash: {PiThread.get_crash_message()}")
                crashed = True
            if crashed:
                while any((t.is_alive() or t.get_alive()) for t in threading.enumerate() if isinstance(t, PiThread)):
                    PiThread.kill_all()
                    time.sleep(0.25)
                break

            # print(f"[MAIN] Camera Thread Freq: {camera_thread.get_measured_frequency():.2f} Hz")
            # print(f"[MAIN] Sensor Thread Freq: {sensor_thread.get_measured_frequency():.2f} Hz")

            if show_camera or show_detections:
                if show_camera:
                    frame = CameraThread["frame"]
                    if frame is not None:
                        cv2.imshow("Camera Feed", frame)
                if show_detections:
                    annotated = InferenceThread["detection.frame"]
                    if annotated is not None:
                        cv2.imshow("Ping Pong Detections", annotated)
                cv2.waitKey(20)
            else:
                time.sleep(0.1)
            
    # --- Exit condition ---
    except KeyboardInterrupt:
        PiThread.kill_all()
        if show_camera:
            cv2.destroyAllWindows()
        print("[MAIN] Shutting down robot...")

if __name__ == "__main__":
    settings = load_settings()["main"]
    with_watchdog = settings["with_watchdog"]
    show_camera = settings["show_camera"]
    show_detections = settings["show_detections"]
    main(with_watchdog=with_watchdog, show_camera=show_camera, show_detections=show_detections)