import time
start_time = time.perf_counter()

from threads.controller.controller_thread import ControllerThread
from threads.iot.iot_camera_feed_thread import IoTCameraFeedThread
from threads.iot.iot_minimap_thread import IoTMinimapThread
from threads.peripherals.sensor_thread import SensorThread
from threads.vision.camera_thread import CameraThread
from utils.load_settings import load_settings
from utils.pi_thread import PiThread
import numpy as np
import threading
import cv2
import time

def main(with_watchdog: bool = True, show_camera: bool = False):
    if show_camera:
        cv2.namedWindow("Ping Pong Detection", cv2.WINDOW_NORMAL)

    # --- Init threads ---
    settings = load_settings()
    controller_thread_frequency = settings["controller_thread"]["frequency"]
    iot_camera_feed_thread_frequency = settings["iot_camera_feed_thread"]["frequency"]
    iot_minimap_thread_frequency = settings["iot_minimap_thread"]["frequency"]
    sensor_thread_frequency = settings["sensor_thread"]["frequency"]
    camera_thread_frequency = settings["camera_thread"]["frequency"]

    controller_thread = ControllerThread(frequency=controller_thread_frequency)
    iot_camera_feed_thread = IoTCameraFeedThread(frequency=iot_camera_feed_thread_frequency)
    iot_minimap_thread = IoTMinimapThread(frequency=iot_minimap_thread_frequency)
    sensor_thread = SensorThread(frequency=sensor_thread_frequency)
    camera_thread = CameraThread(frequency=camera_thread_frequency)

    # --- Start threads ---
    print("[MAIN] Starting up robot...")
    controller_thread.start()
    iot_camera_feed_thread.start()
    iot_minimap_thread.start()
    sensor_thread.start()
    camera_thread.start()

    elapsed_time = time.perf_counter() - start_time
    print(f"STARTUP TIME: {elapsed_time}")

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
                    time.sleep(0.1)
                break

            # print(f"[MAIN] Camera Thread Freq: {camera_thread.get_measured_frequency():.2f} Hz")
            # print(f"[MAIN] Sensor Thread Freq: {sensor_thread.get_measured_frequency():.2f} Hz")

            if show_camera:
                # Show CameraThread's annotated frame
                annotated = CameraThread["detection.frame"]
                if annotated is not None:
                    annotated = np.array(annotated)
                    cv2.imshow("Ping Pong Detection", annotated)
                # print(CameraThread["detection.points"])
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
    main(with_watchdog=with_watchdog, show_camera=show_camera)