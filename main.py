from threads.vision.camera_thread import CameraThread
from threads.arduino_serial.arduino_serial_thread import ArduinoSerialThread
from utils.load_settings import load_settings
from utils.pi_thread import PiThread
import numpy as np
import threading
import cv2
import time

def main(with_watchdog: bool = True, show_camera: bool = False):
    if show_camera:
        from cv2 import namedWindow, imshow, waitKey, destroyAllWindows, WINDOW_NORMAL
        cv2.namedWindow("Ping Pong Detection", cv2.WINDOW_NORMAL)

    settings = load_settings()
    camera_thread_frequency = settings["camera"]["frequency"]
    arduino_serial_thread_frequency = settings["arduino_serial"]["frequency"]

    camera_thread = CameraThread(frequency=camera_thread_frequency)
    arduino_serial_thread = ArduinoSerialThread(frequency=arduino_serial_thread_frequency)

    print("[MAIN] Starting up robot...")
    camera_thread.start()
    arduino_serial_thread.start()

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
            # print(f"[MAIN] Arduino Serial Thread Freq: {arduino_serial_thread.get_measured_frequency():.2f} Hz")

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