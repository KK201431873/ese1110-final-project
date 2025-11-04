from threads.vision.camera_thread import CameraThread
from threads.arduino_serial.arduino_serial_thread import ArduinoSerialThread
from utils.pi_thread import PiThread
import numpy as np
import threading
import cv2
import time

def main(with_watchdog: bool = True, show_camera: bool = False):
    if show_camera:
        from cv2 import namedWindow, imshow, waitKey, destroyAllWindows, WINDOW_NORMAL
        cv2.namedWindow("Ping Pong Detection", cv2.WINDOW_NORMAL)


    camera_thread = CameraThread(frequency=10)
    # arduino_serial_thread = ArduinoSerialThread(frequency=100)

    print("[MAIN] Starting up robot...")
    camera_thread.start()
    # arduino_serial_thread.start()

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
    main(with_watchdog=True, show_camera=True)