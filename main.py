from threads.vision import CameraThread
from threads.arduino_serial import ArduinoSerialThread
from utils import PiThread
import threading
import time
import cv2
import numpy as np

def main(with_watchdog: bool = True):
    cv2.namedWindow("Ping Pong Detection", cv2.WINDOW_NORMAL)


    camera_thread = CameraThread(frequency=10)
    arduino_serial_thread = ArduinoSerialThread(frequency=100)

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
                PiThread.kill_all()
                break

            # Show CameraThread's annotated frame
            annotated = CameraThread["detection.frame"]
            if annotated is not None:
                annotated = np.array(annotated)
                cv2.imshow("Ping Pong Detection", annotated)
            
            print(CameraThread["detection.points"])

            # time.sleep(0.1)
            cv2.waitKey(20)
            
    except KeyboardInterrupt:
        PiThread.kill_all()
        cv2.destroyAllWindows()
        print("[MAIN] Shutting down robot...")

if __name__ == "__main__":
    main(with_watchdog=True)