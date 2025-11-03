from threads.vision import CameraThread
from threads.arduino_serial import ArduinoSerialThread
from utils import PiThread
import threading
import time

def main(with_watchdog: bool = True):
    # camera_thread = CameraThread(frequency=5)
    arduino_serial_thread = ArduinoSerialThread(frequency=100)

    print("[MAIN] Starting up robot...")
    # camera_thread.start()
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
            time.sleep(0.1)
    except KeyboardInterrupt:
        PiThread.kill_all()
        print("[MAIN] Shutting down robot...")

if __name__ == "__main__":
    main(with_watchdog=True)