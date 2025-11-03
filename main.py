# Entry point: starts and coordinates threads
import threading
from threads.vision import CameraThread
from threads.arduino_serial import ArduinoSerialThread

def main():
    # camera_thread = CameraThread(frequency=5)
    arduino_serial_thread = ArduinoSerialThread(frequency=100)

    print("[MAIN] Starting up robot...")
    # camera_thread.start()
    arduino_serial_thread.start()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        # camera_thread.kill()
        arduino_serial_thread.kill()
        print("[MAIN] Shutting down robot...")

if __name__ == "__main__":
    main()