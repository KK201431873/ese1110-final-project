# Entry point: starts and coordinates threads
import threading
from threads.vision import CameraThread
from threads.arduino_serial import ArduinoSerialThread

def main():
    global_data = {}
    lock = threading.Lock()

    camera_thread = CameraThread(global_data, lock, frequency=5)
    arduino_serial_thread = ArduinoSerialThread(global_data, lock, frequency=100)

    # TODO: make centralized, global data storage? (like CAN variables in per namespace ig)

    print("[MAIN] Starting up robot...")
    camera_thread.start()
    arduino_serial_thread.start()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        camera_thread.stop()
        print("[MAIN] Shutting down robot...")

if __name__ == "__main__":
    main()