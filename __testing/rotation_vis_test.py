import sys
import os

# Get the absolute path to the directory containing the module
# For example, if your module is in a 'modules' directory one level up
module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', ''))

# Add the directory to sys.path
sys.path.append(module_dir)

from threads.arduino_serial.arduino_serial_thread import ArduinoSerialThread
from utils.pi_thread import PiThread
import threading
import time

from utils.rotation_visualizer import update_orientation
import matplotlib.pyplot as plt
import numpy as np

def main(with_watchdog: bool = True):
    # camera_thread = CameraThread(frequency=5)
    arduino_serial_thread = ArduinoSerialThread(frequency=100)

    print("[MAIN] Starting up robot...")
    # camera_thread.start()
    arduino_serial_thread.start()

    # === Initialize plot once ===
    plt.ion()  # interactive mode ON
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

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
            # time.sleep(0.1)


            roll = float(ArduinoSerialThread["imu.roll"] or 0)
            pitch = float(ArduinoSerialThread["imu.pitch"] or 0)
            yaw = float(ArduinoSerialThread["imu.yaw"] or 0)

            update_orientation(ax, np.radians(roll), np.radians(pitch), np.radians(yaw))
            plt.pause(0.05)  # update the GUI event loop

    except KeyboardInterrupt:
        PiThread.kill_all()
        print("[MAIN] Shutting down robot...")

if __name__ == "__main__":
    main(with_watchdog=True)