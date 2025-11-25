import subprocess
import os
import time

port1 = "1-2"
port2 = "3-2"
rpistr = f"sudo sh -c 'echo \"{port1}\" > /sys/bus/usb/drivers/usb/unbind'"
rpistr = f"sudo sh -c 'echo \"{port2}\" > /sys/bus/usb/drivers/usb/unbind'"
p = subprocess.Popen(rpistr, shell=True, preexec_fn=os.setsid)
time.sleep(2)
rpistr = f"sudo sh -c 'echo \"{port1}\" > /sys/bus/usb/drivers/usb/bind'"
rpistr = f"sudo sh -c 'echo \"{port2}\" > /sys/bus/usb/drivers/usb/bind'"
p = subprocess.Popen(rpistr, shell=True, preexec_fn=os.setsid)


# import os
# import time
# from pathlib import Path

# def reset_pi5_usb3_ports():
#     """
#     Reset ONLY the Raspberry Pi 5 USB 3.0 ports (where the two MCUs are plugged in).
#     Requires sudo.
#     """
#     usb_root = Path("/sys/bus/usb/devices")
#     usb3_prefix = "2-"  # USB3 root hub on Pi 5

#     devices = [dev for dev in usb_root.iterdir() if dev.name.startswith(usb3_prefix)]

#     if not devices:
#         print("No USB3 devices found under usb2 root hub.")
#         return

#     print(f"Found USB3 devices: {[d.name for d in devices]}")

#     for dev in devices:
#         auth_file = dev / "authorized"
#         if auth_file.exists():
#             try:
#                 print(f"Resetting {dev.name}...")
#                 auth_file.write_text("0")
#                 time.sleep(0.25)
#                 auth_file.write_text("1")
#                 print(f"✓ Reset {dev.name}")
#             except PermissionError:
#                 print(f"ERROR: Need sudo to reset {dev.name}")
#         else:
#             print(f"No 'authorized' file for {dev.name}, skipping.")

# if __name__ == "__main__":
#     reset_pi5_usb3_ports()
