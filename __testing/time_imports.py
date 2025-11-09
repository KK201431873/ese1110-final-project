import socket, time
t=time.perf_counter()
socket.create_connection(("ws://ese.pongworks.dpdns.org/pi_stream", 5000), timeout=1)
print("TCP connect:", time.perf_counter()-t)

# # time_imports.py
# import time
# import importlib
# import sys

# import os

# # Get the absolute path to the directory containing the module
# # For example, if your module is in a 'modules' directory one level up
# module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', ''))

# # Add the directory to sys.path
# sys.path.append(module_dir)

# MODULES = [
#     # hardware / camera
#     "libcamera", "libcamera._libcamera", "picamera2", "picamera2.picamera2",
#     "picamera2.encoders", "picamera2.platform", "videodev2",

#     # media / codec
#     "av", "av.container", "av.audio", "av.video",

#     # vision / imaging
#     "cv2", "PIL", "PIL.Image", "simplejpeg",

#     # numeric & ML
#     "numpy", "numpy.core", "onnxruntime", "onnxruntime.capi.onnxruntime_pybind11_state",

#     # network / websocket
#     "websocket", "websocket._app", "websocket._handshake",

#     # your utils (relative names used in project)
#     "utils.load_settings", "utils.websocket_interface", "threads.vision.camera_thread",
# ]

# def timed_import(name):
#     if name in sys.modules:
#         # already imported; skip to get first-time cost (optional)
#         print(f"{name:40} already imported (skipping)")
#         return
#     t0 = time.perf_counter()
#     try:
#         importlib.import_module(name)
#     except Exception as e:
#         delta = time.perf_counter() - t0
#         print(f"{name:40} -> import ERROR after {delta:.3f}s: {e!r}")
#         return
#     delta = time.perf_counter() - t0
#     print(f"{name:40} -> {delta:.3f}s")

# if __name__ == "__main__":
#     print("Python:", sys.version)
#     print("Running timed imports; this imports modules in separate import order.")
#     for m in MODULES:
#         timed_import(m)
