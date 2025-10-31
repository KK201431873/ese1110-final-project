from picamera2 import Picamera2
import numpy as np
import cv2
import time

# Load AR marker dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# Camera intrinsics (tune later)
focal_length = 800
camera_matrix= np.array([
    [focal_length, 0, 640],
    [0, focal_length, 360],
    [0, 0, 1]
], dtype=float)
dist_coeffs = np.zeros((4,1)) #assume no distortion


camera = Picamera2()
camera.configure(camera.create_preview_configuration(
    main={"size": (1280, 720)}
))
camera.start()

# camera warms up
time.sleep(1)

camera.set_controls({
    "AeEnable": False,
    "ExposureTime": 5000 # microseconds
})


metadata = camera.capture_metadata() # camera settings
frame_duration = metadata.get("FrameDuration", 0)

if frame_duration > 0:
    print(f"{1_000_000 / frame_duration} FPS")
else:
    print("--- FPS")

marker_size = 0.04 # meters
while True:
    frame = cv2.cvtColor(camera.capture_array(), cv2.COLOR_RGB2BGR)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
        if rvecs is not None:
            for rvec, tvec in zip(rvecs, tvecs):
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

    cv2.imshow("Camera Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
camera.stop()