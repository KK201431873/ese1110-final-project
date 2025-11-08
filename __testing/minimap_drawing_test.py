import numpy as np
import cv2
import math


"""
Draw a minimap showing the robot heading and the nearest ping-pong ball.
"""
robot_heading = math.pi/3

# --- Map size ---
width, height = 300, 200  # you can adjust this later dynamically

# Create a blank dark background
minimap = np.zeros((height, width, 3), dtype=np.uint8)
minimap[:] = (20, 20, 20)

# --- Draw robot in center ---
center = (width // 2, height // 2)
cv2.circle(minimap, center, 10, (0, 255, 255), -1)

# Draw heading line
heading_length = 20
end_x = int(center[0] + heading_length * np.cos(robot_heading))
end_y = int(center[1] - heading_length * np.sin(robot_heading))
cv2.line(minimap, center, (end_x, end_y), (0, 255, 255), 2)

# --- Draw ball (if detected) ---
_target_relative_position = (10, 30)
if _target_relative_position:
    bx = int(center[0] + _target_relative_position[1])  # left-right offset
    by = int(center[1] - _target_relative_position[0])  # forward offset
    cv2.circle(minimap, (bx, by), 6, (0, 165, 255), -1)

while True:
    cv2.imshow("Minimap", minimap)
    cv2.waitKey(1)