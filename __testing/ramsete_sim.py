import sys
import os

# Get the absolute path to the directory containing the module
# For example, if your module is in a 'modules' directory one level up
module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', ''))

# Add the directory to sys.path
sys.path.append(module_dir)

from utils.vector2 import Vector2
from threads.controller.algorithm.ramsete_controller import RAMSETEController
import matplotlib.pyplot as plt
import math


# ---- Controller setup ----
controller = RAMSETEController(
    b=2.0, zeta=0.7, v_max=0.8, a_max=1.2,
    replanning_max_delay=0, replanning_drift_threshold=0.05,
    lookahead_time=0.3, wheel_diameter=0.07, wheel_base=0.25
)

# ---- Simulation setup ----
target = Vector2(1.0, 0.5)
pos = Vector2(0.0, 0.0)
vel = Vector2(0.0, 0.0)
theta = 0.0
dt = 0.02
trajectory = [(pos.x, pos.y)]

for step in range(10000):
    # Compute motor speeds
    left_rps, right_rps = controller.update_relative(
        ball_pos=Vector2(
            (target.x - pos.x)*math.cos(-theta) - (target.y - pos.y)*math.sin(-theta),
            (target.x - pos.x)*math.sin(-theta) + (target.y - pos.y)*math.cos(-theta)
        ),
        current_vel=vel
    )

    # Convert back to linear and angular velocity
    v_l = left_rps * controller.wheel_circumference
    v_r = right_rps * controller.wheel_circumference
    v = (v_r + v_l) / 2.0
    omega = (v_r - v_l) / controller.wheel_base

    # Integrate forward
    pos = Vector2(
        pos.x + v * math.cos(theta) * dt,
        pos.y + v * math.sin(theta) * dt
    )
    theta += omega * dt

    trajectory.append((pos.x, pos.y))

    # Stop if near target
    if (target - pos).norm() < 0.02:
        break

# ---- Plot ----
x, y = zip(*trajectory)
plt.figure(figsize=(5, 5))
plt.plot(x, y, 'b-', label="Robot path")
plt.plot(target.x, target.y, 'ro', label="Target (1, 0.5)")
plt.quiver(0, 0, 1, 0, angles='xy', scale_units='xy', scale=0.3, color='gray', label='Start dir')
plt.axis('equal')
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.title("RAMSETE trajectory toward (1.0, 0.5)")
plt.legend()
plt.grid(True)
plt.show()
