import math
import numpy as np

import math
import matplotlib.pyplot as plt

# === Paste your TankDriveModel class above this line ===
from tank_drive_model import TankDriveModel

# === Include the trajectory + RAMSETE functions from before ===
# (shortened here for brevity; make sure you include the full definitions)
# make_trapezoidal_profile(...)
# make_line_to_point_trajectory(...)
# simulate_to_ball(...)

# ---------- Trapezoidal profile generator ----------
def make_trapezoidal_profile(distance, vmax, amax):
    """
    Returns a function profile(t) -> (s, s_dot, s_ddot) for 0 <= t <= T_total.
    distance: total distance to travel (m)
    vmax: desired max linear speed (m/s)
    amax: max linear acceleration (m/s^2)
    """
    # handle degenerate
    if distance <= 0:
        return lambda t: (0.0, 0.0, 0.0), 0.0

    # time to accelerate to vmax
    t_acc = vmax / amax
    s_acc = 0.5 * amax * t_acc * t_acc

    if 2 * s_acc <= distance:
        # trapezoidal: accel, cruise, decel
        s_cruise = distance - 2 * s_acc
        t_cruise = s_cruise / vmax
        T = 2 * t_acc + t_cruise

        def profile(t):
            if t <= 0:
                return 0.0, 0.0, 0.0
            if t < t_acc:
                s = 0.5 * amax * t * t
                sd = amax * t
                sdd = amax
            elif t < t_acc + t_cruise:
                s = s_acc + vmax * (t - t_acc)
                sd = vmax
                sdd = 0.0
            elif t < T:
                td = t - (t_acc + t_cruise)
                s = s_acc + s_cruise + vmax * td - 0.5 * amax * td * td
                sd = vmax - amax * td
                sdd = -amax
            else:
                s = distance
                sd = 0.0
                sdd = 0.0
            return s, sd, sdd

        return profile, T
    else:
        # triangular profile (can't reach vmax)
        # peak velocity v_peak satisfying 2*(1/2 * v_peak^2 / a) = distance => v_peak = sqrt(a * distance)
        v_peak = math.sqrt(amax * distance)
        t_acc = v_peak / amax
        T = 2 * t_acc

        def profile(t):
            if t <= 0:
                return 0.0, 0.0, 0.0
            if t < t_acc:
                s = 0.5 * amax * t * t
                sd = amax * t
                sdd = amax
            elif t < T:
                td = t - t_acc
                s = 0.5 * amax * t_acc * t_acc + v_peak * td - 0.5 * amax * td * td
                sd = v_peak - amax * td
                sdd = -amax
            else:
                s = distance
                sd = 0.0
                sdd = 0.0
            return s, sd, sdd

        return profile, T

# ---------- Trajectory factory ----------
def make_line_to_point_trajectory(start_pose, goal_point, model,
                                  vmax=None, amax=None, stop_heading_at_goal=True):
    """
    start_pose: (x0,y0,theta0, v0, omega0) - only x0,y0 used for path
    goal_point: (xg, yg)
    model: your TankDriveModel instance (used to derive defaults)
    vmax: maximum linear speed to use (defaults to model.v_max)
    amax: maximum linear acceleration (defaults to physically possible from stall torque)
    stop_heading_at_goal: if True final heading = direction toward ball (path heading)
    Returns: traj(t) -> (x_d, y_d, theta_d, v_d, omega_d), total_time
    """
    x0, y0, _, _, _ = start_pose
    xg, yg = goal_point
    dx = xg - x0
    dy = yg - y0
    D = math.hypot(dx, dy)
    if D < 1e-9:
        # trivial
        def traj_zero(t):
            return xg, yg, 0.0, 0.0, 0.0
        return traj_zero, 0.0

    # defaults
    vmax = model.v_max if vmax is None else float(vmax)

    # estimate amax from available stall torque at wheel:
    # total forward force if both wheels give stall torque: F_total = 2 * tau_wheel / r
    # tau_wheel we estimated in model as model.tau_stall_wheel
    # use a safety factor (e.g., 0.6) because stall is not sustainable and motor curve drops.
    safety_factor = 0.6
    if amax is None:
        a_est = (2.0 * model.tau_stall_wheel * safety_factor) / (model.r * model.m)
        # clamp to reasonable value
        amax = max(0.05, float(min(a_est, 5.0)))  # at least 0.05 m/s^2, but not absurdly large
        print(f"a_est: {a_est}")
    else:
        amax = float(amax)

    print(f"amax: {amax}")
    print(f"vmax: {vmax}")

    # build 1D profile along path
    profile, T_total = make_trapezoidal_profile(D, vmax, amax)

    # path heading (we want to face along the path)
    path_theta = math.atan2(dy, dx)

    def traj(t):
        # clamp t
        if t <= 0:
            return x0, y0, path_theta, 0.0, 0.0
        s, sd, sdd = profile(t)
        # position interpolation along straight line
        ratio = s / D if D != 0 else 0.0
        x_d = x0 + ratio * dx
        y_d = y0 + ratio * dy

        # heading: keep facing along path (constant)
        theta_d = path_theta

        # linear velocity is sd
        v_d = sd

        # angular velocity: derivative of heading over time; heading is constant here -> zero
        omega_d = 0.0

        # If you prefer, we can smoothly ramp heading from initial to path heading over start window.
        # For simplicity we keep it constant; RAMSETE will handle small heading errors.
        return x_d, y_d, theta_d, v_d, omega_d

    return traj, T_total

# ---------- Example usage with RAMSETE loop ----------
def simulate_to_ball(model, start_state, ball_xy, dt=0.01):
    """
    start_state: (x,y,theta,v,omega)
    ball_xy: (xg, yg)
    returns time-series (t_list, x_list, y_list, x_d_list, y_d_list)
    """
    traj, T = make_line_to_point_trajectory(start_state, ball_xy, model)
    # choose final time horizon a bit longer to allow settling with controller
    T_sim = T + 2.0


    print(f"T: {T}")

    # ramsete controller (simple wrapper) - use common gains
    def ramsete_control(current, desired, b=2.0, zeta=0.7):
        x, y, theta, v, omega = current
        x_d, y_d, theta_d, v_d, omega_d = desired

        # pose error in robot frame
        dx = x_d - x
        dy = y_d - y
        ex = math.cos(theta) * dx + math.sin(theta) * dy
        ey = -math.sin(theta) * dx + math.cos(theta) * dy
        eth = theta_d - theta
        eth = math.atan2(math.sin(eth), math.cos(eth))

        k = 2 * zeta * math.sqrt(omega_d**2 + b * v_d**2)
        v_cmd = v_d * math.cos(eth) + k * ex
        # handle small eth safely for sin(eth)/eth
        if abs(eth) > 1e-6:
            s_over_th = math.sin(eth) / eth
        else:
            s_over_th = 1.0
        omega_cmd = omega_d + b * v_d * s_over_th * ey + k * eth
        return v_cmd, omega_cmd

    t_list = []
    x_list = []
    y_list = []
    x_d_list = []
    y_d_list = []
    v_list = []
    for i in range(int(T_sim / dt)):
        t = i * dt
        desired = traj(t)
        current = model.state()
        v_cmd, omega_cmd = ramsete_control(current, desired)
        # print(v_cmd, omega_cmd)

        # convert chassis (v,omega) to wheel linear speeds
        vL = v_cmd - (omega_cmd * model.L / 2.0)
        vR = v_cmd + (omega_cmd * model.L / 2.0)

        # convert to normalized motor commands using model.v_max
        uL = max(-1.0, min(1.0, vL / model.v_max))
        uR = max(-1.0, min(1.0, vR / model.v_max))

        model.step(uL, uR, dt)

        # log
        t_list.append(t)
        x, y, theta, v, omega = model.state()
        x_list.append(x); y_list.append(y)
        x_d_list.append(desired[0]); y_d_list.append(desired[1])
        v_list.append(v)

    return t_list, x_list, y_list, x_d_list, y_d_list, v_list



# --- your provided model setup ---
model = TankDriveModel(
    mass=5.0,              # kg
    wheelbase=0.332,       # m
    wheel_radius=0.035,    # m
    gear_ratio=0.5,        # motor speed : wheel speed
    stall_torque=34.0,     # kgf·cm
    v_max=5,             # m/s
    friction_static=0.2,   # fraction of stall torque
    friction_viscous=0.05, # Nm per (rad/s)
    friction_rolling=0.01, # fraction of weight
    gearbox_efficiency=0.95,
    x0=0.0, y0=0.0, h0=0.0,
    v0=0.0, omega0=0.0
)


# --- simulation setup ---
start_state = (0.0, 0.0, 0.0, 0.0, 0.0)     # x,y,θ,v,ω
ball_xy = (1.2, 0.2)                        # target location (m)

t_list, x_list, y_list, x_d_list, y_d_list, v_list = simulate_to_ball(
    model, start_state, ball_xy, dt=0.01
)

# --- visualize results ---
plt.figure(figsize=(7,6))
plt.plot(x_d_list, y_d_list, 'k--', label="Desired path")
plt.plot(x_list, y_list, 'b-', label="Actual path")
plt.scatter(ball_xy[0], ball_xy[1], c='orange', marker='o', s=80, label="Ball")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.axis("equal")
plt.legend()
plt.title("Robot trajectory to ball")

plt.figure(figsize=(7,4))
plt.plot(t_list, v_list, label="Actual linear velocity")
plt.xlabel("Time (s)")
plt.ylabel("v (m/s)")
plt.title("Velocity over time")
plt.grid(True)
plt.legend()
plt.show()










# ======= Keyboard-controllable sim ========
# import pygame
# import math
# from tank_drive_model import TankDriveModel  # assuming your model class is saved as tank_drive_model.py

# # === Initialize model ===
# model = TankDriveModel(
#     mass=5.0,
#     wheelbase=0.332,
#     wheel_radius=0.035,
#     gear_ratio=0.5,
#     stall_torque=34.0,
#     v_max=0.8,
#     friction_static=0.2,
#     friction_viscous=0.05,
#     friction_rolling=0.01,
#     gearbox_efficiency=0.95,
#     x0=0.0, y0=0.0, h0=0.0,
#     v0=0.0, omega0=0.0
# )

# # === Pygame setup ===
# pygame.init()
# WIDTH, HEIGHT = 800, 600
# screen = pygame.display.set_mode((WIDTH, HEIGHT))
# pygame.display.set_caption("TankDriveModel Test")
# clock = pygame.time.Clock()

# # Scale factor (m to pixels)
# SCALE = 200  # 1 m = 200 px
# CENTER_X, CENTER_Y = WIDTH // 2, HEIGHT // 2

# # Control states
# left_cmd = 0.0
# right_cmd = 0.0

# # Colors
# WHITE = (255, 255, 255)
# RED = (255, 60, 60)
# BLACK = (0, 0, 0)

# running = True
# dt = 1 / 60  # simulation timestep (60Hz)

# # === Main loop ===
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#         # Key press
#         if event.type == pygame.KEYDOWN:
#             if event.key == pygame.K_ESCAPE:
#                 running = False
#             elif event.key == pygame.K_q:  # left forward
#                 left_cmd = 1.0
#             elif event.key == pygame.K_z:  # left reverse
#                 left_cmd = -1.0
#             elif event.key == pygame.K_e:  # right forward
#                 right_cmd = 1.0
#             elif event.key == pygame.K_c:  # right reverse
#                 right_cmd = -1.0

#         # Key release
#         if event.type == pygame.KEYUP:
#             if event.key in [pygame.K_q, pygame.K_z]:
#                 left_cmd = 0.0
#             elif event.key in [pygame.K_e, pygame.K_c]:
#                 right_cmd = 0.0

#     # === Step simulation ===
#     model.step(left_cmd, right_cmd, dt)
#     x, y, theta, v, omega = model.state()

#     # === Draw robot ===
#     screen.fill(WHITE)

#     # Convert model coordinates to screen coords
#     px = CENTER_X + x * SCALE
#     py = CENTER_Y - y * SCALE

#     # Robot dimensions
#     half_L = model.wheelbase * SCALE / 2
#     body_length = model.wheelbase * SCALE  # draw as square for simplicity

#     # Compute corners
#     corners = []
#     for dx, dy in [(-half_L, -half_L), (half_L, -half_L),
#                    (half_L, half_L), (-half_L, half_L)]:
#         # Rotate and translate
#         rx = dx * math.cos(theta) + dy * math.sin(theta)
#         ry = -dx * math.sin(theta) + dy * math.cos(theta)
#         corners.append((px + rx, py + ry))

#     pygame.draw.polygon(screen, RED, corners, 2)

#     # Draw heading arrow
#     arrow_len = body_length / 1.5
#     arrow_x = px + arrow_len * math.cos(theta)
#     arrow_y = py - arrow_len * math.sin(theta)
#     pygame.draw.line(screen, BLACK, (px, py), (arrow_x, arrow_y), 3)

#     # HUD text
#     font = pygame.font.SysFont("consolas", 20)
#     text = font.render(f"x={x:.2f}  y={y:.2f}  v={v:.2f}  ω={omega:.2f}", True, BLACK)
#     screen.blit(text, (10, 10))

#     pygame.display.flip()
#     clock.tick(60)

# pygame.quit()
















# import numpy as np
# import matplotlib.pyplot as plt
# from math import cos, sin, atan2, sqrt

# # --- Assume TankDriveModel is already defined as per your config ---
# from tank_drive_model import TankDriveModel  # your sim core

# def ramsete_control(current, target, b=2.0, zeta=0.7):
#     x, y, theta, v, omega = current
#     x_d, y_d, theta_d, v_d, omega_d = target

#     # Compute pose error in robot frame
#     dx = x_d - x
#     dy = y_d - y
#     ex = cos(theta) * dx + sin(theta) * dy
#     ey = -sin(theta) * dx + cos(theta) * dy
#     eth = theta_d - theta
#     eth = np.arctan2(np.sin(eth), np.cos(eth))  # wrap to [-pi, pi]

#     k = 2 * zeta * sqrt(omega_d**2 + b * v_d**2)

#     v_cmd = v_d * cos(eth) + k * ex
#     omega_cmd = omega_d + b * v_d * (sin(eth) / eth if abs(eth) > 1e-5 else 1.0) * ey + k * eth

#     return v_cmd, omega_cmd

# # --- Create model ---
# model = TankDriveModel(
#     mass=5.0, wheelbase=0.332, wheel_radius=0.035,
#     gear_ratio=0.5, stall_torque=34.0, v_max=0.8,
#     friction_static=0.2, friction_viscous=0.05,
#     friction_rolling=0.01, gearbox_efficiency=0.95,
#     x0=0.0, y0=0.0, h0=0.0, v0=0.0, omega0=0.0
# )

# # --- Simulation setup ---
# dt = 0.01
# target_pose = (2.0, 1.0, np.pi/6)  # (x, y, heading)
# target_velocity = (0.0, 0.0)       # we want to end at rest
# b, zeta = 2.0, 0.7

# xs, ys = [], []
# for t in np.arange(0, 10, dt):
#     x, y, theta, v, omega = model.state()
#     xs.append(x)
#     ys.append(y)

#     # Define target pose and velocity (constant goal)
#     x_d, y_d, theta_d = target_pose
#     v_d, omega_d = target_velocity

#     v_cmd, omega_cmd = ramsete_control(
#         (x, y, theta, v, omega),
#         (x_d, y_d, theta_d, v_d, omega_d),
#         b, zeta
#     )

#     # Convert to wheel speeds
#     vL = v_cmd - omega_cmd * model.wheelbase / 2
#     vR = v_cmd + omega_cmd * model.wheelbase / 2

#     # Map to normalized motor commands
#     uL = np.clip(vL / model.v_max, -1, 1)
#     uR = np.clip(vR / model.v_max, -1, 1)

#     model.step(uL, uR, dt)

# # --- Visualization ---
# plt.figure(figsize=(6, 6))
# plt.plot(xs, ys, label="Path")
# plt.plot([target_pose[0]], [target_pose[1]], "ro", label="Goal")
# plt.axis("equal")
# plt.title("RAMSETE Controller Tracking to Target")
# plt.xlabel("X [m]")
# plt.ylabel("Y [m]")
# plt.legend()
# plt.show()















# import sys
# import os
# from tank_drive_model import TankDriveModel  # your sim core

# # Get the absolute path to the directory containing the module
# # For example, if your module is in a 'modules' directory one level up
# module_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', ''))

# # Add the directory to sys.path
# sys.path.append(module_dir)

# from utils.vector2 import Vector2
# from utils.pose2 import Pose2
# from threads.controller.algorithm.ramsete_controller import RAMSETEController
# import matplotlib.pyplot as plt
# import math


# # ---- Controller setup ----
# controller = RAMSETEController(
#     b=2.0, zeta=0.7, v_max=0.8, a_max=1.2,
#     replanning_max_delay=0, replanning_drift_threshold=0.05,
#     lookahead_time=0.3, wheel_diameter=0.07, wheel_base=0.25
# )

# # ---- Simulation setup ----
# target = Vector2(1.0, 0.5)
# pos = Vector2(0.0, 0.0)
# vel = Vector2(0.0, 0.0)
# theta = 0.0
# dt = 0.02
# el_time = 0
# trajectory = [(pos.x, pos.y)]

# # --- Create model ---
# model = TankDriveModel(
#     mass=5.0, wheelbase=0.332, wheel_radius=0.035,
#     gear_ratio=0.5, stall_torque=34.0, v_max=0.8,
#     friction_static=0.2, friction_viscous=0.05,
#     friction_rolling=0.01, gearbox_efficiency=0.95,
#     x0=pos.x, y0=pos.y, h0=theta, v0=vel.norm(), omega0=0.0
# )

# for step in range(100):
#     el_time += dt

#     # Compute motor speeds
#     left_power, right_power = controller.update_controller(
#         ball_pos=target,
#         robot_pose=Pose2(
#             Vector2(trajectory[-1][0],trajectory[-1][1]),
#             theta
#         ),
#         current_vel=vel,
#         elapsed_time=el_time
#     )
#     print(left_power, right_power)

#     model.step(left_power, right_power, dt)

#     xt, yt, thetat, vt, omegat = model.state()

#     trajectory.append((xt, yt))

#     # # Stop if near target
#     # if (target - pos).norm() < 0.02:
#     #     break

# # ---- Plot ----
# x, y = zip(*trajectory)
# plt.figure(figsize=(5, 5))
# plt.plot(x, y, 'b-', label="Robot path")
# plt.plot(target.x, target.y, 'ro', label="Target (1, 0.5)")
# plt.quiver(0, 0, 1, 0, angles='xy', scale_units='xy', scale=0.3, color='gray', label='Start dir')
# plt.axis('equal')
# plt.xlabel("x (m)")
# plt.ylabel("y (m)")
# plt.title("RAMSETE trajectory toward (1.0, 0.5)")
# plt.legend()
# plt.grid(True)
# plt.show()











