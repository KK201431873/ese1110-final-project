import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import math
import time

class TankDriveModel:
    def __init__(self,
                 mass: float,
                 wheelbase: float,
                 wheel_radius: float,
                 gear_ratio: float,
                 stall_torque: float,
                 v_max: float,
                 friction_static: float = 0.2,
                 friction_viscous: float = 0.05,
                 friction_rolling: float = 0.01,
                 gearbox_efficiency: float = 0.95,
                 x0: float = 0.0, y0: float = 0.0, h0: float = 0.0,
                 v0: float = 0.0, omega0: float = 0.0):
        """
        TankDriveModel - dynamic tank-drive core.

        Args:
            mass (kg)
            wheelbase (m) - distance between left and right wheel centers
            wheel_radius (m)
            gear_ratio (motor_rev_per_wheel_rev): motor revs / wheel revs (e.g. 10 => 10:1 reduction)
            stall_torque (kg·cm) - stall torque spec at the motor shaft (12 V). We'll convert to N·m.
            v_max (m/s) - maximum linear speed of the robot center (used to infer motor no-load speed)
            friction_static (fraction of motor stall torque) - fraction [0..1] of motor stall torque needed to break static friction
            friction_viscous (Nm per (rad/s)) - applied at motor shaft (viscous damping)
            friction_rolling (fraction of weight) - fraction of m*g used as a constant opposing translational force
            gearbox_efficiency (0..1) - approximate gearbox torque transmission efficiency
            initial states x0, y0, h0 (rad), v0 (m/s), omega0 (rad/s)
        """
        # Physical parameters
        self.m = float(mass)
        self.L = float(wheelbase)
        self.r = float(wheel_radius)
        self.G = float(gear_ratio)            # motor_rev / wheel_rev
        self.v_max = float(v_max)
        
        self.wheelbase = wheelbase

        # Convert advertised stall torque (kg·cm) -> motor shaft N·m
        self.tau_stall_motor = float(stall_torque) * 0.0980665

        # Gearbox / wheel-side stall torque (approx)
        self.gearbox_eff = float(gearbox_efficiency)
        # wheel stall torque = motor_stall * gear_ratio * eff
        self.tau_stall_wheel = self.tau_stall_motor * self.G * self.gearbox_eff

        # Motor no-load angular speed (rad/s) at 12 V:
        # wheel no-load omega = v_max / r  (rad/s)
        # motor no-load omega = wheel_omega * gear_ratio
        self.omega_noload_motor = (self.v_max / self.r) * self.G

        # rotational inertia about z for square plate (side = L): I = 1/6 m L^2
        self.Iz = (1.0 / 6.0) * self.m * (self.L ** 2)

        # Friction coefficients
        # static friction threshold expressed as fraction of motor shaft stall torque
        self.friction_static_frac = float(friction_static)
        # viscous friction (apply at motor shaft, Nm per rad/s)
        self.friction_viscous_motor = float(friction_viscous)
        # rolling friction: fraction of weight; convert to Newtons (constant opposing force)
        self.friction_rolling_N = float(friction_rolling) * self.m * 9.81

        # State
        self.x = float(x0)
        self.y = float(y0)
        self.theta = float(h0)
        self.v = float(v0)
        self.omega = float(omega0)

    def _motor_torque_and_wheel_torque(self, commanded_voltage_ratio: float, wheel_linear_speed: float) -> tuple[float, float, float]:
        """
        Compute motor torque (Nm at motor shaft) and resulting wheel torque (Nm at wheel shaft).
        commanded_voltage_ratio: [-1,1] fraction of supply voltage / command
        wheel_linear_speed: m/s (tangential speed at wheel surface)
        """
        # wheel angular speed (rad/s)
        omega_wheel = wheel_linear_speed / self.r
        # motor angular speed (rad/s)
        omega_motor = omega_wheel * self.G

        # Linear torque-speed curve (motor shaft). Handle sign properly.
        # tau_motor_mag = tau_stall_motor * (1 - |omega_motor| / omega_noload_motor)
        # tau_motor = sign(cmd) * tau_motor_mag  (scaled by commanded voltage ratio)
        if self.omega_noload_motor <= 0:
            # protect division by zero
            tau_motor_no_load_frac = 0.0
        else:
            tau_motor_no_load_frac = max(0.0, 1.0 - (abs(omega_motor) / self.omega_noload_motor))

        # Command scales available torque (and sign)
        tau_motor = self.tau_stall_motor * commanded_voltage_ratio * tau_motor_no_load_frac

        # Limit to stall torque bounds
        tau_motor = max(min(tau_motor, self.tau_stall_motor), -self.tau_stall_motor)

        # Apply viscous friction at motor shaft (opposes motor rotation)
        tau_motor -= self.friction_viscous_motor * omega_motor

        # Static friction at motor shaft: if motor is nearly stopped and torque is below threshold, set to zero
        static_threshold = self.friction_static_frac * self.tau_stall_motor
        if abs(omega_motor) < 1e-3 and abs(tau_motor) < static_threshold:
            # motor can't overcome static friction -> zero motor torque delivered
            tau_motor = 0.0

        # Convert motor torque to wheel torque via gearbox (multiply by gear ratio and efficiency)
        tau_wheel = tau_motor * self.G * self.gearbox_eff

        return tau_motor, tau_wheel, omega_motor

    def step(self, left_cmd: float, right_cmd: float, dt: float) -> None:
        """
        left_cmd, right_cmd: commanded voltage ratios [-1, 1] for left and right motors
        dt: timestep seconds
        """
        # compute left/right wheel linear speeds from body velocities
        v_L = self.v - (self.omega * self.L / 2.0)
        v_R = self.v + (self.omega * self.L / 2.0)

        # compute motor & wheel torques
        tau_motor_L, tau_wheel_L, _ = self._motor_torque_and_wheel_torque(left_cmd, v_L)
        tau_motor_R, tau_wheel_R, _ = self._motor_torque_and_wheel_torque(right_cmd, v_R)

        # wheel forces at ground (F = tau_wheel / r)
        F_L = tau_wheel_L / self.r
        F_R = tau_wheel_R / self.r

        # rolling resistance force opposing motion (acts on translational COM)
        if abs(self.v) > 1e-6:
            sign_v = math.copysign(1.0, self.v)
        else:
            sign_v = 0.0
        F_rolling = -self.friction_rolling_N * sign_v

        # net linear acceleration
        a = (F_L + F_R + F_rolling) / self.m

        # torque about center from wheel forces: tau_z = (F_R - F_L) * (L/2)
        tau_z = (F_R - F_L) * (self.L / 2.0)
        alpha = tau_z / self.Iz

        # integrate
        self.v += a * dt
        self.omega += alpha * dt
        self.theta += self.omega * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

    def state(self) -> tuple[float, float, float, float, float]:
        """
        Returns:
            x (m), y (m), heading (rad), linear velocity (m/s), angular velocity (rad/s)
        """
        return (self.x, self.y, self.theta, self.v, self.omega)







# if __name__ == "__main__":
#     model = TankDriveModel(
#         mass=5.0,              # kg
#         wheelbase=0.332,       # m
#         wheel_radius=0.035,    # m
#         gear_ratio=0.5,        # motor speed:wheel speed
#         stall_torque=34.0,     # kgfcm
#         v_max=0.8,             # m/s
#         friction_static=0.2,   # fraction of stall torque
#         friction_viscous=0.05, # Nm per (rad/s)
#         friction_rolling=0.01, # fraction of weight
#         gearbox_efficiency=0.95,
#         x0=0.0, y0=0.0, h0=0.0,
#         v0=0.0, omega0=0.0
#     )
#     # --- Simulation parameters ---
#     dt=0.01
#     total_time = 6.0
#     steps = int(total_time / dt)
#     xs, ys, thetas = [], [], []

#     start_time = time.perf_counter()
#     for t in range(steps):
#         time_s = t * dt
#         if time_s < 3.0:
#             model.step(1.0, 1.0, dt)
#         else:
#             model.step(0.0, 0.0, dt)

#         x, y, theta, v, omega = model.state()
#         xs.append(x)
#         ys.append(y)
#         thetas.append(theta)
#     print(f"Sim took {time.perf_counter()-start_time}s to run")

#     # --- Visualization ---
#     fig, ax = plt.subplots()
#     ax.set_aspect('equal')
#     ax.set_xlim(-2.5, 2.5)
#     ax.set_ylim(-2.5, 2.5)
#     path, = ax.plot([], [], 'b-', lw=2)
#     robot_body, = ax.plot([], [], 'r-', lw=2)

#     def init():
#         path.set_data([], [])
#         robot_body.set_data([], [])
#         return path, robot_body

#     def update(i):
#         path.set_data(xs[:i], ys[:i])
#         # robot body as square centered on (x, y)
#         L = model.L
#         x, y, theta = xs[i], ys[i], thetas[i]
#         half = L / 2
#         corners = np.array([
#             [-half, -half],
#             [ half, -half],
#             [ half,  half],
#             [-half,  half],
#             [-half, -half]
#         ])
#         R = np.array([[np.cos(theta), -np.sin(theta)],
#                       [np.sin(theta),  np.cos(theta)]])
#         rotated = corners @ R.T + np.array([x, y])
#         robot_body.set_data(rotated[:, 0], rotated[:, 1])
#         return path, robot_body

#     ani = animation.FuncAnimation(fig, update, frames=steps,
#                                   init_func=init, interval=10, blit=True)
#     plt.title("Tank Drive Simulation")
#     plt.show()