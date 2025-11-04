import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def euler_to_rotation_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,             cp*cr]
    ])
    return R

def draw_block(ax, R, size=1.0):
    s = size / 2
    vertices = np.array([
        [-s, -s, -s],
        [ s, -s, -s],
        [ s,  s, -s],
        [-s,  s, -s],
        [-s, -s,  s],
        [ s, -s,  s],
        [ s,  s,  s],
        [-s,  s,  s]
    ])
    rotated = vertices @ R.T
    faces = [
        [rotated[j] for j in [0,1,2,3]],
        [rotated[j] for j in [4,5,6,7]],
        [rotated[j] for j in [0,1,5,4]],
        [rotated[j] for j in [2,3,7,6]],
        [rotated[j] for j in [1,2,6,5]],
        [rotated[j] for j in [0,3,7,4]]
    ]
    colors = ['red', 'green', 'blue', 'yellow', 'orange', 'cyan']
    for i, f in enumerate(faces):
        ax.add_collection3d(Poly3DCollection([f], color=colors[i], alpha=0.6))

def update_orientation(ax, roll, pitch, yaw):
    """Clear and redraw the block with updated orientation."""
    ax.cla()  # clear current axis
    R = euler_to_rotation_matrix(roll, pitch, yaw)
    draw_block(ax, R)
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f"Roll={np.degrees(roll):.1f}°, Pitch={np.degrees(pitch):.1f}°, Yaw={np.degrees(yaw):.1f}°")
