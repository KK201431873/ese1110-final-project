from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.websocket_interface import WebSocketInterface
from utils.pose2 import Pose2
from utils.vector2 import Vector2
from threads.peripherals.sensor_thread import SensorThread
from threads.vision.camera_thread import CameraThread
import numpy as np
import math
import cv2

class IoTMinimapThread(PiThread):
    ROBOT_POSE: Pose2 | None
    """Global robot X and Y coordinates in meters and heading in radians."""

    BG_COLOR: tuple[int, int, int]
    """BGR background color."""

    MINOR_COLOR: tuple[int, int, int]
    """BGR color of fractional number grid lines."""

    MAJOR_COLOR: tuple[int, int, int]
    """BGR color of whole number grid lines."""

    FAR_DETECTION_COLOR: tuple[int, int, int]
    """BGR color of a ping-pong ball that isn't the closest one."""

    BEST_DETECTION_COLOR: tuple[int, int, int]
    """BGR color of the closest ping-pong ball."""

    w: int
    """Width of the minimap, in pixels."""

    h: int
    """Height of the minimap, in pixels."""

    width_meters: float
    """Physical width represented by the minimap, in meters."""

    meter_subdivisions: int
    """Number of equally-spaced grid lines drawn per meter"""

    wheel_base: float
    """Distance between the left and right wheels, in meters."""

    aspect_ratio: float
    """Width divided by height."""

    meters_per_pixel: float
    """Conversion factor from pixels to meters."""

    grid_width: float
    """Spacing between grid lines, in meters."""

    def _on_created_impl(self) -> None:
        self.ROBOT_POSE = None

        # --- Load settings ---
        settings = load_settings()
        minimap_settings = settings["iot_minimap_thread"]

        # Thread config
        self.BG_COLOR = tuple(minimap_settings["BG_COLOR"])
        self.MINOR_COLOR = tuple(minimap_settings["MINOR_COLOR"])
        self.MAJOR_COLOR = tuple(minimap_settings["MAJOR_COLOR"])
        self.FAR_DETECTION_COLOR = tuple(minimap_settings["FAR_DETECTION_COLOR"])
        self.BEST_DETECTION_COLOR = tuple(minimap_settings["BEST_DETECTION_COLOR"])
        self.w, self.h = tuple(minimap_settings["minimap_size"])
        self.width_meters = minimap_settings["width_meters"]
        self.meter_subdivisions = minimap_settings["meter_subdivisions"]
        # From drive config, but still used here
        self.wheel_base = settings["drive_config"]["wheel_base"]

        # Calculated parameters
        self.aspect_ratio = self.w / self.h
        self.meters_per_pixel = self.width_meters / self.w
        self.grid_width = 1.0 / self.meter_subdivisions

        # Connect to WebSocket
        WebSocketInterface._ensure_socket(self)

    def _on_start_impl(self) -> None:
        self.print("Alive!")

    def _loop_impl(self) -> None:
        self.ROBOT_POSE = SensorThread["localization.pose"]
        if self.ROBOT_POSE is None:
            return
        
        # === --- and send minimap ---
        minimap = self.generate_minimap(self.ROBOT_POSE)
        WebSocketInterface.send_minimap(self, minimap)
    
        
    def generate_minimap(self, robot_pose: Pose2) -> np.ndarray:
        minimap = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        minimap[:] = self.BG_COLOR # Create a blank dark background
        center = (self.w // 2, self.h // 2)

        # --- Draw grid lines ---
        # Pixels per meter
        px_per_m = 1.0 / self.meters_per_pixel

        # Center world coordinates (in meters)
        cx, cy = robot_pose.COORDS.x, robot_pose.COORDS.y

        # Compute world-space bounds visible in minimap
        half_width_m = (self.w / 2) * self.meters_per_pixel
        half_height_m = (self.h / 2) * self.meters_per_pixel

        xmin = cx - half_width_m
        xmax = cx + half_width_m
        ymin = cy - half_height_m
        ymax = cy + half_height_m

        # Round down to nearest grid step to start lines cleanly
        start_x = math.floor(xmin / self.grid_width) * self.grid_width
        start_y = math.floor(ymin / self.grid_width) * self.grid_width

        # Draw vertical and horizontal grid lines
        for gx in np.arange(start_x, xmax, self.grid_width):
            x_px = round((gx - cx) * px_per_m + self.w / 2)
            is_major = (round(gx / self.grid_width) % self.meter_subdivisions == 0)
            color = self.MAJOR_COLOR if is_major else self.MINOR_COLOR
            cv2.line(minimap, (x_px, 0), (x_px, self.h), color, 1, cv2.LINE_AA)

        for gy in np.arange(start_y, ymax, self.grid_width):
            y_px = round(self.h / 2 - (gy - cy) * px_per_m)
            is_major = (round(gy / self.grid_width) % self.meter_subdivisions == 0)
            color = self.MAJOR_COLOR if is_major else self.MINOR_COLOR
            cv2.line(minimap, (0, y_px), (self.w, y_px), color, 1, cv2.LINE_AA)

        # --- Draw robot in center ---
        bot_side_length = int(self.wheel_base * px_per_m)
        rect = (center, (bot_side_length, bot_side_length), -math.degrees(robot_pose.h)) 
        box = cv2.boxPoints(rect)
        box = np.intp(box)
        
        # semi-transparent fill + outline
        overlay = minimap.copy()
        cv2.fillPoly(overlay, [box], color=(255, 255, 255)) # type: ignore
        alpha = 0.4
        cv2.addWeighted(overlay, alpha, minimap, 1 - alpha, 0, minimap)
        cv2.polylines(minimap, [box], isClosed=True, color=(255, 255, 255), thickness=1, lineType=cv2.LINE_AA) # type: ignore
        
        # Draw heading line
        end_x = int(center[0] + bot_side_length/2 * np.cos(robot_pose.h))
        end_y = int(center[1] - bot_side_length/2 * np.sin(robot_pose.h))
        cv2.line(minimap, center, (end_x, end_y), (255, 255, 255), thickness=1, lineType=cv2.LINE_AA)

        # --- Draw detections ---
        detection_points: list[Vector2] | None = CameraThread["detection.absolute_points"]
        if detection_points is None or len(detection_points) == 0:
            return minimap

        closest_ball, remaining_balls = self.separate_closest_ball(detection_points, robot_pose.COORDS)

        def draw_ball(ball: Vector2, color: tuple[int, int, int]) -> None:
            bx = int(center[0] + px_per_m * (ball.x - robot_pose.COORDS.x))
            by = int(center[1] - px_per_m * (ball.y - robot_pose.COORDS.y))
            # Skip if outside image bounds
            if not (0 <= bx < self.w and 0 <= by < self.h):
                return
            cv2.circle(minimap, (bx, by), 6, color, thickness=cv2.FILLED, lineType=cv2.LINE_AA)

        # Draw closest ball
        draw_ball(closest_ball, self.BEST_DETECTION_COLOR)

        # Draw farther balls
        for ball in remaining_balls:
            draw_ball(ball, self.FAR_DETECTION_COLOR)

        return minimap

    
    def separate_closest_ball(self, detection_points: list[Vector2], robot_pos: Vector2) -> tuple[Vector2, list[Vector2]]:
        """Takes a nonempty list of detection points and returns the closest point and a list with that point removed."""
        if not detection_points:
            raise ValueError("detection_points must be nonempty")
        
        i, closest_ball = min(enumerate(detection_points), key=lambda p: (p[1]-robot_pos).norm())
        remaining = detection_points[:i] + detection_points[i+1:]
        return closest_ball, remaining

    def _on_shutdown_impl(self) -> None:
        WebSocketInterface.close()