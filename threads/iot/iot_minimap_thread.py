from utils.pi_thread import PiThread
from utils.load_settings import load_settings
from utils.websocket_interface import WebSocketInterface
from utils.pose2 import Pose2
from utils.vector2 import Vector2
from threads.peripherals.sensor_thread import SensorThread
from threads.vision.inference_thread import InferenceThread
from threads.controller.controller_thread import ControllerThread
import numpy as np
import math
import cv2

class IoTMinimapThread(PiThread):
    ROBOT_POSE: Pose2 | None
    """Global robot X and Y coordinates in meters and heading in radians."""

    # === Loaded parameters ===
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

    trail_length: int
    """Length of "track marks" left by robot while driving (number of list items)."""

    trail: list[tuple[Vector2, Vector2]]
    """A list of "track mark" coordinates in meters, where the first & second values are the left & right trails, respectively."""

    # === Calculated parameters ===
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
        self.trail_length = minimap_settings["trail_length"]
        self.trail = []
        # From drive config, but still used here
        self.wheel_base = settings["drive_config"]["wheel_base"]

        # Calculated parameters
        self.aspect_ratio = self.w / self.h
        self.meters_per_pixel = self.width_meters / self.w
        self.grid_width = 1.0 / self.meter_subdivisions

        # # Connect to WebSocket
        # WebSocketInterface._ensure_socket(self)

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

        # --- Update and draw track marks ---
        # Helper function
        def real_to_px(real_coords: Vector2) -> tuple[int, int]:
            """Convert the given coordinates in meters to pixel coordinates, using the robot's current pose."""
            px = int(center[0] + px_per_m * (real_coords.x - robot_pose.COORDS.x))
            py = int(center[1] - px_per_m * (real_coords.y - robot_pose.COORDS.y))
            return (px, py)
        
        # Update trail list
        heading_line = Vector2(self.wheel_base/2, 0).rotate(robot_pose.h)
        left_point = robot_pose.COORDS + heading_line.rotate(math.pi/2)
        right_point = robot_pose.COORDS + heading_line.rotate(-math.pi/2)
        self.trail.append((left_point, right_point))
        while len(self.trail) > self.trail_length:
            del self.trail[0] # truncate list to maximum length

        # Draw trails
        def index_to_color(i: int) -> tuple[int, int, int]:
            """Gets the appropriate grayscale color for the given segment of the trail list."""
            proportion = (self.trail_length - (len(self.trail) - i)) / self.trail_length
            value: int = int(self.BG_COLOR[0] + proportion * (255 - self.BG_COLOR[0]))
            return (value, value, value)

        if len(self.trail) > 1:
            left_trail = [points[0] for points in self.trail]
            right_trail = [points[1] for points in self.trail]
            for i in range(len(left_trail)-1):
                color = index_to_color(i)
                # left
                p1l = real_to_px(left_trail[i])
                p2l = real_to_px(left_trail[i+1])
                cv2.line(minimap, p1l, p2l, color, 1, cv2.LINE_AA)
                # right
                p1r = real_to_px(right_trail[i])
                p2r = real_to_px(right_trail[i+1])
                cv2.line(minimap, p1r, p2r, color, 1, cv2.LINE_AA)

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
        # Helper function
        def draw_ball(ball: Vector2, color: tuple[int, int, int]) -> tuple[int, int]:
            """Draw the current ball, and return its converted pixel coordinates."""
            bx, by = real_to_px(ball)
            # Skip if outside image bounds
            if not (0 <= bx < self.w and 0 <= by < self.h):
                return (-1, -1)
            cv2.circle(minimap, (bx, by), 6, color, thickness=cv2.FILLED, lineType=cv2.LINE_AA)
            return (bx, by)

        # First draw all current detections
        detection_points: dict[str, list[Vector2]] = InferenceThread["detection.points"] or {
            "relative_points": [],
            "absolute_points": []
        }
        relative_points = detection_points["relative_points"]
        absolute_points = detection_points["absolute_points"]
        if 0 < len(relative_points) == len(absolute_points):
            for ball in absolute_points:
                draw_ball(ball, self.FAR_DETECTION_COLOR)
        
        # Then draw currently targeted ball
        target_ball_points: tuple[Vector2, Vector2] | None = ControllerThread["target_ball_points"]
        if target_ball_points is not None:
            target_px = draw_ball(target_ball_points[1], self.BEST_DETECTION_COLOR) # Absolute position of closest ball

            # Draw desired trajectory
            if target_px[0]>=0 and target_px[1]>=0:
                cv2.line(minimap, center, target_px, (0, 255, 255), thickness=1, lineType=cv2.LINE_AA)
            
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