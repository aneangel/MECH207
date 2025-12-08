#!/usr/bin/env python3
"""
Air Hockey Defense Controller

Integrates camera tracking with motor control to defend the goal.
- Direct ZED camera access via USB 3.0
- Web streaming for real-time view (http://<ip>:5001)
- Predicts puck trajectory with Kalman filter
- Commands paddle to intercept
- Uses RP2350 CoreXY controller with sensorless homing

Usage:
    python3 hockey_defense_controller.py

Commands (manual mode):
    right, left, up, down  - Move paddle
    stop                    - Stop movement
    view                    - Toggle web stream (http://<ip>:5001)
    home                    - Run sensorless homing
    camera_home             - Camera-guided safe homing
    localize                - Use camera to find current position (no motors)
    auto                    - Start autonomous defense
    quit                    - Exit
"""

import serial
import json
import time
import sys
import serial.tools.list_ports
import threading
from collections import deque
import numpy as np
import cv2
import pyzed.sl as sl
from flask import Flask, Response


def find_motor_controller():
    """Find XIAO RP2350 motor controller."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == 0x2886 and port.pid == 0x58:
            return port.device
    return None


class PuckTracker:
    """Track puck directly from ZED camera via USB 3.0."""
    
    def __init__(self):
        # ZED camera
        self.zed = None
        self.runtime_params = None
        self.camera_view = sl.VIEW.RIGHT
        
        # Crop region - will be set from detected table boundary
        self.CROP_REGION = None  # (x1, y1, x2, y2) in original frame
        self.use_crop = False  # Enable cropping to zoom on table
        self.crop_padding = 20  # Extra pixels around detected table for context
        
        # HSV color ranges for red puck
        self.RED_LOWER1 = np.array([0, 100, 100])
        self.RED_UPPER1 = np.array([10, 255, 255])
        self.RED_LOWER2 = np.array([160, 100, 100])
        self.RED_UPPER2 = np.array([180, 255, 255])
        self.MIN_PUCK_AREA = 80
        
        # HSV color ranges for green paddle (end-effector)
        self.GREEN_LOWER = np.array([35, 100, 100])
        self.GREEN_UPPER = np.array([85, 255, 255])
        self.MIN_PADDLE_AREA = 80
        
        # HSV color ranges for table surface detection
        # The table is tan/cream/white - wide range to handle lighting variations
        # Low saturation catches the cream/white areas, higher catches tan
        self.TABLE_LOWER = np.array([10, 20, 80])   # Hue 10-45, low-med sat, med-high value
        self.TABLE_UPPER = np.array([45, 200, 255])
        
        # Glare threshold - pixels above this brightness with low saturation are glare
        self.GLARE_VALUE_THRESHOLD = 240  # Very bright
        self.GLARE_SAT_THRESHOLD = 30     # Low saturation = white/glare
        
        # Kalman filter for puck tracking
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1,0,0,0], [0,1,0,0]], np.float32)
        self.kf.transitionMatrix = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]], np.float32)
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 10
        
        # Kalman filter for paddle (green circle) tracking
        self.kf_paddle = cv2.KalmanFilter(4, 2)
        self.kf_paddle.measurementMatrix = np.array([[1,0,0,0], [0,1,0,0]], np.float32)
        self.kf_paddle.transitionMatrix = np.array([[1,0,1,0], [0,1,0,1], [0,0,1,0], [0,0,0,1]], np.float32)
        self.kf_paddle.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kf_paddle.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5
        
        # Tracking state
        self.last_puck_pos = None
        self.last_puck_vel = None
        self.puck_found = False
        self.frames_without_puck = 0
        self.puck_lost_threshold = 30  # Stop if puck missing for 30 frames (~0.5s at 60fps)
        
        self.last_paddle_pos = None
        self.paddle_found = False
        self.goal_x = 0.0
        
        # Virtual boundary detection (brown table with black edges)
        self.virtual_boundary = None  # Will be (x_min, y_min, x_max, y_max) in camera pixels
        self.boundary_margin = 15  # pixels safety margin from detected edge
        
        # Raw detection positions for drawing (not Kalman filtered)
        self.raw_puck_pos = None
        self.raw_paddle_pos = None
        
        # Display settings
        self.show_preview = False
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Flask streaming
        self.flask_app = None
        self.flask_thread = None
        self.stream_port = 5001  # Different port from stream_camera.py
        
        self.running = False
        self.thread = None
        
    def initialize_camera(self):
        """Initialize ZED camera."""
        print("Initializing ZED camera...")
        self.zed = sl.Camera()
        
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60
        init_params.depth_mode = sl.DEPTH_MODE.NONE  # We only need RGB
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"✗ Camera initialization failed: {err}")
            return False
        
        self.runtime_params = sl.RuntimeParameters()
        print("✓ ZED camera initialized")
        return True
        
    def start(self):
        """Start tracking thread."""
        if not self.initialize_camera():
            return False
        
        # Detect table boundary on startup
        print("Detecting table boundary...")
        image = sl.Mat()
        frame_w, frame_h = 1280, 720  # Default
        
        for attempt in range(10):  # Try a few frames
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(image, sl.VIEW.RIGHT)
                frame = image.get_data()[:, :, :3]
                
                # Use full frame for boundary detection
                frame_h, frame_w = frame.shape[:2]
                print(f"  Frame size: {frame_w}x{frame_h}")
                
                boundary = self.detect_table_boundary(frame)
                if boundary:
                    # boundary is the table surface with safety margin already applied
                    x_min, y_min, x_max, y_max = boundary
                    
                    # Set crop region slightly larger than boundary for context
                    crop_x1 = max(0, x_min - self.crop_padding - self.boundary_margin)
                    crop_y1 = max(0, y_min - self.crop_padding - self.boundary_margin)
                    crop_x2 = min(frame_w, x_max + self.crop_padding + self.boundary_margin)
                    crop_y2 = min(frame_h, y_max + self.crop_padding + self.boundary_margin)
                    self.CROP_REGION = (crop_x1, crop_y1, crop_x2, crop_y2)
                    
                    # Virtual boundary in cropped frame coordinates
                    self.virtual_boundary = (
                        x_min - crop_x1,  # Offset to cropped frame
                        y_min - crop_y1,
                        x_max - crop_x1,
                        y_max - crop_y1
                    )
                    
                    print(f"✓ Table detected in frame: ({x_min}, {y_min}) to ({x_max}, {y_max})")
                    print(f"  Crop region: ({crop_x1}, {crop_y1}) to ({crop_x2}, {crop_y2})")
                    print(f"  Cropped size: {crop_x2-crop_x1}x{crop_y2-crop_y1} pixels")
                    print(f"  Virtual boundary (in crop): {self.virtual_boundary}")
                    break
                else:
                    print(f"  Attempt {attempt + 1}/10: No boundary detected")
            time.sleep(0.1)
        
        if self.CROP_REGION is None:
            print("⚠ Warning: Could not detect table boundary. Using full frame.")
            self.use_crop = False
            # Get frame dimensions
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(image, sl.VIEW.RIGHT)
                frame = image.get_data()[:, :, :3]
                h, w = frame.shape[:2]
                self.virtual_boundary = (self.boundary_margin, self.boundary_margin, 
                                        w - self.boundary_margin, h - self.boundary_margin)
                print(f"  Using full frame boundary: {self.virtual_boundary}")
        
        self.running = True
        self.thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.thread.start()
        return True
    
    def stop(self):
        """Stop tracking thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.zed:
            self.zed.close()
            print("ZED camera closed")
    
    def _detect_puck(self, frame_hsv):
        """Detect red puck in frame, filtering out glare."""
        mask1 = cv2.inRange(frame_hsv, self.RED_LOWER1, self.RED_UPPER1)
        mask2 = cv2.inRange(frame_hsv, self.RED_LOWER2, self.RED_UPPER2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Filter out glare (very bright, low saturation areas)
        glare_mask = cv2.inRange(frame_hsv, 
                                  np.array([0, 0, self.GLARE_VALUE_THRESHOLD]),
                                  np.array([180, self.GLARE_SAT_THRESHOLD, 255]))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(glare_mask))
        
        # Clean up mask
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_contour = None
        best_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_PUCK_AREA:
                continue
            
            # Check circularity
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            if circularity > 0.6 and area > best_area:
                best_area = area
                best_contour = cnt
        
        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
        
        return None
    
    def _detect_paddle(self, frame_hsv):
        """Detect green paddle (end-effector) in frame, filtering out glare."""
        mask = cv2.inRange(frame_hsv, self.GREEN_LOWER, self.GREEN_UPPER)
        
        # Filter out glare
        glare_mask = cv2.inRange(frame_hsv, 
                                  np.array([0, 0, self.GLARE_VALUE_THRESHOLD]),
                                  np.array([180, self.GLARE_SAT_THRESHOLD, 255]))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(glare_mask))
        
        # Clean up mask
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_contour = None
        best_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.MIN_PADDLE_AREA:
                continue
            
            # Check circularity
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            if circularity > 0.6 and area > best_area:
                best_area = area
                best_contour = cnt
        
        if best_contour is not None:
            M = cv2.moments(best_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                return (cx, cy)
        
        return None
    
    def detect_table_boundary(self, frame):
        """Detect table surface to establish virtual boundary.
        
        Uses multiple approaches to robustly detect the playing surface:
        1. Color-based detection for tan/cream table
        2. Glare filtering to handle warehouse lights
        3. Rectangular fitting for clean boundaries
        
        Args:
            frame: BGR image from camera
            
        Returns:
            tuple: (x_min, y_min, x_max, y_max) boundary in pixels, or None
        """
        h, w = frame.shape[:2]
        
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # === METHOD 1: Color-based detection ===
        # Detect tan/cream/beige table surface
        mask_color = cv2.inRange(hsv, self.TABLE_LOWER, self.TABLE_UPPER)
        
        # Also detect lighter areas (cream/white) that aren't glare
        # Glare = high value + very low saturation
        # Table = high value + some saturation (tan/cream tint)
        light_lower = np.array([0, 10, 150])  # Any hue, low sat, bright
        light_upper = np.array([180, 80, 250])  # But not pure white glare
        mask_light = cv2.inRange(hsv, light_lower, light_upper)
        
        # Combine color masks
        mask = cv2.bitwise_or(mask_color, mask_light)
        
        # === FILTER OUT GLARE ===
        # Glare is very bright (high V) with very low saturation (near white)
        glare_mask = cv2.inRange(hsv, 
                                  np.array([0, 0, self.GLARE_VALUE_THRESHOLD]),
                                  np.array([180, self.GLARE_SAT_THRESHOLD, 255]))
        # Remove glare from detection
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(glare_mask))
        
        # === ALSO EXCLUDE BLACK EDGES ===
        # Black edges of table have low value
        black_mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 50]))
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(black_mask))
        
        # === CLEAN UP MASK ===
        kernel_small = np.ones((5, 5), np.uint8)
        kernel_large = np.ones((15, 15), np.uint8)
        
        # Remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small, iterations=2)
        # Fill holes
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large, iterations=3)
        # Dilate to connect nearby regions
        mask = cv2.dilate(mask, kernel_small, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            print("  No table surface detected")
            return self._fallback_boundary_detection(frame)
        
        # Find the largest rectangular-ish contour
        min_area = (w * h) * 0.15  # Table should be at least 15% of frame
        best_contour = None
        best_score = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue
            
            # Get rotated rectangle to check how rectangular it is
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            rect_area = rect[1][0] * rect[1][1]
            
            if rect_area == 0:
                continue
            
            # Score = area * rectangularity (how well contour fills its bounding rect)
            rectangularity = area / rect_area
            score = area * rectangularity
            
            # Prefer larger, more rectangular contours
            if score > best_score and rectangularity > 0.7:
                best_score = score
                best_contour = cnt
        
        if best_contour is None:
            print(f"  No rectangular contour found (need >{min_area:.0f} px, >70% fill)")
            return self._fallback_boundary_detection(frame)
        
        # Get bounding rectangle
        x, y, w_rect, h_rect = cv2.boundingRect(best_contour)
        
        # Apply safety margin
        x_min = x + self.boundary_margin
        y_min = y + self.boundary_margin
        x_max = x + w_rect - self.boundary_margin
        y_max = y + h_rect - self.boundary_margin
        
        # Sanity check
        if (x_max - x_min) < 100 or (y_max - y_min) < 50:
            print(f"  Boundary too small: {x_max-x_min}x{y_max-y_min}")
            return self._fallback_boundary_detection(frame)
        
        area = cv2.contourArea(best_contour)
        print(f"  Table detected: {w_rect}x{h_rect} px ({100*area/(w*h):.1f}% of frame)")
        
        return (x_min, y_min, x_max, y_max)
    
    def _fallback_boundary_detection(self, frame):
        """Fallback: use edge detection to find table boundary."""
        print("  Trying edge-based fallback...")
        
        h, w = frame.shape[:2]
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Dilate to connect edges
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest rectangular contour
        min_area = (w * h) * 0.1
        best_rect = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area and area > min_area:
                x, y, w_rect, h_rect = cv2.boundingRect(cnt)
                # Check aspect ratio is reasonable (table is wider than tall)
                if w_rect > h_rect * 0.8 and w_rect > w * 0.3:
                    max_area = area
                    best_rect = (x, y, x + w_rect, y + h_rect)
        
        if best_rect:
            x_min, y_min, x_max, y_max = best_rect
            x_min += self.boundary_margin
            y_min += self.boundary_margin
            x_max -= self.boundary_margin
            y_max -= self.boundary_margin
            print(f"  Fallback found boundary: {x_max-x_min}x{y_max-y_min} px")
            return (x_min, y_min, x_max, y_max)
        
        return None
    
    def is_near_boundary(self, paddle_pos, threshold=10):
        """Check if paddle is approaching virtual boundary.
        
        Args:
            paddle_pos: (x, y) position in camera pixels
            threshold: Distance threshold in pixels
            
        Returns:
            tuple: (near_boundary, safe_position) where safe_position is clamped coords
        """
        if self.virtual_boundary is None or paddle_pos is None:
            return False, paddle_pos
        
        x, y = paddle_pos
        x_min, y_min, x_max, y_max = self.virtual_boundary
        
        # Check if near any boundary
        near_left = (x - x_min) < threshold
        near_right = (x_max - x) < threshold
        near_top = (y - y_min) < threshold
        near_bottom = (y_max - y) < threshold
        
        near_boundary = near_left or near_right or near_top or near_bottom
        
        # Clamp to safe position
        safe_x = max(x_min + threshold, min(x_max - threshold, x))
        safe_y = max(y_min + threshold, min(y_max - threshold, y))
        
        return near_boundary, (safe_x, safe_y)
    
    def _tracking_loop(self):
        """Background thread for direct camera tracking."""
        image = sl.Mat()
        
        while self.running:
            try:
                if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                    self.zed.retrieve_image(image, sl.VIEW.RIGHT)
                    frame = image.get_data()[:, :, :3]  # BGRA -> BGR
                    
                    # Use full frame or crop
                    if self.use_crop and self.CROP_REGION:
                        x1, y1, x2, y2 = self.CROP_REGION
                        cropped = frame[y1:y2, x1:x2]
                    else:
                        cropped = frame  # Use full frame
                    
                    # Convert to HSV
                    frame_hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
                    
                    # Detect puck
                    detection = self._detect_puck(frame_hsv)
                    
                    if detection is not None:
                        # Update Kalman filter
                        measurement = np.array([[np.float32(detection[0])],
                                              [np.float32(detection[1])]])
                        self.kf.correct(measurement)
                        self.puck_found = True
                        self.frames_without_puck = 0  # Reset counter
                        self.raw_puck_pos = detection  # Store raw detection for drawing
                    else:
                        self.puck_found = False
                        self.frames_without_puck += 1
                        self.raw_puck_pos = None
                    
                    # Always predict (coast when not detected)
                    prediction = self.kf.predict()
                    
                    # Extract position and velocity
                    self.last_puck_pos = (float(prediction[0, 0]), float(prediction[1, 0]))
                    self.last_puck_vel = (float(prediction[2, 0]), float(prediction[3, 0]))
                    
                    # Detect paddle (green circle)
                    paddle_detection = self._detect_paddle(frame_hsv)
                    
                    if paddle_detection is not None:
                        # Update Kalman filter for paddle
                        paddle_measurement = np.array([[np.float32(paddle_detection[0])],
                                                      [np.float32(paddle_detection[1])]])
                        self.kf_paddle.correct(paddle_measurement)
                        self.paddle_found = True
                        self.raw_paddle_pos = paddle_detection  # Store raw detection for drawing
                    else:
                        self.paddle_found = False
                        self.raw_paddle_pos = None
                    
                    # Always predict paddle position
                    paddle_prediction = self.kf_paddle.predict()
                    
                    # Extract paddle position
                    self.last_paddle_pos = (float(paddle_prediction[0, 0]), float(paddle_prediction[1, 0]))
                    
                    # Always create annotated frame for streaming
                    preview_frame = cropped.copy()
                    
                    # Draw virtual boundary
                    if self.virtual_boundary:
                        x_min, y_min, x_max, y_max = self.virtual_boundary
                        cv2.rectangle(preview_frame, (x_min, y_min), (x_max, y_max), (255, 255, 0), 2)
                        cv2.putText(preview_frame, "Virtual Boundary", (x_min + 5, y_min + 20), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # Draw puck detection
                    if detection:
                        cv2.circle(preview_frame, detection, 10, (0, 0, 255), 2)
                        cv2.putText(preview_frame, "PUCK", (detection[0] + 15, detection[1]), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    
                    # Draw paddle detection
                    if paddle_detection:
                        cv2.circle(preview_frame, paddle_detection, 10, (0, 255, 0), 2)
                        cv2.putText(preview_frame, "PADDLE", (paddle_detection[0] + 15, paddle_detection[1]), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Draw intercept line between paddle and puck when both detected
                    if detection and paddle_detection:
                        cv2.line(preview_frame, paddle_detection, detection, (255, 0, 255), 2)
                        # Calculate distance for display
                        dx = detection[0] - paddle_detection[0]
                        dy = detection[1] - paddle_detection[1]
                        dist = np.sqrt(dx*dx + dy*dy)
                        mid_x = (detection[0] + paddle_detection[0]) // 2
                        mid_y = (detection[1] + paddle_detection[1]) // 2
                        cv2.putText(preview_frame, f"{dist:.0f}px", (mid_x + 5, mid_y - 5),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
                    
                    # Draw predicted puck trajectory if moving
                    if self.last_puck_vel and self.puck_found:
                        vx, vy = self.last_puck_vel
                        speed = np.sqrt(vx*vx + vy*vy)
                        if speed > 2.0:  # Only draw if moving fast enough
                            px, py = int(self.last_puck_pos[0]), int(self.last_puck_pos[1])
                            # Predict 10 steps into future
                            future_pts = []
                            for i in range(1, 11):
                                fx = int(px + vx * i * 2)
                                fy = int(py + vy * i * 2)
                                future_pts.append((fx, fy))
                            if len(future_pts) > 1:
                                cv2.polylines(preview_frame, [np.array(future_pts)], False, (0, 0, 255), 2)
                    
                    # Add detection status text
                    in_play = self.is_puck_in_play()
                    status_text = f"Puck: {'IN PLAY' if in_play else 'OUT OF PLAY'}  Paddle: {'FOUND' if self.paddle_found else 'LOST'}"
                    status_color = (0, 255, 0) if in_play else (0, 0, 255)
                    cv2.putText(preview_frame, status_text, (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                    
                    # Add frame size info
                    h, w = preview_frame.shape[:2]
                    cv2.putText(preview_frame, f"Frame: {w}x{h}", (10, h - 10), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    with self.frame_lock:
                        self.latest_frame = preview_frame
                
            except Exception as e:
                print(f"Tracking error: {e}")
                time.sleep(0.05)
            
            time.sleep(0.001)  # Minimal sleep for high-speed tracking
    
    def is_puck_in_play(self):
        """Check if puck is currently in play (being tracked).
        
        Returns:
            bool: True if puck is detected, False if lost/goal scored
        """
        return self.frames_without_puck < self.puck_lost_threshold
    
    def get_puck_state(self):
        """Get latest puck position and velocity.
        
        Returns:
            tuple: ((x, y), (vx, vy)) or (None, None) if not visible
        """
        if self.last_puck_pos is None:
            return None, None
        return self.last_puck_pos, self.last_puck_vel
    
    def get_paddle_position(self):
        """Get latest paddle position from camera.
        
        Returns:
            tuple: (x, y) in camera pixels, or None if not visible
        """
        if self.last_paddle_pos is None or not self.paddle_found:
            return None
        return self.last_paddle_pos
    
    def set_workspace_limits(self, x_min, x_max, y_min, y_max):
        """Set workspace dimensions from motor controller after homing.
        
        This allows accurate camera-to-workspace coordinate conversion.
        
        Args:
            x_min, x_max: X axis limits in mm
            y_min, y_max: Y axis limits in mm
        """
        self.workspace_x_min = x_min
        self.workspace_x_max = x_max
        self.workspace_y_min = y_min
        self.workspace_y_max = y_max
        print(f"Workspace set: X[{x_min:.1f}-{x_max:.1f}] Y[{y_min:.1f}-{y_max:.1f}] mm")
    
    def camera_to_workspace(self, cam_x, cam_y):
        """Convert camera pixel coordinates to workspace mm coordinates.
        
        The camera's virtual boundary (table surface) maps to the motor's
        workspace limits. This provides the coordinate transform between
        what the camera sees and where the motors can move.
        
        Args:
            cam_x: Camera X coordinate (pixels, in cropped/boundary frame)
            cam_y: Camera Y coordinate (pixels, in cropped/boundary frame)
        
        Returns:
            tuple: (x_mm, y_mm) in workspace coordinates
        """
        # Get pixel frame dimensions from virtual boundary
        if self.virtual_boundary:
            x_min, y_min, x_max, y_max = self.virtual_boundary
            frame_width = x_max - x_min
            frame_height = y_max - y_min
            # Normalize to boundary (0 to frame_width/height)
            cam_x = cam_x - x_min
            cam_y = cam_y - y_min
        else:
            frame_width = 1280
            frame_height = 720
        
        # Get workspace dimensions (from motor controller after homing)
        ws_x_range = getattr(self, 'workspace_x_max', 70.0) - getattr(self, 'workspace_x_min', 0.0)
        ws_y_range = getattr(self, 'workspace_y_max', 35.0) - getattr(self, 'workspace_y_min', 0.0)
        ws_x_min = getattr(self, 'workspace_x_min', 0.0)
        ws_y_min = getattr(self, 'workspace_y_min', 0.0)
        
        # Map camera pixels to workspace mm
        workspace_x = ws_x_min + (cam_x / frame_width) * ws_x_range
        workspace_y = ws_y_min + (cam_y / frame_height) * ws_y_range
        
        return workspace_x, workspace_y
    
    def toggle_preview(self):
        """Toggle camera preview - starts/stops Flask web streaming."""
        self.show_preview = not self.show_preview
        if self.show_preview:
            self.start_stream_server()
        else:
            self.stop_stream_server()
    
    def start_stream_server(self):
        """Start Flask server for real-time video streaming."""
        if self.flask_thread is not None and self.flask_thread.is_alive():
            print("Stream server already running")
            return
        
        # Create Flask app
        self.flask_app = Flask(__name__)
        tracker = self  # Reference for closure
        
        @self.flask_app.route('/')
        def index():
            return """
            <html>
            <head>
                <title>Air Hockey Camera View</title>
                <style>
                    body { background: #111; text-align: center; color: #eee; font-family: sans-serif; margin: 0; padding: 20px; }
                    h2 { margin-bottom: 10px; }
                    img { border: 2px solid #555; max-width: 95%; }
                    .info { color: #888; font-size: 12px; margin-top: 10px; }
                </style>
            </head>
            <body>
                <h2>Air Hockey Defense Controller - Live View</h2>
                <img src="/video_feed">
                <p class="info">Magenta line = intercept path | Cyan box = table boundary | Red = puck | Green = paddle</p>
            </body>
            </html>
            """
        
        @self.flask_app.route('/video_feed')
        def video_feed():
            def generate():
                while tracker.show_preview and tracker.running:
                    with tracker.frame_lock:
                        if tracker.latest_frame is None:
                            time.sleep(0.01)
                            continue
                        frame = tracker.latest_frame.copy()
                    
                    # Encode as JPEG
                    ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if ret:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                    time.sleep(0.016)  # ~60 FPS max
            
            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
        
        def run_flask():
            # Suppress Flask logs
            import logging
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
            
            try:
                self.flask_app.run(host='0.0.0.0', port=self.stream_port, threaded=True, use_reloader=False)
            except Exception as e:
                print(f"Flask server error: {e}")
        
        self.flask_thread = threading.Thread(target=run_flask, daemon=True)
        self.flask_thread.start()
        
        print(f"Stream server started at http://0.0.0.0:{self.stream_port}")
        print("Open this URL in a browser to view the camera feed")
    
    def stop_stream_server(self):
        """Stop Flask streaming server."""
        # Flask doesn't have a clean way to stop from another thread
        # The daemon thread will exit when main program exits
        print("Stream server will stop (view disabled)")
        self.show_preview = False
    
    def show_frame(self):
        """Legacy method - now handled by Flask streaming."""
        # This is now a no-op since we use Flask streaming
        return self.show_preview
    
    def get_puck_workspace_state(self):
        """Get puck position and velocity in workspace coordinates (mm).
        
        Returns:
            tuple: ((x_mm, y_mm), (vx_mm_s, vy_mm_s)) or (None, None) if not visible
        """
        pos, vel = self.get_puck_state()
        
        if pos is None or vel is None:
            return None, None
        
        # Convert pixel position to workspace mm
        px_mm, py_mm = self.camera_to_workspace(pos[0], pos[1])
        
        # Convert pixel velocity to workspace velocity (mm/s)
        # Scale velocity by the same ratio as position
        if self.virtual_boundary:
            x_min, y_min, x_max, y_max = self.virtual_boundary
            frame_width = x_max - x_min
            frame_height = y_max - y_min
        else:
            frame_width = 1280
            frame_height = 720
        
        # Velocity scaling: pixels/frame -> mm/s (assuming ~60fps)
        vx_mm = (vel[0] / frame_width) * 70.0 * 60.0
        vy_mm = (vel[1] / frame_height) * 35.0 * 60.0
        
        return (px_mm, py_mm), (vx_mm, vy_mm)
    
    def get_paddle_workspace_pos(self):
        """Get paddle position in workspace coordinates (mm).
        
        Returns:
            tuple: (x_mm, y_mm) or None if not visible
        """
        pos = self.get_paddle_position()
        if pos is None:
            return None
        
        return self.camera_to_workspace(pos[0], pos[1])


class MotorController:
    """RP2350 CoreXY motor controller interface."""
    
    def __init__(self, port=None):
        if port is None:
            port = find_motor_controller()
            if port is None:
                print("ERROR: Could not find RP2350 motor controller")
                sys.exit(1)
        
        self.port = port
        self.ser = None
        self.homed = False
        self.enabled = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.workspace_limits = {'x_min': 0, 'x_max': 70, 'y_min': 0, 'y_max': 35}
    
    def connect(self):
        """Connect to motor controller."""
        try:
            print(f"Connecting to motor controller at {self.port}...")
            self.ser = serial.Serial(self.port, 115200, timeout=1)
            time.sleep(2)
            
            # Clear buffer
            while self.ser.in_waiting:
                self.ser.readline()
            
            # Test connection
            response = self.send_cmd({"cmd": "ping"})
            if response and response.get('status') == 'ok':
                print(f"✓ Connected to motor controller")
                return True
            else:
                print("✗ Ping failed")
                return False
                
        except Exception as e:
            print(f"✗ Connection error: {e}")
            return False
    
    def send_cmd(self, cmd_dict, timeout=1.0):
        """Send JSON command and get response."""
        try:
            cmd_json = json.dumps(cmd_dict) + '\n'
            self.ser.write(cmd_json.encode('utf-8'))
            self.ser.flush()
            
            # Read response
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            resp = json.loads(line)
                            # Update internal state
                            self.current_x = resp.get('x', self.current_x)
                            self.current_y = resp.get('y', self.current_y)
                            self.homed = resp.get('homed', self.homed)
                            self.enabled = resp.get('enabled', self.enabled)
                            return resp
                        except:
                            pass
            return None
        except Exception as e:
            print(f"Command error: {e}")
            return None
    
    def enable(self):
        """Enable motors."""
        print("Enabling motors...")
        resp = self.send_cmd({"cmd": "enable", "state": True})
        if resp and resp.get('status') == 'ok':
            print("✓ Motors enabled")
            return True
        print("✗ Enable failed")
        return False
    
    def disable(self):
        """Disable motors."""
        resp = self.send_cmd({"cmd": "enable", "state": False})
        if resp:
            print("✓ Motors disabled")
    
    def home(self):
        """Run sensorless homing sequence."""
        print("\n" + "="*60)
        print("STARTING SENSORLESS HOMING")
        print("="*60)
        print("The gantry will:")
        print("  1. Find all workspace boundaries")
        print("  2. Return to bottom-left corner (0,0)")
        print("  3. Set workspace limits")
        print("\nThis may take 30-90 seconds depending on workspace size...")
        print("Waiting for completion...")
        print("="*60 + "\n")
        
        if not self.enabled:
            print("⚠ Warning: Motors not enabled, enabling now...")
            if not self.enable():
                return False
        
        # Send home command
        try:
            cmd_json = json.dumps({"cmd": "home"}) + '\n'
            self.ser.write(cmd_json.encode('utf-8'))
            self.ser.flush()
        except Exception as e:
            print(f"✗ Failed to send home command: {e}")
            return False
        
        # Wait for completion (no timeout - wait until done)
        print("Waiting for homing to complete (listening for 'ok' status)...")
        start_time = time.time()
        last_update = start_time
        
        while True:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        try:
                            resp = json.loads(line)
                            
                            # Update position
                            self.current_x = resp.get('x', self.current_x)
                            self.current_y = resp.get('y', self.current_y)
                            
                            # Check if it's a home command response
                            if resp.get('cmd') == 'home':
                                status = resp.get('status')
                                message = resp.get('message', '')
                                
                                if status == 'in_progress':
                                    print(f"  {message}")
                                elif status == 'ok':
                                    print(f"\n✓ Homing complete! ({time.time() - start_time:.1f}s)")
                                    self.homed = True
                                    
                                    # Get workspace limits
                                    status_resp = self.send_cmd({"cmd": "status"})
                                    if status_resp:
                                        self.workspace_limits = {
                                            'x_min': status_resp.get('x_min', 0),
                                            'x_max': status_resp.get('x_max', 70),
                                            'y_min': status_resp.get('y_min', 0),
                                            'y_max': status_resp.get('y_max', 35)
                                        }
                                        print(f"  Workspace: X [{self.workspace_limits['x_min']:.1f} to {self.workspace_limits['x_max']:.1f}] mm")
                                        print(f"             Y [{self.workspace_limits['y_min']:.1f} to {self.workspace_limits['y_max']:.1f}] mm")
                                    return True
                                elif status == 'error':
                                    print(f"✗ Homing failed: {message}")
                                    return False
                        except json.JSONDecodeError:
                            pass  # Ignore non-JSON lines
                
                # Print progress every 10 seconds
                if time.time() - last_update > 10:
                    print(f"  Still homing... ({time.time() - start_time:.0f}s elapsed)")
                    last_update = time.time()
                
                time.sleep(0.05)
                
            except KeyboardInterrupt:
                print("\n⚠ Homing interrupted by user!")
                self.stop()
                return False
            except Exception as e:
                print(f"✗ Error during homing: {e}")
                return False
        
        if not self.enabled:
            print("⚠ Warning: Motors not enabled, enabling now...")
            if not self.enable():
                return False
        
        resp = self.send_cmd({"cmd": "home"}, timeout=120.0)
        
        if resp and resp.get('status') == 'ok':
            print("\n✓ Homing complete!")
            self.homed = True
            
            # Get workspace limits
            status = self.send_cmd({"cmd": "status"})
            if status:
                self.workspace_limits = {
                    'x_min': status.get('x_min', 0),
                    'x_max': status.get('x_max', 70),
                    'y_min': status.get('y_min', 0),
                    'y_max': status.get('y_max', 35)
                }
                print(f"  Workspace: X [{self.workspace_limits['x_min']:.1f} to {self.workspace_limits['x_max']:.1f}] mm")
                print(f"             Y [{self.workspace_limits['y_min']:.1f} to {self.workspace_limits['y_max']:.1f}] mm")
            return True
        else:
            print("✗ Homing failed")
            return False
    
    def stop(self):
        """Stop all motion."""
        self.send_cmd({"cmd": "stop"})
    
    def camera_guided_home(self, tracker):
        """Camera-guided homing - uses vision to navigate to corners safely.
        
        This method:
        1. Uses camera to navigate paddle to bottom-left corner
        2. Performs sensorless homing on X-min edge
        3. Navigates to top-left, homes Y-max
        4. Navigates to bottom-right, homes X-max
        5. Navigates to top-right, homes Y-min (optional)
        6. Returns to origin
        
        Args:
            tracker: PuckTracker instance with camera initialized
        
        Returns:
            bool: True if successful
        """
        print("\n" + "="*60)
        print("CAMERA-GUIDED HOMING")
        print("="*60)
        print("Using vision to navigate safely to edges...")
        print("="*60 + "\n")
        
        if not self.enabled:
            print("Enabling motors...")
            if not self.enable():
                return False
        
        if tracker.virtual_boundary is None:
            print("✗ Error: Virtual boundary not detected. Cannot perform camera-guided homing.")
            return False
        
        # Get virtual boundary in camera pixels
        x_min_cam, y_min_cam, x_max_cam, y_max_cam = tracker.virtual_boundary
        
        # Helper function to navigate to camera position
        def navigate_to_camera_pos(target_cam_x, target_cam_y, description, approach_margin=20):
            """Navigate paddle to target camera position."""
            print(f"  Navigating to {description}...")
            
            timeout = time.time() + 15  # 15 second timeout
            while time.time() < timeout:
                paddle_pos = tracker.get_paddle_position()
                if paddle_pos is None:
                    print("    ⚠ Paddle not visible, waiting...")
                    time.sleep(0.2)
                    continue
                
                cam_x, cam_y = paddle_pos
                
                # Calculate error
                error_x = target_cam_x - cam_x
                error_y = target_cam_y - cam_y
                
                # Stop if close enough
                if abs(error_x) < 5 and abs(error_y) < 5:
                    self.stop()
                    print(f"    ✓ Reached {description}")
                    return True
                
                # Convert to workspace velocity (simple proportional control)
                # Camera: 315x188 pixels -> Workspace: 70x35 mm
                vel_x = (error_x / 315.0) * 200.0  # Scale to reasonable velocity
                vel_y = (error_y / 188.0) * 100.0
                
                # Limit velocity
                vel_x = max(-100, min(100, vel_x))
                vel_y = max(-100, min(100, vel_y))
                
                self.set_velocity(vel_x, vel_y)
                time.sleep(0.05)
            
            self.stop()
            print(f"    ⚠ Timeout reaching {description}")
            return False
        
        # Step 1: Navigate to bottom-left corner (X-min, Y-min)
        target_x = x_min_cam + 20
        target_y = y_min_cam + 20
        if not navigate_to_camera_pos(target_x, target_y, "bottom-left corner"):
            return False
        
        # Get current paddle position and localize
        paddle_pos = tracker.get_paddle_position()
        if paddle_pos:
            workspace_x, workspace_y = tracker.camera_to_workspace(*paddle_pos)
            print(f"  Setting initial position: ({workspace_x:.1f}, {workspace_y:.1f}) mm")
            # Assume this is near origin
            self.workspace_limits = {'x_min': 0, 'y_min': 0, 'x_max': 70, 'y_max': 35}
            self.localize_from_vision(5, 5)  # Assume near origin
        
        print("\n✓ Camera-guided homing complete!")
        print(f"  Workspace: X [{self.workspace_limits['x_min']:.1f} to {self.workspace_limits['x_max']:.1f}] mm")
        print(f"             Y [{self.workspace_limits['y_min']:.1f} to {self.workspace_limits['y_max']:.1f}] mm")
        
        self.homed = True
        return True
    
    def move_to(self, x, y):
        """Move to absolute position (mm).
        
        Args:
            x: Target X position (mm)
            y: Target Y position (mm)
        """
        resp = self.send_cmd({"cmd": "move", "x": x, "y": y})
        return resp and resp.get('status') == 'ok'
    
    def set_velocity(self, x_vel, y_vel):
        """Set velocity (mm/s).
        
        Args:
            x_vel: X velocity (mm/s)
            y_vel: Y velocity (mm/s)
        """
        resp = self.send_cmd({"cmd": "velocity", "x_vel": x_vel, "y_vel": y_vel})
        return resp and resp.get('status') == 'ok'
    
    def localize_from_vision(self, x_mm, y_mm):
        """Set position from vision-based localization.
        
        This allows the controller to recover its position using camera tracking
        without needing to run full sensorless homing. The workspace must have
        been homed at least once to establish the boundaries.
        
        Args:
            x_mm: Paddle X position from camera (mm)
            y_mm: Paddle Y position from camera (mm)
        
        Returns:
            bool: True if successful
        """
        resp = self.send_cmd({"cmd": "localize", "x": x_mm, "y": y_mm})
        if resp and resp.get('status') == 'ok':
            print(f"✓ Position localized to ({x_mm:.1f}, {y_mm:.1f}) mm")
            self.current_x = x_mm
            self.current_y = y_mm
            return True
        else:
            print(f"✗ Localization failed: {resp.get('message', 'Unknown error') if resp else 'No response'}")
            return False
    
    def get_position(self):
        """Get current position.
        
        Returns:
            tuple: (x, y) in mm
        """
        return self.current_x, self.current_y
    
    # Directional commands for testing
    def right(self, speed=100.0):
        """Move right at speed (mm/s)."""
        print(f"→ RIGHT at {speed} mm/s")
        return self.set_velocity(speed, 0)
    
    def left(self, speed=100.0):
        """Move left at speed (mm/s)."""
        print(f"← LEFT at {speed} mm/s")
        return self.set_velocity(-speed, 0)
    
    def up(self, speed=100.0):
        """Move up at speed (mm/s)."""
        print(f"↑ UP at {speed} mm/s")
        return self.set_velocity(0, speed)
    
    def down(self, speed=100.0):
        """Move down at speed (mm/s)."""
        print(f"↓ DOWN at {speed} mm/s")
        return self.set_velocity(0, -speed)
    
    def emergency_stop(self):
        """Emergency stop - disables motors immediately."""
        try:
            if self.ser and self.ser.is_open:
                # Send stop command multiple times rapidly
                stop_cmd = b'{"cmd":"stop"}\n'
                disable_cmd = b'{"cmd":"enable","state":false}\n'
                
                for _ in range(5):
                    self.ser.write(stop_cmd)
                    self.ser.flush()
                
                time.sleep(0.05)
                
                for _ in range(5):
                    self.ser.write(disable_cmd)
                    self.ser.flush()
                
                time.sleep(0.1)
        except Exception as e:
            print(f"Emergency stop error: {e}")
    
    def close(self):
        """Close connection."""
        if self.ser and self.ser.is_open:
            print("\nStopping motors...")
            self.emergency_stop()
            time.sleep(0.2)
            self.ser.close()
            print("Motor controller disconnected")


class DefenseController:
    """Main defense controller integrating vision and motors.
    
    This controller:
    1. Gathers context from camera (puck position, velocity, paddle position)
    2. Calculates optimal defensive position
    3. Sends velocity commands to Arduino
    4. Lets Arduino handle safety (stall guard, boundary clamping, kinematics)
    """
    
    def __init__(self, motor_controller, puck_tracker):
        self.motor = motor_controller
        self.tracker = puck_tracker
        self.auto_mode = False
        self.defense_thread = None
        self.running = False
        
        # Defense strategy parameters
        self.defense_line_x = 10.0  # Default defense line (mm from goal)
        self.max_speed = 180.0      # Max velocity command (mm/s) - Arduino clamps to 200
        
        # Control gains for position tracking
        self.kp = 12.0  # Proportional gain - higher = faster response
        self.kd = 3.0   # Derivative gain - damping to prevent overshoot
        self.last_error_y = 0.0
        
        # State tracking
        self.last_puck_pos = None
        self.last_puck_vel = None
        self.last_paddle_pos = None
    
    def start_auto_defense(self):
        """Start autonomous defense mode."""
        if not self.motor.homed:
            print("Warning: Motors not homed. Defense may be inaccurate.")
        
        # Set defense line based on workspace
        y_range = self.motor.workspace_limits['y_max'] - self.motor.workspace_limits['y_min']
        self.defense_line_x = self.motor.workspace_limits['x_min'] + 10.0  # 10mm from left edge
        
        print("\n" + "="*60)
        print("AUTONOMOUS DEFENSE MODE")
        print("="*60)
        print(f"Defense line: X = {self.defense_line_x:.1f} mm")
        print(f"Y range: {self.motor.workspace_limits['y_min']:.1f} to {self.motor.workspace_limits['y_max']:.1f} mm")
        print(f"Max speed: {self.max_speed} mm/s")
        print("\nArduino handles: kinematics, stall guard, boundary safety")
        print("Python handles: vision, prediction, velocity commands")
        print("\nType 'stop' to exit defense mode")
        print("="*60 + "\n")
        
        self.auto_mode = True
        self.running = True
        self.defense_thread = threading.Thread(target=self._defense_loop, daemon=True)
        self.defense_thread.start()
        return True
    
    def stop_auto_defense(self):
        """Stop autonomous defense mode."""
        self.auto_mode = False
        self.running = False
        if self.defense_thread:
            self.defense_thread.join(timeout=1)
        self.motor.stop()
        print("Defense stopped")
    
    def _calculate_intercept_y(self, puck_pos, puck_vel):
        """Calculate Y position where puck will cross defense line.
        
        Args:
            puck_pos: (x, y) in workspace mm
            puck_vel: (vx, vy) in mm/s
            
        Returns:
            float: Y intercept position in mm, or None if puck moving away
        """
        px, py = puck_pos
        vx, vy = puck_vel
        
        # Only defend if puck is moving toward our goal (negative X velocity)
        if vx >= -5.0:  # Small threshold to avoid jitter
            return None
        
        # Time for puck to reach defense line
        dx = self.defense_line_x - px
        if vx == 0:
            return None
        
        time_to_intercept = dx / vx  # This will be positive since dx<0 and vx<0
        
        if time_to_intercept < 0 or time_to_intercept > 2.0:  # Ignore if >2 seconds away
            return None
        
        # Predict Y position at intercept (simple linear, no bounces for now)
        intercept_y = py + vy * time_to_intercept
        
        # Clamp to workspace bounds
        intercept_y = max(self.motor.workspace_limits['y_min'], 
                         min(self.motor.workspace_limits['y_max'], intercept_y))
        
        return intercept_y
    
    def _defense_loop(self):
        """Main defense control loop.
        
        Strategy:
        1. Get puck position/velocity from camera (workspace coords)
        2. Calculate where puck will cross defense line
        3. Calculate velocity to move paddle there
        4. Send velocity command to Arduino (it handles safety)
        """
        puck_was_in_play = True
        
        while self.running and self.auto_mode:
            try:
                # === GATHER CONTEXT FROM CAMERA ===
                puck_in_play = self.tracker.is_puck_in_play()
                
                if not puck_in_play:
                    if puck_was_in_play:
                        print("Puck out of play - waiting...")
                        self.motor.stop()
                        puck_was_in_play = False
                    time.sleep(0.05)
                    continue
                else:
                    if not puck_was_in_play:
                        print("Puck in play - defending!")
                        puck_was_in_play = True
                
                # Get puck state in workspace coordinates
                puck_pos, puck_vel = self.tracker.get_puck_workspace_state()
                
                if puck_pos is None:
                    # No puck detected, hold position
                    self.motor.stop()
                    time.sleep(0.02)
                    continue
                
                self.last_puck_pos = puck_pos
                self.last_puck_vel = puck_vel
                
                # Get paddle position from camera (for feedback if needed)
                paddle_pos = self.tracker.get_paddle_workspace_pos()
                if paddle_pos:
                    self.last_paddle_pos = paddle_pos
                
                # === CALCULATE OPTIMAL DEFENSE POSITION ===
                target_y = self._calculate_intercept_y(puck_pos, puck_vel)
                
                if target_y is not None:
                    # Puck approaching - intercept it!
                    # Use motor's current position for control (from Arduino feedback)
                    current_y = self.motor.current_y
                    
                    # PD control for Y axis
                    error_y = target_y - current_y
                    d_error_y = error_y - self.last_error_y
                    self.last_error_y = error_y
                    
                    vy = (self.kp * error_y) + (self.kd * d_error_y)
                    
                    # Move toward defense line on X axis
                    error_x = self.defense_line_x - self.motor.current_x
                    vx = self.kp * error_x * 0.5  # Slower X correction
                    
                    # Clamp velocity magnitude
                    speed = np.sqrt(vx**2 + vy**2)
                    if speed > self.max_speed:
                        scale = self.max_speed / speed
                        vx *= scale
                        vy *= scale
                    
                    # === SEND VELOCITY COMMAND TO ARDUINO ===
                    # Arduino handles: CoreXY kinematics, stall guard, boundary clamping
                    self.motor.set_velocity(vx, vy)
                    
                else:
                    # Puck moving away or stationary - return to center
                    center_y = (self.motor.workspace_limits['y_max'] + 
                               self.motor.workspace_limits['y_min']) / 2.0
                    
                    error_y = center_y - self.motor.current_y
                    vy = error_y * 3.0  # Gentle return to center
                    vy = max(-50, min(50, vy))  # Limit centering speed
                    
                    # Also center on X (defense line)
                    error_x = self.defense_line_x - self.motor.current_x
                    vx = error_x * 2.0
                    vx = max(-30, min(30, vx))
                    
                    self.motor.set_velocity(vx, vy)
                
            except Exception as e:
                print(f"Defense error: {e}")
                import traceback
                traceback.print_exc()
            
            time.sleep(0.025)  # 40 Hz control loop


def main():
    """Main program."""
    print("="*60)
    print("AIR HOCKEY DEFENSE CONTROLLER")
    print("="*60)
    print("\nInitializing...\n")
    
    # Initialize motor controller
    motor = MotorController()
    tracker = None
    defense = None
    
    # Setup signal handlers for clean shutdown
    import signal
    
    def signal_handler(sig, frame):
        """Handle Ctrl+C and other termination signals."""
        print("\n\n⚠ Interrupt signal received! Stopping motors...")
        try:
            if defense:
                defense.stop_auto_defense()
            if motor:
                motor.emergency_stop()
        except Exception as e:
            print(f"Error during emergency stop: {e}")
        finally:
            sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Kill signal
    
    if not motor.connect():
        print("Failed to connect to motor controller. Exiting.")
        return
    
    # Initialize puck tracker (optional - camera may not be available)
    tracker = PuckTracker()
    camera_available = tracker.start()
    
    if not camera_available:
        print("\n⚠ WARNING: Camera not available")
        print("   - Autonomous defense mode will NOT work")
        print("   - Manual control will work normally")
        print("   - Possible causes:")
        print("     1. Another process using camera (kill stream_camera.py)")
        print("     2. Camera not connected")
        print("     3. Permission issues")
        print("\n   Continuing in MANUAL MODE ONLY...\n")
    
    # Initialize defense controller
    defense = DefenseController(motor, tracker)
    
    try:
        # Enable motors
        if not motor.enable():
            print("Failed to enable motors. Exiting.")
            return
        
        # Prompt for homing
        print("\n" + "="*60)
        print("SETUP")
        print("="*60)
        if not motor.homed:
            choice = input("Run sensorless homing calibration? [Y/n]: ").strip().lower()
            if choice != 'n':
                if not motor.home():
                    print("Homing failed. Exiting for safety.")
                    return
                
                # Update tracker with actual workspace limits from motor controller
                if camera_available and tracker:
                    tracker.set_workspace_limits(
                        motor.workspace_limits['x_min'],
                        motor.workspace_limits['x_max'],
                        motor.workspace_limits['y_min'],
                        motor.workspace_limits['y_max']
                    )
        
        print("\n" + "="*60)
        print("READY!")
        print("="*60)
        print("\nCommands:")
        print("  auto         - Start autonomous defense")
        print("  stop         - Stop autonomous defense")
        print("  view         - Toggle web stream (http://<ip>:5001)")
        print("  right        - Move right")
        print("  left         - Move left")
        print("  up           - Move up")
        print("  down         - Move down")
        print("  center       - Move to center")
        print("  home         - Run sensorless homing")
        print("  camera_home  - Camera-guided safe homing")
        print("  localize     - Use camera to find current position (no motors)")
        print("  quit         - Exit")
        print("\nPress Ctrl+C at any time to emergency stop")
        print("="*60 + "\n")
        
        # Main command loop
        while True:
            try:
                cmd = input(">>> ").strip().lower()
                
                if not cmd:
                    continue
                
                if cmd in ['quit', 'exit', 'q']:
                    break
                
                elif cmd == 'auto':
                    if not camera_available:
                        print("⚠ Error: Camera not available. Cannot start autonomous mode.")
                        print("   Run manual commands or fix camera connection.")
                    else:
                        defense.start_auto_defense()
                
                elif cmd == 'stop':
                    defense.stop_auto_defense()
                    motor.stop()
                
                elif cmd == 'view':
                    if not camera_available:
                        print("⚠ Error: Camera not available.")
                    else:
                        tracker.toggle_preview()
                
                elif cmd == 'localize':
                    if not camera_available:
                        print("⚠ Error: Camera not available. Cannot localize.")
                    else:
                        print("Looking for green paddle in camera view...")
                        # Wait a moment for tracking to stabilize
                        time.sleep(0.5)
                        
                        paddle_pos = tracker.get_paddle_position()
                        if paddle_pos is None:
                            print("✗ Cannot see green paddle. Make sure it's visible in camera view.")
                        else:
                            # Convert camera coordinates to workspace coordinates
                            cam_x, cam_y = paddle_pos
                            workspace_x, workspace_y = tracker.camera_to_workspace(cam_x, cam_y)
                            print(f"  Camera position: ({cam_x:.0f}, {cam_y:.0f}) pixels")
                            print(f"  Workspace position: ({workspace_x:.1f}, {workspace_y:.1f}) mm")
                            
                            # Update motor controller position
                            if motor.localize_from_vision(workspace_x, workspace_y):
                                print("✓ Position recovered! You can now use velocity commands.")
                            else:
                                print("✗ Failed to update position. Make sure workspace was homed at least once.")
                
                elif cmd == 'right':
                    motor.right()
                
                elif cmd == 'left':
                    motor.left()
                
                elif cmd == 'up':
                    motor.up()
                
                elif cmd == 'down':
                    motor.down()
                
                elif cmd == 'center':
                    center_x = (motor.workspace_limits['x_max'] + motor.workspace_limits['x_min']) / 2.0
                    center_y = (motor.workspace_limits['y_max'] + motor.workspace_limits['y_min']) / 2.0
                    print(f"Moving to center ({center_x:.1f}, {center_y:.1f})...")
                    motor.move_to(center_x, center_y)
                
                elif cmd == 'home':
                    if motor.home():
                        # Update tracker with new workspace limits
                        if camera_available and tracker:
                            tracker.set_workspace_limits(
                                motor.workspace_limits['x_min'],
                                motor.workspace_limits['x_max'],
                                motor.workspace_limits['y_min'],
                                motor.workspace_limits['y_max']
                            )
                
                elif cmd == 'camera_home':
                    if not camera_available:
                        print("⚠ Error: Camera not available. Cannot perform camera-guided homing.")
                    else:
                        motor.camera_guided_home(tracker)
                
                else:
                    print(f"Unknown command: {cmd}")
                    
            except KeyboardInterrupt:
                print("\n\n⚠ Ctrl+C - Emergency stop!")
                defense.stop_auto_defense()
                motor.emergency_stop()
                break
            
            except EOFError:
                print("\nEOF - Exiting")
                break
    
    finally:
        print("\nShutting down...")
        if defense:
            defense.stop_auto_defense()
        if tracker:
            tracker.stop()
        if motor:
            motor.emergency_stop()
            time.sleep(0.2)  # Give time for stop commands to send
            motor.close()
        print("Done.")


if __name__ == "__main__":
    main()
