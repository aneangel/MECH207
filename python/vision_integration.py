#!/usr/bin/env python3
"""
Vision Integration Module
Bridges hockey_vision.py with the main air hockey system.
"""

import time
import threading
import queue
from typing import Dict, Any, Optional, Callable
import cv2
import numpy as np
from collections import deque

# Import the existing vision components
from hockey_vision import PuckTrackerHistory, has_display

class VisionIntegration:
    """
    Integration layer between hockey_vision.py and the main system.
    Provides vision data to the AI and game systems.
    """
    
    def __init__(self, system_callback: Optional[Callable] = None):
        # Vision system components
        self.tracker = PuckTrackerHistory()
        self.camera = None
        self.camera_index = 0
        
        # Vision settings (from hockey_vision.py)
        self.WIDTH = 1280
        self.HEIGHT = 720
        self.FPS = 60
        
        # Color thresholds (from hockey_vision.py)
        self.PUCK_LOWER_RED1 = np.array([0, 100, 100], dtype=np.uint8)
        self.PUCK_UPPER_RED1 = np.array([15, 255, 255], dtype=np.uint8)
        self.PUCK_LOWER_RED2 = np.array([165, 100, 100], dtype=np.uint8)
        self.PUCK_UPPER_RED2 = np.array([180, 255, 255], dtype=np.uint8)
        self.PUCK_MIN_AREA = 300
        
        self.PADDLE_LOWER_HSV = np.array([0, 0, 0], dtype=np.uint8)
        self.PADDLE_UPPER_HSV = np.array([180, 255, 50], dtype=np.uint8)
        self.PADDLE_MIN_AREA = 1500
        
        # Morphological kernels
        self.kernel_puck = np.ones((3, 3), np.uint8)
        self.kernel_paddle = np.ones((5, 5), np.uint8)
        
        # Threading and data sharing
        self.vision_thread = None
        self.running = False
        self.stop_event = threading.Event()
        self.system_callback = system_callback
        
        # Vision data
        self.current_vision_data = {}
        self.data_lock = threading.Lock()
        
        # Performance tracking
        self.frame_count = 0
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.processing_fps = 0.0
        
        # Display settings
        self.display_available = has_display()
        self.show_display = False  # Can be enabled for debugging
        
    def initialize_camera(self) -> bool:
        """Initialize camera for vision processing."""
        print(f"Initializing camera {self.camera_index}...")
        
        try:
            self.camera = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
            
            if not self.camera.isOpened():
                print(f"Failed to open camera {self.camera_index}")
                return False
            
            # Configure camera
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.WIDTH)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.HEIGHT)
            self.camera.set(cv2.CAP_PROP_FPS, self.FPS)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Low latency
            
            # Verify settings
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            print(f"Camera initialized: {actual_width}x{actual_height}")
            return True
            
        except Exception as e:
            print(f"Camera initialization error: {e}")
            return False
    
    def start_vision_processing(self) -> bool:
        """Start vision processing thread."""
        if self.running:
            return True
        
        if not self.camera or not self.camera.isOpened():
            if not self.initialize_camera():
                return False
        
        self.stop_event.clear()
        self.running = True
        self.vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
        self.vision_thread.start()
        
        print("Vision processing started")
        return True
    
    def stop_vision_processing(self):
        """Stop vision processing thread."""
        self.running = False
        self.stop_event.set()
        
        if self.vision_thread and self.vision_thread.is_alive():
            self.vision_thread.join(timeout=2)
        
        if self.camera:
            self.camera.release()
        
        cv2.destroyAllWindows()
        print("Vision processing stopped")
    
    def _vision_loop(self):
        """Main vision processing loop (runs in separate thread)."""
        print("Vision processing loop started")
        
        # Setup display window if enabled
        window_name = "Air Hockey Vision"
        if self.show_display and self.display_available:
            cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        
        frame_process_counter = 0
        start_time = time.time()
        
        try:
            while not self.stop_event.is_set() and self.running:
                ret, frame = self.camera.read()
                current_time = time.time()
                
                if not ret:
                    print("Warning: Failed to read frame from camera")
                    time.sleep(0.001)
                    continue
                
                self.frame_count += 1
                frame_process_counter += 1
                
                # Process the frame
                vision_data = self._process_frame(frame, current_time)
                
                # Update shared data
                with self.data_lock:
                    self.current_vision_data = vision_data
                
                # Send data to main system
                if self.system_callback:
                    try:
                        self.system_callback(vision_data)
                    except Exception as e:
                        print(f"System callback error: {e}")
                
                # Calculate processing FPS
                if (current_time - start_time) >= 1.0:
                    self.processing_fps = frame_process_counter / (current_time - start_time)
                    start_time = current_time
                    frame_process_counter = 0
                
                # Display frame if enabled
                if self.show_display and self.display_available:
                    display_frame = self._create_display_frame(frame, vision_data)
                    cv2.imshow(window_name, display_frame)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.001)
                
        except Exception as e:
            print(f"Vision loop error: {e}")
        
        finally:
            if self.show_display and self.display_available:
                cv2.destroyWindow(window_name)
    
    def _process_frame(self, frame, timestamp) -> Dict[str, Any]:
        """
        Process a single frame and extract vision data.
        
        Args:
            frame: OpenCV frame
            timestamp: Frame timestamp
            
        Returns:
            Dict containing vision analysis results
        """
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect puck (red)
        puck_pos, puck_data = self._detect_puck(hsv, timestamp)
        
        # Detect paddle (black)
        paddle_pos = self._detect_paddle(hsv)
        
        # Calculate velocities and predictions
        velocity = (None, None)
        speed = 0.0
        direction = "STATIONARY"
        predicted_positions = {}
        distance_to_paddle = None
        moving_towards_paddle = False
        intercept_time = None
        intercept_point = None
        
        if puck_pos:
            # Get velocity from tracker
            velocity = self.tracker.get_velocity()
            
            if velocity[0] is not None and velocity[1] is not None:
                speed = np.sqrt(velocity[0]**2 + velocity[1]**2)
                direction = self._get_movement_direction(velocity[0], velocity[1])
                
                # Trajectory predictions
                predicted_positions = {
                    0.25: self._predict_position(0.25),
                    0.5: self._predict_position(0.5), 
                    1.0: self._predict_position(1.0)
                }
                
                # Distance and intercept calculations
                if paddle_pos:
                    distance_to_paddle = self._calculate_distance(puck_pos, paddle_pos)
                    moving_towards_paddle, intercept_time, intercept_point = self._calculate_intercept(
                        puck_pos, velocity, paddle_pos
                    )
        
        # Compile vision data
        vision_data = {
            'timestamp': timestamp,
            'frame_count': self.frame_count,
            'processing_fps': self.processing_fps,
            'puck_pos': puck_pos,
            'paddle_pos': paddle_pos,
            'velocity': velocity,
            'speed': speed,
            'direction': direction,
            'predicted_positions': predicted_positions,
            'distance_to_paddle': distance_to_paddle,
            'moving_towards_paddle': moving_towards_paddle,
            'intercept_time': intercept_time,
            'intercept_point': intercept_point,
            'puck_detected': puck_pos is not None,
            'paddle_detected': paddle_pos is not None,
        }
        
        return vision_data
    
    def _detect_puck(self, hsv, timestamp) -> tuple:
        """Detect red puck in HSV image."""
        # Create masks for both red ranges
        mask1 = cv2.inRange(hsv, self.PUCK_LOWER_RED1, self.PUCK_UPPER_RED1)
        mask2 = cv2.inRange(hsv, self.PUCK_LOWER_RED2, self.PUCK_UPPER_RED2)
        puck_mask = mask1 | mask2
        
        # Clean up mask
        puck_mask = cv2.morphologyEx(puck_mask, cv2.MORPH_OPEN, self.kernel_puck)
        
        # Find contours
        contours, _ = cv2.findContours(puck_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Get largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) >= self.PUCK_MIN_AREA:
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    puck_pos = (cx, cy)
                    
                    # Add to tracker for velocity calculation
                    self.tracker.add_position(cx, cy, timestamp)
                    
                    return puck_pos, largest_contour
        
        return None, None
    
    def _detect_paddle(self, hsv) -> Optional[tuple]:
        """Detect black paddle in HSV image."""
        # Create mask for black objects
        paddle_mask = cv2.inRange(hsv, self.PADDLE_LOWER_HSV, self.PADDLE_UPPER_HSV)
        paddle_mask = cv2.morphologyEx(paddle_mask, cv2.MORPH_OPEN, self.kernel_paddle)
        
        # Find contours
        contours, _ = cv2.findContours(paddle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) >= self.PADDLE_MIN_AREA:
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)
        
        return None
    
    def _predict_position(self, time_ahead: float) -> Optional[tuple]:
        """Predict puck position after given time."""
        current_pos = self.tracker.get_current_pos()
        velocity = self.tracker.get_velocity()
        
        if current_pos and velocity[0] is not None and velocity[1] is not None:
            future_x = current_pos[0] + velocity[0] * time_ahead
            future_y = current_pos[1] + velocity[1] * time_ahead
            return (int(future_x), int(future_y))
        
        return None
    
    def _calculate_distance(self, pos1: tuple, pos2: tuple) -> float:
        """Calculate Euclidean distance between two points."""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def _calculate_intercept(self, puck_pos: tuple, velocity: tuple, paddle_pos: tuple):
        """Calculate if puck is moving towards paddle and intercept details."""
        if velocity[0] is None or velocity[1] is None:
            return False, None, None
        
        # Vector from puck to paddle
        dx = paddle_pos[0] - puck_pos[0]
        dy = paddle_pos[1] - puck_pos[1]
        
        # Dot product with velocity
        dot_product = dx * velocity[0] + dy * velocity[1]
        
        # Check if moving towards paddle
        moving_towards = dot_product > 0
        
        if moving_towards:
            # Calculate intercept time and point
            distance = self._calculate_distance(puck_pos, paddle_pos)
            speed = np.sqrt(velocity[0]**2 + velocity[1]**2)
            
            if speed > 0:
                intercept_time = distance / speed
                intercept_point = self._predict_position(intercept_time)
                return True, intercept_time, intercept_point
        
        return False, None, None
    
    def _get_movement_direction(self, vx: float, vy: float) -> str:
        """Get human-readable movement direction."""
        speed = np.sqrt(vx**2 + vy**2)
        
        if speed < 50:
            return "STATIONARY"
        
        if abs(vx) > abs(vy):
            return "RIGHT" if vx > 0 else "LEFT"
        else:
            return "DOWN" if vy > 0 else "UP"
    
    def _create_display_frame(self, frame, vision_data) -> np.ndarray:
        """Create annotated display frame for debugging."""
        display_frame = frame.copy()
        
        # Draw puck detection
        if vision_data['puck_pos']:
            puck_pos = vision_data['puck_pos']
            cv2.circle(display_frame, puck_pos, 10, (0, 0, 255), -1)  # Red circle
            cv2.putText(display_frame, "PUCK", (puck_pos[0] + 15, puck_pos[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Draw paddle detection
        if vision_data['paddle_pos']:
            paddle_pos = vision_data['paddle_pos']
            cv2.circle(display_frame, paddle_pos, 15, (255, 0, 255), -1)  # Magenta circle
            cv2.putText(display_frame, "PADDLE", (paddle_pos[0] + 20, paddle_pos[1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        # Draw velocity vector
        if vision_data['puck_pos'] and vision_data['velocity'][0] is not None:
            puck_pos = vision_data['puck_pos']
            vx, vy = vision_data['velocity']
            
            # Scale velocity for display
            end_x = int(puck_pos[0] + vx * 0.1)
            end_y = int(puck_pos[1] + vy * 0.1)
            
            cv2.arrowedLine(display_frame, puck_pos, (end_x, end_y), (0, 255, 0), 3)
        
        # Draw predictions
        for time_ahead, pred_pos in vision_data['predicted_positions'].items():
            if pred_pos:
                cv2.circle(display_frame, pred_pos, 5, (0, 255, 255), 2)
                cv2.putText(display_frame, f"{time_ahead}s", 
                           (pred_pos[0] + 8, pred_pos[1]),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        
        # Add status text
        cv2.putText(display_frame, f"FPS: {vision_data['processing_fps']:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_frame, f"Speed: {vision_data['speed']:.1f} px/s", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_frame, f"Direction: {vision_data['direction']}", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return display_frame
    
    # === PUBLIC API METHODS ===
    
    def get_latest_vision_data(self) -> Dict[str, Any]:
        """Get the latest vision data (thread-safe)."""
        with self.data_lock:
            return self.current_vision_data.copy()
    
    def is_running(self) -> bool:
        """Check if vision processing is running."""
        return self.running
    
    def get_processing_fps(self) -> float:
        """Get current processing FPS."""
        return self.processing_fps
    
    def enable_display(self, enable: bool = True):
        """Enable/disable visual display for debugging."""
        self.show_display = enable and self.display_available
    
    def set_camera_index(self, index: int):
        """Set camera index."""
        self.camera_index = index
    
    def get_status(self) -> Dict[str, Any]:
        """Get comprehensive vision system status."""
        return {
            'running': self.running,
            'camera_initialized': self.camera is not None,
            'display_available': self.display_available,
            'show_display': self.show_display,
            'frame_count': self.frame_count,
            'processing_fps': self.processing_fps,
            'camera_index': self.camera_index,
            'resolution': f"{self.WIDTH}x{self.HEIGHT}",
            'target_fps': self.FPS
        }
