#!/usr/bin/env python3
"""
Air Hockey Vision and Tracking Script for NVIDIA Jetson Nano/Orin.
Detects a RED puck and a BLACK end-effector (paddle) with high-speed tracking.

This script manages the vision pipeline and object history required for trajectory prediction.
*** Designed for continuous, real-time video stream processing (NOT still images). ***

Controls:
- 's' key: Save current frame
- 'q' key or ESC: Quit
- 'r' key: Toggle auto-recording mode
- Space bar: Pause/resume video
"""

import cv2
import time
import os
import sys
import numpy as np
from collections import deque

# --- 1. VISION CONFIGURATION (TUNE THESE VALUES) ---

# Camera Index (Based on your USB connection)
CAMERA_INDEX = 0  

# Default Camera Settings
WIDTH = 1280
HEIGHT = 720
FPS = 60 # Requesting 30 FPS, actual FPS depends on camera and USB throughput

# File/Debug Settings
SAVE_DIR = "vision_captures"
AUTO_SAVE_INTERVAL = 5.0  

# Trajectory History Size (How many recent puck positions to store for velocity calculation)
HISTORY_MAX_LENGTH = 5 

# --- COLOR THRESHOLDS (HSV) ---
# Tuned for red puck and black paddle.

# 1. PUCK (Small Red Circle) - Tuned for the captured image
PUCK_LOWER_RED1 = np.array([0, 100, 100], dtype=np.uint8)
PUCK_UPPER_RED1 = np.array([15, 255, 255], dtype=np.uint8)
PUCK_LOWER_RED2 = np.array([165, 100, 100], dtype=np.uint8)
PUCK_UPPER_RED2 = np.array([180, 255, 255], dtype=np.uint8)
PUCK_MIN_AREA = 300  # Minimum pixel area for the puck

# 2. PADDLE (Larger Black Circle)
PADDLE_LOWER_HSV = np.array([0, 0, 0], dtype=np.uint8)
PADDLE_UPPER_HSV = np.array([180, 255, 50], dtype=np.uint8) 
PADDLE_MIN_AREA = 1500 # Minimum pixel area for the paddle

# --- 2. VISION UTILITY CLASSES ---

class PuckTrackerHistory:
    """Manages the history of puck positions (x, y, timestamp) using a deque."""
    def __init__(self, max_len=HISTORY_MAX_LENGTH):
        # deque ensures fast append and removal from the left side
        self.history = deque(maxlen=max_len)

    def add_position(self, x, y, timestamp):
        """Adds a new position and timestamp."""
        # Ensure data types are native floats for speed in calculations
        self.history.append({'x': float(x), 'y': float(y), 't': float(timestamp)})

    def get_velocity(self):
        """Calculates current puck velocity (vx, vy) based on the last two points."""
        if len(self.history) < 2:
            return None, None
        
        # Get the most recent two points
        P2 = self.history[-1]
        P1 = self.history[-2]
        
        dt = P2['t'] - P1['t']
        
        if dt > 0.0001: # Avoid division by zero and near-zero timings
            vx = (P2['x'] - P1['x']) / dt
            vy = (P2['y'] - P1['y']) / dt
            return vx, vy
        
        return 0.0, 0.0 # Return zero velocity if time delta is too small
    
    def get_current_pos(self):
        """Returns the most recent position (x, y) or None."""
        if self.history:
            P_curr = self.history[-1]
            return (int(P_curr['x']), int(P_curr['y']))
        return None

def has_display():
    """Check if display is available (essential for headless operations)."""
    display = os.environ.get('DISPLAY')
    wayland = os.environ.get('WAYLAND_DISPLAY')
    return bool(display and display.strip()) or bool(wayland and wayland.strip())

def calculate_distance(pos1, pos2):
    """Calculate Euclidean distance between two points."""
    if pos1 is None or pos2 is None:
        return None
    return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

def predict_trajectory(tracker, steps_ahead=30):
    """Predict where puck will be in 'steps_ahead' frames based on current velocity."""
    if len(tracker.history) < 2:
        return None
    
    current_pos = tracker.get_current_pos()
    vx, vy = tracker.get_velocity()
    
    if current_pos is None or vx is None or vy is None:
        return None
    
    # Assume ~60 FPS, so each step is ~1/60 second
    time_step = 1.0 / 60.0
    future_time = steps_ahead * time_step
    
    future_x = current_pos[0] + vx * future_time
    future_y = current_pos[1] + vy * future_time
    
    return (int(future_x), int(future_y))

def get_trajectory_path(tracker, num_points=5):
    """Get the recent path points for trajectory visualization."""
    if len(tracker.history) < 2:
        return []
    
    path_points = []
    for point in list(tracker.history)[-num_points:]:
        path_points.append((int(point['x']), int(point['y'])))
    
    return path_points

def analyze_movement_pattern(tracker):
    """Analyze the movement pattern over the trajectory history."""
    if len(tracker.history) < 3:
        return "INSUFFICIENT_DATA"
    
    # Get velocities for the last few points
    recent_velocities = []
    history_list = list(tracker.history)
    
    for i in range(len(history_list) - 2, max(0, len(history_list) - 5), -1):
        p1, p2 = history_list[i], history_list[i + 1]
        dt = p2['t'] - p1['t']
        if dt > 0:
            vx = (p2['x'] - p1['x']) / dt
            vy = (p2['y'] - p1['y']) / dt
            speed = np.sqrt(vx**2 + vy**2)
            recent_velocities.append(speed)
    
    if not recent_velocities:
        return "NO_VELOCITY_DATA"
    
    avg_speed = np.mean(recent_velocities)
    speed_variance = np.var(recent_velocities) if len(recent_velocities) > 1 else 0
    
    if avg_speed < 30:
        return "SLOW_MOVEMENT"
    elif avg_speed < 100:
        if speed_variance < 500:
            return "STEADY_MOVEMENT" 
        else:
            return "ERRATIC_MOVEMENT"
    elif avg_speed < 300:
        if speed_variance < 1000:
            return "FAST_STEADY"
        else:
            return "FAST_ERRATIC"
    else:
        return "VERY_FAST_MOVEMENT"

def get_movement_direction(vx, vy):
    """Get human-readable movement direction based on velocity."""
    if vx is None or vy is None:
        return "STATIONARY"
    
    speed = np.sqrt(vx**2 + vy**2)
    if speed < 50:  # Pixels per second threshold for "stationary"
        return "STATIONARY"
    
    # Determine primary direction
    if abs(vx) > abs(vy):
        return "RIGHT" if vx > 0 else "LEFT"
    else:
        return "DOWN" if vy > 0 else "UP"

# --- 3. MAIN EXECUTION ---

def main():
    # Initialize the history tracker
    tracker = PuckTrackerHistory()
    
    # State variables
    frame_count = 0
    saved_count = 0
    auto_record = False
    paused = False
    last_auto_save = time.time()
    puck_pos = None
    paddle_pos = None
    
    # Timing variables for calculating the actual processing FPS
    start_time = time.time()
    frame_process_counter = 0
    
    # Headless mode terminal output variables
    last_terminal_update = time.time()
    terminal_update_interval = 0.5  # Update terminal every 0.5 seconds in headless mode
    last_puck_pos = None
    last_paddle_pos = None
    puck_lost_count = 0
    paddle_lost_count = 0

    print("Air Hockey Vision - Initialization")
    print("=" * 50)
    
    # Check display availability
    display_available = has_display()
    print(f"Display: {'Available' if display_available else 'Headless Mode (Use ssh -Y)'}")
    
    # Create save directory
    if not os.path.exists(SAVE_DIR):
        os.makedirs(SAVE_DIR)
        print(f"Created directory: {SAVE_DIR}")
    
    # Initialize camera
    print(f"Connecting to camera {CAMERA_INDEX}...")
    # NOTE: Adding cv2.CAP_V4L2 can sometimes explicitly help with faster throughput
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2) 
    
    if not cap.isOpened():
        print(f"ERROR: Could not open camera {CAMERA_INDEX}")
        print("Try changing CAMERA_INDEX (0, 1, 2, etc.) or check camera permissions.")
        return 1
    
    # Set camera properties
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    
    # *** LOW LATENCY OPTIMIZATION ***
    # This flag forces the V4L2 backend to minimize internal buffering, reducing latency.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) 
    
    # Get actual settings
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps_set = cap.get(cv2.CAP_PROP_FPS) # FPS requested from camera
    
    print(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps_set:.1f} FPS set")
    
    # Setup window if display available
    window_name = "Air Hockey Vision - Live Tracking"
    if display_available:
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE) 
        print("Live video window opened (if display is forwarded)")
    
    print("\nControls:")
    if display_available:
        print("- Press 's' in video window to save frame")
        print("- Press 'r' to toggle auto-recording")
        print("- Press SPACE to pause/resume")
        print("- Press 'q' or ESC to quit")
        print("- Real-time terminal output shows detection & trajectory data")
    else:
        print("- Ctrl+C to quit")
        print("- Auto-saving frames every 5 seconds")
    
    print("- Comprehensive terminal output every 0.5s:")
    print("  * Object detection status and positions")
    print("  * Velocity, speed, and movement patterns")
    print("  * Trajectory predictions and path analysis")
    print("  * Collision detection and intercept calculations")
    
    print("=" * 50)
    
    # Define the morphological kernel (used for cleanup)
    kernel_paddle = np.ones((5, 5), np.uint8)
    kernel_puck = np.ones((3, 3), np.uint8)

    try:
        while True:
            if not paused:
                # Retrieve the latest frame (low-latency reading)
                ret, frame = cap.read()
                current_time = time.time() 
                
                if not ret:
                    print("Warning: Failed to read frame from camera")
                    time.sleep(0.001)
                    continue
                
                frame_count += 1
                frame_process_counter += 1
                
                # --- CORE VISION PROCESSING ---
                # No frame.copy() here for display efficiency! We will use the original frame
                # for calculation and drawing directly onto a copy only if display is active.
                
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                # 1. Detect Puck (Red)
                mask1 = cv2.inRange(hsv, PUCK_LOWER_RED1, PUCK_UPPER_RED1)
                mask2 = cv2.inRange(hsv, PUCK_LOWER_RED2, PUCK_UPPER_RED2)
                puck_mask = mask1 | mask2
                puck_mask = cv2.morphologyEx(puck_mask, cv2.MORPH_OPEN, kernel_puck)
                
                contours, _ = cv2.findContours(puck_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                puck_pos = None
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    if cv2.contourArea(c) >= PUCK_MIN_AREA:
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            puck_center_x = int(M["m10"] / M["m00"])
                            puck_center_y = int(M["m01"] / M["m00"])
                            puck_pos = (puck_center_x, puck_center_y)
                            
                            # Add position to history tracker
                            tracker.add_position(puck_center_x, puck_center_y, current_time)

                # 2. Detect Paddle (Black)
                paddle_mask = cv2.inRange(hsv, PADDLE_LOWER_HSV, PADDLE_UPPER_HSV)
                paddle_mask = cv2.morphologyEx(paddle_mask, cv2.MORPH_OPEN, kernel_paddle)
                
                contours_paddle, _ = cv2.findContours(paddle_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                paddle_pos = None
                if contours_paddle:
                    c = max(contours_paddle, key=cv2.contourArea)
                    if cv2.contourArea(c) >= PADDLE_MIN_AREA:
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            paddle_center_x = int(M["m10"] / M["m00"])
                            paddle_center_y = int(M["m01"] / M["m00"])
                            paddle_pos = (paddle_center_x, paddle_center_y)

                # Calculate Processing FPS
                if (current_time - start_time) >= 1.0:
                    processing_fps = frame_process_counter / (current_time - start_time)
                    start_time = current_time
                    frame_process_counter = 0
                else:
                    processing_fps = frame_process_counter / (current_time - start_time) if (current_time - start_time) > 0 else actual_fps_set


                # --- DISPLAY (Only perform expensive copy and drawing if display is active) ---
                if display_available:
                    # Create copy for display purposes (drawing overlays)
                    main.display_frame = frame.copy() 
                    
                    # Redraw detection overlays for display
                    if puck_pos:
                        ((x, y), radius) = cv2.minEnclosingCircle(c)
                        cv2.circle(main.display_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                        cv2.circle(main.display_frame, puck_pos, 5, (0, 0, 255), -1)

                    if paddle_pos:
                        ((x, y), radius) = cv2.minEnclosingCircle(c) # Re-calculate radius based on current paddle contour 'c'
                        cv2.circle(main.display_frame, (int(x), int(y)), int(radius), (255, 255, 0), 2)
                        cv2.circle(main.display_frame, paddle_pos, 8, (255, 0, 255), -1)

                    # --- TRACKING / DEBUG INFO ---
                    vx, vy = tracker.get_velocity()
                    
                    status_text = f"Proc. FPS: {processing_fps:.1f} | Frames: {frame_count}"
                    
                    puck_status = f"Puck: {puck_pos}" if puck_pos else "Puck: NOT FOUND"
                    paddle_status = f"Paddle: {paddle_pos}" if paddle_pos else "Paddle: NOT FOUND"
                    
                    cv2.putText(main.display_frame, status_text, (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(main.display_frame, puck_status, (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(main.display_frame, paddle_status, (10, 90), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

                    if vx is not None:
                         cv2.putText(main.display_frame, f"Vx: {vx:.2f} px/s, Vy: {vy:.2f} px/s", (10, 120), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                         cv2.putText(main.display_frame, f"History: {len(tracker.history)}/{HISTORY_MAX_LENGTH}", (10, 150),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


                    cv2.putText(main.display_frame, "s=save, r=auto-record, space=pause, q=quit", 
                               (10, main.display_frame.shape[0] - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    cv2.imshow(window_name, main.display_frame)
            
            # --- TERMINAL OUTPUT (Both Display and Headless Modes) ---
            if current_time - last_terminal_update >= terminal_update_interval:
                # Comprehensive terminal output for headless operation
                vx, vy = tracker.get_velocity()
                speed = np.sqrt(vx**2 + vy**2) if vx is not None and vy is not None else 0
                direction = get_movement_direction(vx, vy)
                
                # Track detection changes
                puck_status_changed = (puck_pos is None) != (last_puck_pos is None)
                paddle_status_changed = (paddle_pos is None) != (last_paddle_pos is None)
                
                # Count consecutive lost detections
                if puck_pos is None:
                    puck_lost_count += 1
                else:
                    puck_lost_count = 0
                    
                if paddle_pos is None:
                    paddle_lost_count += 1
                else:
                    paddle_lost_count = 0
                
                # Clear screen occasionally for cleaner output
                if frame_count % 120 == 0:  # Every ~2 seconds at 60 FPS
                    os.system('clear' if os.name == 'posix' else 'cls')
                
                print(f"\n{'='*80}")
                print(f"AIR HOCKEY VISION - FRAME {frame_count} | Processing FPS: {processing_fps:.1f}")
                print(f"{'='*80}")
                
                # === PUCK DETECTION STATUS ===
                if puck_pos:
                    print(f"PUCK DETECTED: Position ({puck_pos[0]}, {puck_pos[1]})")
                    
                    if vx is not None and vy is not None:
                        print(f"  Velocity: {vx:.1f} px/s horizontal, {vy:.1f} px/s vertical")
                        print(f"  Speed: {speed:.1f} px/s | Direction: {direction}")
                        print(f"  History Points: {len(tracker.history)}/{HISTORY_MAX_LENGTH}")
                        
                        # Movement pattern analysis
                        movement_pattern = analyze_movement_pattern(tracker)
                        print(f"  Movement Pattern: {movement_pattern}")
                        
                        # Trajectory path
                        path_points = get_trajectory_path(tracker)
                        if len(path_points) >= 2:
                            print(f"  Recent Path: {' -> '.join([f'({p[0]},{p[1]})' for p in path_points])}")
                        
                        # Multiple trajectory predictions
                        future_pos_short = predict_trajectory(tracker, 15)  # 0.25s
                        future_pos_medium = predict_trajectory(tracker, 30) # 0.5s
                        future_pos_long = predict_trajectory(tracker, 60)   # 1.0s
                        
                        if future_pos_short:
                            print(f"  Predicted (0.25s): ({future_pos_short[0]}, {future_pos_short[1]})")
                        if future_pos_medium:
                            print(f"  Predicted (0.50s): ({future_pos_medium[0]}, {future_pos_medium[1]})")
                        if future_pos_long:
                            print(f"  Predicted (1.00s): ({future_pos_long[0]}, {future_pos_long[1]})")
                            
                    else:
                        print(f"  Velocity: CALCULATING... (need more position data)")
                    
                    # Distance to paddle
                    if paddle_pos:
                        distance = calculate_distance(puck_pos, paddle_pos)
                        print(f"  Distance to Paddle: {distance:.1f} pixels")
                        
                        # Relative position to paddle
                        dx = puck_pos[0] - paddle_pos[0]
                        dy = puck_pos[1] - paddle_pos[1]
                        if abs(dx) > abs(dy):
                            relative_pos = "RIGHT of paddle" if dx > 0 else "LEFT of paddle"
                        else:
                            relative_pos = "BELOW paddle" if dy > 0 else "ABOVE paddle"
                        print(f"  Relative Position: {relative_pos}")
                else:
                    print(f"PUCK NOT DETECTED (lost for {puck_lost_count} updates)")
                    if puck_lost_count > 5:
                        print("  WARNING: Puck lost for extended period - check lighting/occlusion")
                    
                    # Show last known trajectory if recently lost
                    if last_puck_pos and puck_lost_count < 10:
                        print(f"  Last Known Position: ({last_puck_pos[0]}, {last_puck_pos[1]})")
                
                # === PADDLE DETECTION STATUS ===
                if paddle_pos:
                    print(f"PADDLE DETECTED: Position ({paddle_pos[0]}, {paddle_pos[1]})")
                else:
                    print(f"PADDLE NOT DETECTED (lost for {paddle_lost_count} updates)")
                    if paddle_lost_count > 5:
                        print("  WARNING: Paddle lost - check if it's in frame")
                
                # === DETECTION CHANGES ===
                if puck_status_changed:
                    if puck_pos:
                        print("  >>> PUCK REACQUIRED <<<")
                    else:
                        print("  >>> PUCK LOST <<<")
                
                if paddle_status_changed:
                    if paddle_pos:
                        print("  >>> PADDLE REACQUIRED <<<")
                    else:
                        print("  >>> PADDLE LOST <<<")
                
                # === GAME ANALYSIS & TRAJECTORY INTELLIGENCE ===
                if puck_pos and paddle_pos and vx is not None and vy is not None:
                    distance = calculate_distance(puck_pos, paddle_pos)
                    
                    print(f"\nTRAJECTORY ANALYSIS:")
                    
                    # Speed classification
                    if speed > 300:
                        print(f"  SPEED CLASS: VERY HIGH ({speed:.1f} px/s)")
                    elif speed > 200:
                        print(f"  SPEED CLASS: HIGH ({speed:.1f} px/s)")
                    elif speed > 100:
                        print(f"  SPEED CLASS: MODERATE ({speed:.1f} px/s)")
                    elif speed > 50:
                        print(f"  SPEED CLASS: LOW ({speed:.1f} px/s)")
                    else:
                        print(f"  SPEED CLASS: MINIMAL ({speed:.1f} px/s)")
                    
                    # Proximity analysis
                    if distance < 50:
                        print(f"  PROXIMITY: VERY CLOSE ({distance:.1f} px) - CONTACT IMMINENT")
                    elif distance < 100:
                        print(f"  PROXIMITY: CLOSE ({distance:.1f} px) - NEAR PADDLE")
                    elif distance < 200:
                        print(f"  PROXIMITY: MODERATE ({distance:.1f} px)")
                    else:
                        print(f"  PROXIMITY: FAR ({distance:.1f} px)")
                    
                    # Direction analysis towards/away from paddle
                    if distance > 0 and speed > 20:
                        # Calculate if velocity vector points towards paddle
                        dx = paddle_pos[0] - puck_pos[0]
                        dy = paddle_pos[1] - puck_pos[1]
                        
                        # Normalize direction vector to paddle
                        paddle_distance = np.sqrt(dx*dx + dy*dy)
                        if paddle_distance > 0:
                            dx_norm = dx / paddle_distance
                            dy_norm = dy / paddle_distance
                            
                            # Dot product with velocity to see alignment
                            dot_product = dx_norm * vx + dy_norm * vy
                            
                            if dot_product > 50:  # Moving towards paddle
                                # Calculate intercept time
                                time_to_paddle = distance / speed if speed > 0 else float('inf')
                                print(f"  TRAJECTORY: Moving TOWARDS paddle (alignment: {dot_product:.1f})")
                                print(f"  INTERCEPT TIME: {time_to_paddle:.2f} seconds")
                                
                                # Predict exact intercept point
                                intercept_time_frames = int(time_to_paddle * 60)  # Convert to frames
                                intercept_pos = predict_trajectory(tracker, intercept_time_frames)
                                if intercept_pos:
                                    print(f"  INTERCEPT POINT: ({intercept_pos[0]}, {intercept_pos[1]})")
                                    
                            elif dot_product < -50:  # Moving away from paddle
                                print(f"  TRAJECTORY: Moving AWAY from paddle (divergence: {abs(dot_product):.1f})")
                            else:
                                print(f"  TRAJECTORY: Moving PARALLEL to paddle (perpendicular motion)")
                    
                    # Collision prediction
                    future_positions = [
                        predict_trajectory(tracker, 15),
                        predict_trajectory(tracker, 30), 
                        predict_trajectory(tracker, 45),
                        predict_trajectory(tracker, 60)
                    ]
                    
                    collision_times = []
                    for i, future_pos in enumerate(future_positions):
                        if future_pos:
                            future_distance = calculate_distance(future_pos, paddle_pos)
                            if future_distance and future_distance < 75:  # Collision threshold
                                time_prediction = (i + 1) * 0.25  # 0.25s intervals
                                collision_times.append(time_prediction)
                    
                    if collision_times:
                        print(f"  COLLISION ALERT: Potential contact in {collision_times[0]:.2f}s")
                    else:
                        print(f"  COLLISION STATUS: No immediate contact predicted")
                
                # === SYSTEM STATUS ===
                print(f"\nSYSTEM STATUS:")
                print(f"  Camera: {actual_width}x{actual_height} @ {processing_fps:.1f} FPS")
                print(f"  Mode: {'Display + Terminal' if display_available else 'Headless (Terminal Only)'}")
                print(f"  Auto-save: {'ON' if auto_record else 'OFF'}")
                print(f"  Frame Count: {frame_count}")
                
                # Performance metrics
                if processing_fps < 30:
                    print(f"  PERFORMANCE: LOW FPS - Consider reducing resolution")
                elif processing_fps < 45:
                    print(f"  PERFORMANCE: MODERATE FPS - Good for tracking")
                else:
                    print(f"  PERFORMANCE: HIGH FPS - Excellent for fast tracking")
                
                # Auto-save in headless mode
                if (not display_available or auto_record) and (current_time - last_auto_save >= AUTO_SAVE_INTERVAL):
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"{SAVE_DIR}/headless_{timestamp}.jpg"
                    cv2.imwrite(filename, frame)
                    saved_count += 1
                    print(f"\nAUTO-SAVED: {filename}")
                    last_auto_save = current_time
                
                # Update tracking variables
                last_terminal_update = current_time
                last_puck_pos = puck_pos
                last_paddle_pos = paddle_pos
            
            # --- KEYBOARD INPUT ---
            if display_available:
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q') or key == 27:  # 'q' or ESC
                    break
                elif key == ord('s'):
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    filename = f"{SAVE_DIR}/vision_manual_{timestamp}.jpg"
                    # Note: Saves the frame *with* the detection overlays
                    cv2.imwrite(filename, main.display_frame) 
                    saved_count += 1
                    print(f"Manual save: {filename}")
                elif key == ord('r'):
                    auto_record = not auto_record
                    status = "ON" if auto_record else "OFF"
                    print(f"Auto-recording: {status}")
                    if auto_record:
                        last_auto_save = time.time()
                elif key == ord(' '):
                    paused = not paused
                    status = "PAUSED" if paused else "RESUMED"
                    print(f"Video {status}")
            else:
                # Headless mode - small delay for rate limiting I/O 
                time.sleep(0.001) 
    
    except KeyboardInterrupt:
        print(f"\nStopping... Captured {saved_count} images total")
    
    finally:
        # Cleanup
        cap.release()
        if display_available:
            cv2.destroyAllWindows()
        
        print(f"Session complete.")
        print(f"- Total frames processed: {frame_count}")
        print(f"- Images saved: {saved_count}")
        print(f"- Images saved to: {SAVE_DIR}/")
        
    return 0

if __name__ == "__main__":
    sys.exit(main())