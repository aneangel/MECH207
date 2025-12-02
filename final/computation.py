"""
Air Hockey Robot Computation Module

Processes puck detection data from camera, calculates trajectory and velocity,
computes interception point, and sends CoreXY motor commands to position
the paddle/end effector to intercept the puck.

Physics & Kinematics:
- Tracks puck position over time
- Calculates velocity and trajectory
- Predicts future position
- Computes CoreXY motor movements for interception

Usage:
    from computation import TrajectoryCalculator
"""

import numpy as np
import time
from collections import deque
from typing import Optional, Tuple, Dict, List
import threading

class TrajectoryCalculator:
    """
    Calculates puck trajectory and computes motor commands for interception.
    """
    
    def __init__(self, motor_controller=None):
        """
        Initialize trajectory calculator.
        
        Args:
            motor_controller: MotorController instance from sense.py
        """
        self.motor_controller = motor_controller
        
        # ===== CAMERA & PHYSICAL CALIBRATION =====
        # Camera resolution (from camera.py)
        self.camera_width = 1280  # pixels
        self.camera_height = 720  # pixels
        
        # Physical table dimensions (ADJUST TO YOUR TABLE!)
        self.table_width_mm = 35.0   # Width in mm (example: 1.2m table)
        self.table_height_mm = 70.0   # Height in mm (example: 0.8m table)
        
        # Pixels to mm conversion factors
        self.px_to_mm_x = self.table_width_mm / self.camera_width
        self.px_to_mm_y = self.table_height_mm / self.camera_height
        
        # ===== COREXY MECHANICAL PARAMETERS =====
        # Motor steps per revolution (0.9° motor = 400 steps, 1/64 microstepping)
        self.steps_per_rev = 400.0
        self.microsteps = 64.0
        self.total_steps_per_rev = self.steps_per_rev * self.microsteps
        
        # Belt and pulley configuration
        self.pulley_teeth = 20  # GT2 pulley teeth count
        self.belt_pitch_mm = 2.0  # GT2 belt pitch (2mm)
        self.mm_per_rev = self.pulley_teeth * self.belt_pitch_mm  # 40mm per revolution
        
        # Steps per mm for each motor
        self.steps_per_mm = self.total_steps_per_rev / self.mm_per_rev
        
        # Maximum motor speed (RPM) - adjust based on your motors
        self.max_rpm = 100.0
        self.max_velocity_mm_s = (self.max_rpm * self.mm_per_rev) / 60.0
        
        # ===== PUCK TRACKING =====
        # History buffer for velocity calculation
        self.position_history = deque(maxlen=10)  # Store last 10 positions
        self.time_history = deque(maxlen=10)
        
        # Current state
        self.current_position = None  # (x, y) in mm
        self.current_velocity = None  # (vx, vy) in mm/s
        self.last_update_time = None
        
        # ===== ROBOT STATE =====
        # Current end effector position (mm from origin)
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Defense zone (where robot can operate)
        self.defense_zone_y_min = 0.0  # Bottom of table
        self.defense_zone_y_max = self.table_height_mm / 3.0  # Defend bottom third
        
        # Reaction time and prediction
        self.reaction_delay_s = 0.1  # System response delay
        self.prediction_time_s = 0.5  # How far ahead to predict
        
        # Thread safety
        self.lock = threading.Lock()
        
        print("=" * 60)
        print("TRAJECTORY CALCULATOR INITIALIZED")
        print("=" * 60)
        print(f"Table Size: {self.table_width_mm}mm x {self.table_height_mm}mm")
        print(f"Camera: {self.camera_width}x{self.camera_height} pixels")
        print(f"Conversion: {self.px_to_mm_x:.3f} mm/px (X), {self.px_to_mm_y:.3f} mm/px (Y)")
        print(f"CoreXY: {self.steps_per_mm:.2f} steps/mm")
        print(f"Max Velocity: {self.max_velocity_mm_s:.1f} mm/s")
        print(f"Motor Controller: {'Connected' if motor_controller else 'Not Connected'}")
        print("=" * 60)
    
    def pixel_to_mm(self, px_x: int, px_y: int) -> Tuple[float, float]:
        """
        Convert pixel coordinates to physical mm coordinates.
        
        Args:
            px_x: X coordinate in pixels
            px_y: Y coordinate in pixels
            
        Returns:
            (x_mm, y_mm): Position in millimeters
        """
        # Convert with origin at top-left
        x_mm = px_x * self.px_to_mm_x
        y_mm = px_y * self.px_to_mm_y
        
        return x_mm, y_mm
    
    def update_puck_position(self, detection: Dict) -> bool:
        """
        Update puck position from camera detection.
        
        Args:
            detection: Detection dict from camera.py with 'center', 'area', 'bbox'
            
        Returns:
            bool: True if position updated successfully
        """
        with self.lock:
            # Extract pixel coordinates
            px_x, px_y = detection['center']
            
            # Convert to mm
            x_mm, y_mm = self.pixel_to_mm(px_x, px_y)
            
            # Store position and timestamp
            current_time = time.time()
            
            self.position_history.append((x_mm, y_mm))
            self.time_history.append(current_time)
            
            self.current_position = (x_mm, y_mm)
            self.last_update_time = current_time
            
            # Calculate velocity if we have enough history
            if len(self.position_history) >= 2:
                self._calculate_velocity()
            
            return True
    
    def _calculate_velocity(self):
        """Calculate puck velocity from position history."""
        if len(self.position_history) < 2:
            self.current_velocity = (0.0, 0.0)
            return
        
        # Use linear regression for better velocity estimate
        positions = list(self.position_history)
        times = list(self.time_history)
        
        # Calculate average velocity over recent history
        x_positions = [p[0] for p in positions]
        y_positions = [p[1] for p in positions]
        
        # Time differences
        dt = times[-1] - times[0]
        
        if dt < 0.001:  # Avoid division by zero
            self.current_velocity = (0.0, 0.0)
            return
        
        # Velocity = displacement / time
        dx = x_positions[-1] - x_positions[0]
        dy = y_positions[-1] - y_positions[0]
        
        vx = dx / dt  # mm/s
        vy = dy / dt  # mm/s
        
        self.current_velocity = (vx, vy)
    
    def predict_future_position(self, time_ahead_s: float) -> Optional[Tuple[float, float]]:
        """
        Predict future puck position assuming linear motion.
        
        Args:
            time_ahead_s: Time in seconds to predict ahead
            
        Returns:
            (x, y) predicted position in mm, or None if insufficient data
        """
        with self.lock:
            if self.current_position is None or self.current_velocity is None:
                return None
            
            x, y = self.current_position
            vx, vy = self.current_velocity
            
            # Linear prediction: position = current_pos + velocity * time
            future_x = x + vx * time_ahead_s
            future_y = y + vy * time_ahead_s
            
            # Clamp to table bounds
            future_x = max(0, min(self.table_width_mm, future_x))
            future_y = max(0, min(self.table_height_mm, future_y))
            
            return (future_x, future_y)
    
    def calculate_interception_point(self) -> Optional[Tuple[float, float]]:
        """
        Calculate where robot should move to intercept the puck.
        
        Returns:
            (x, y) interception point in mm, or None if no interception needed
        """
        with self.lock:
            if self.current_velocity is None or self.current_position is None:
                return None
            
            vx, vy = self.current_velocity
            
            # Check if puck is moving towards our defense zone
            if vy <= 0:  # Moving away or stationary
                return None
            
            # Predict where puck will be after reaction delay
            prediction_time = self.reaction_delay_s + self.prediction_time_s
            future_pos = self.predict_future_position(prediction_time)
            
            if future_pos is None:
                return None
            
            future_x, future_y = future_pos
            
            # Check if puck will enter our defense zone
            if future_y < self.defense_zone_y_min or future_y > self.defense_zone_y_max:
                return None
            
            # Interception point is at the edge of our defense zone
            intercept_y = self.defense_zone_y_max
            
            # Calculate time for puck to reach intercept line
            if vy > 0.1:  # Avoid division by near-zero
                time_to_intercept = (intercept_y - self.current_position[1]) / vy
            else:
                return None
            
            # Where will puck be horizontally at intercept line?
            intercept_x = self.current_position[0] + vx * time_to_intercept
            
            # Clamp to table width
            intercept_x = max(0, min(self.table_width_mm, intercept_x))
            
            return (intercept_x, intercept_y)
    
    def corexy_inverse_kinematics(self, target_x: float, target_y: float) -> Tuple[float, float]:
        """
        Calculate CoreXY motor rotations to reach target position.
        
        CoreXY kinematics:
        Motor A (right) controls: X + Y
        Motor B (left) controls:  X - Y
        
        Args:
            target_x: Target X position in mm
            target_y: Target Y position in mm
            
        Returns:
            (motor_a_revs, motor_b_revs): Revolutions for each motor from current position
        """
        # Calculate displacement from current position
        dx = target_x - self.robot_x
        dy = target_y - self.robot_y
        
        # CoreXY inverse kinematics
        # Motor A distance = dx + dy
        # Motor B distance = dx - dy
        dist_a = dx + dy
        dist_b = dx - dy
        
        # Convert distance to motor revolutions
        revs_a = dist_a / self.mm_per_rev
        revs_b = dist_b / self.mm_per_rev
        
        return (revs_a, revs_b)
    
    def calculate_motor_rpm(self, revolutions: float, time_available: float) -> float:
        """
        Calculate required motor RPM to complete movement in given time.
        
        Args:
            revolutions: Number of revolutions needed
            time_available: Time in seconds to complete movement
            
        Returns:
            float: Required RPM
        """
        if time_available <= 0.001:
            return self.max_rpm
        
        # RPM = (revolutions / time_in_minutes)
        rpm = abs(revolutions) / (time_available / 60.0)
        
        # Clamp to maximum
        rpm = min(rpm, self.max_rpm)
        
        # Direction is handled by sign of revolutions
        if revolutions < 0:
            rpm = -rpm
        
        return rpm
    
    def move_to_interception(self, intercept_point: Tuple[float, float], 
                            time_to_intercept: float = 0.5) -> bool:
        """
        Command motors to move to interception point.
        
        Args:
            intercept_point: (x, y) target position in mm
            time_to_intercept: Time in seconds to reach position
            
        Returns:
            bool: True if command sent successfully
        """
        if self.motor_controller is None:
            print("No motor controller connected!")
            return False
        
        target_x, target_y = intercept_point
        
        # Calculate motor movements
        revs_a, revs_b = self.corexy_inverse_kinematics(target_x, target_y)
        
        # Calculate required RPM for each motor
        rpm_a = self.calculate_motor_rpm(revs_a, time_to_intercept)
        rpm_b = self.calculate_motor_rpm(revs_b, time_to_intercept)
        
        print(f"\n[INTERCEPTION COMMAND]")
        print(f"Target: ({target_x:.1f}, {target_y:.1f}) mm")
        print(f"Current: ({self.robot_x:.1f}, {self.robot_y:.1f}) mm")
        print(f"Motor A (Right): {revs_a:.3f} revs @ {rpm_a:.1f} RPM")
        print(f"Motor B (Left):  {revs_b:.3f} revs @ {rpm_b:.1f} RPM")
        
        # Send command to motors
        response = self.motor_controller.set_both_rpm(rpm_a, rpm_b)
        
        if response and response.get('status') == 'ok':
            # Update robot position (will be corrected by feedback if available)
            self.robot_x = target_x
            self.robot_y = target_y
            print(f"Command sent successfully!")
            return True
        else:
            print(f"Command failed: {response}")
            return False
    
    def get_motor_commands(self, intercept_point: Tuple[float, float], 
                          time_to_intercept: float = 0.5) -> Optional[Dict]:
        """
        Calculate motor commands without sending them.
        
        Args:
            intercept_point: (x, y) target position in mm
            time_to_intercept: Time in seconds to reach position
            
        Returns:
            Dict with motor commands: {'rpm_right': float, 'rpm_left': float, 'revs_right': float, 'revs_left': float}
        """
        target_x, target_y = intercept_point
        
        # Calculate motor movements
        revs_a, revs_b = self.corexy_inverse_kinematics(target_x, target_y)
        
        # Calculate required RPM for each motor
        rpm_a = self.calculate_motor_rpm(revs_a, time_to_intercept)
        rpm_b = self.calculate_motor_rpm(revs_b, time_to_intercept)
        
        return {
            'rpm_right': rpm_a,  # Motor A (right)
            'rpm_left': rpm_b,   # Motor B (left)
            'revs_right': revs_a,
            'revs_left': revs_b,
            'target_position': (target_x, target_y),
            'time_to_intercept': time_to_intercept
        }
    
    def process_detection(self, detection: Dict) -> Optional[Dict]:
        """
        Main processing function - updates position, calculates trajectory, and sends motor commands.
        
        Args:
            detection: Detection dict from camera.py
            
        Returns:
            Dict with processing results, or None if no action taken
        """
        # Update position tracking
        self.update_puck_position(detection)
        
        # Calculate interception point
        intercept = self.calculate_interception_point()
        
        if intercept is None:
            return None
        
        # Calculate time to intercept
        if self.current_velocity is not None:
            vx, vy = self.current_velocity
            speed = np.sqrt(vx**2 + vy**2)
            
            if speed > 1.0:  # Only if puck is moving significantly
                # Distance to intercept line
                dist_to_intercept = intercept[1] - self.current_position[1]
                time_to_intercept = abs(dist_to_intercept / vy) if vy != 0 else 1.0
                
                # Give robot time to react
                time_available = max(0.2, time_to_intercept - self.reaction_delay_s)
                
                # Move to interception point
                success = self.move_to_interception(intercept, time_available)
                
                return {
                    'puck_position': self.current_position,
                    'puck_velocity': self.current_velocity,
                    'intercept_point': intercept,
                    'time_to_intercept': time_to_intercept,
                    'command_sent': success
                }
        
        return None
    
    def stop_motors(self):
        """Emergency stop - halt all motors."""
        if self.motor_controller:
            print("\n[EMERGENCY STOP] Stopping all motors!")
            self.motor_controller.stop_all()
    
    def get_status(self) -> Dict:
        """Get current system status."""
        with self.lock:
            return {
                'robot_position': (self.robot_x, self.robot_y),
                'puck_position': self.current_position,
                'puck_velocity': self.current_velocity,
                'tracking_points': len(self.position_history),
                'motor_controller_connected': self.motor_controller is not None
            }


def test_trajectory_calculator():
    """Test function for trajectory calculator without camera."""
    print("\n" + "="*60)
    print("TRAJECTORY CALCULATOR TEST MODE")
    print("="*60)
    
    # Create calculator without motor controller for testing
    calc = TrajectoryCalculator(motor_controller=None)
    
    # Simulate puck detections
    test_detections = [
        {'center': (640, 100), 'area': 500, 'bbox': (620, 80, 40, 40)},  # Top center
        {'center': (640, 200), 'area': 500, 'bbox': (620, 180, 40, 40)},  # Moving down
        {'center': (640, 300), 'area': 500, 'bbox': (620, 280, 40, 40)},  # Moving down
        {'center': (640, 400), 'area': 500, 'bbox': (620, 380, 40, 40)},  # Moving down
    ]
    
    for i, detection in enumerate(test_detections):
        print(f"\n--- Detection {i+1} ---")
        calc.update_puck_position(detection)
        
        if calc.current_velocity:
            print(f"Position: {calc.current_position}")
            print(f"Velocity: {calc.current_velocity}")
            
            future = calc.predict_future_position(0.5)
            print(f"Predicted position (0.5s): {future}")
            
            intercept = calc.calculate_interception_point()
            if intercept:
                print(f"Interception point: {intercept}")
                revs_a, revs_b = calc.corexy_inverse_kinematics(intercept[0], intercept[1])
                print(f"Motor movements: A={revs_a:.3f} revs, B={revs_b:.3f} revs")
        
        time.sleep(0.1)
    
    print("\n" + "="*60)
    print("Test complete!")
    print("="*60)


if __name__ == "__main__":
    test_trajectory_calculator()