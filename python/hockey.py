import cv2
import serial
import time
import numpy as np

class CornerNavigator:
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=115200):
        """Initialize the corner navigator with camera and serial connection."""
        # Serial connection to ESP32
        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for connection to establish
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)  # 0 for default webcam
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")
        
        # Play area dimensions (in mm or your units)
        self.play_width = 1000   # Width of play area
        self.play_height = 600   # Height of play area
        
        # Current position (starts at center)
        self.current_x = self.play_width / 2
        self.current_y = self.play_height / 2
        
        # Corner positions: [top-left, top-right, bottom-right, bottom-left]
        self.corners = [
            (0, 0),                              # Top-left
            (self.play_width, 0),                # Top-right
            (self.play_width, self.play_height), # Bottom-right
            (0, self.play_height)                # Bottom-left
        ]
        
    def send_command(self, command):
        """Send command to ESP32 and get response."""
        self.ser.write(f"{command}\n".encode())
        time.sleep(0.1)
        
        # Read response
        if self.ser.in_waiting:
            response = self.ser.readline().decode().strip()
            print(f"ESP32 Response: {response}")
            return response
        return None
    
    def move_to_position(self, target_x, target_y):
        """Calculate movement needed and send commands to ESP32."""
        # Calculate delta movement
        delta_x = target_x - self.current_x
        delta_y = target_y - self.current_y
        
        print(f"\nMoving from ({self.current_x:.1f}, {self.current_y:.1f}) to ({target_x:.1f}, {target_y:.1f})")
        print(f"Delta: ({delta_x:.1f}, {delta_y:.1f})")
        
        # Send M2 command for horizontal movement (left/right)
        if abs(delta_x) > 0.1:
            # Positive = right, negative = left
            speed = 200  # You can adjust this
            direction_speed = int(speed if delta_x > 0 else -speed)
            print(f"Horizontal movement: {direction_speed} steps/sec")
            self.send_command(f"M2_START_SPEED_{direction_speed}")
            
            # Simulate movement time (you'll need to calculate actual time)
            movement_time = abs(delta_x) / speed
            time.sleep(movement_time)
            
            self.send_command("M2_STOP")
        
        # Send M1 command for vertical movement (up/down)
        if abs(delta_y) > 0.1:
            # Positive = down, negative = up
            speed = 200
            direction_speed = int(speed if delta_y > 0 else -speed)
            print(f"Vertical movement: {direction_speed} steps/sec")
            self.send_command(f"M1_START_SPEED_{direction_speed}")
            
            # Simulate movement time
            movement_time = abs(delta_y) / speed
            time.sleep(movement_time)
            
            self.send_command("M1_STOP")
        
        # Update current position
        self.current_x = target_x
        self.current_y = target_y
    
    def visit_corners(self):
        """Visit all four corners of the play area."""
        corner_names = ["Top-Left", "Top-Right", "Bottom-Right", "Bottom-Left"]
        
        for i, (x, y) in enumerate(self.corners):
            print(f"\n{'='*50}")
            print(f"Moving to {corner_names[i]} corner: ({x}, {y})")
            print(f"{'='*50}")
            
            self.move_to_position(x, y)
            
            # Get current status from ESP32
            self.send_command("M1_GET_STATUS")
            self.send_command("M2_GET_STATUS")
            
            # Show camera feed
            ret, frame = self.cap.read()
            if ret:
                # Draw corner marker
                h, w = frame.shape[:2]
                # Scale corner position to frame size
                frame_x = int((x / self.play_width) * w)
                frame_y = int((y / self.play_height) * h)
                
                cv2.circle(frame, (frame_x, frame_y), 10, (0, 255, 0), -1)
                cv2.putText(frame, corner_names[i], (frame_x + 15, frame_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Draw play area boundary
                cv2.rectangle(frame, (0, 0), (w, h), (255, 0, 0), 2)
                
                cv2.imshow('Camera Feed - Corner Navigation', frame)
                cv2.waitKey(2000)  # Wait 2 seconds at each corner
        
        # Return to center
        print(f"\n{'='*50}")
        print("Returning to center")
        print(f"{'='*50}")
        self.move_to_position(self.play_width / 2, self.play_height / 2)
    
    def cleanup(self):
        """Clean up resources."""
        self.cap.release()
        cv2.destroyAllWindows()
        self.ser.close()

def main():
    """Main function to run the corner navigation test."""
    try:
        # Initialize navigator
        navigator = CornerNavigator(serial_port='/dev/ttyUSB0')
        
        print("Corner Navigation Test")
        print(f"Play Area: {navigator.play_width} x {navigator.play_height}")
        print(f"Starting position: ({navigator.current_x}, {navigator.current_y})")
        print("\nPress Ctrl+C to exit")
        
        # Visit all corners
        navigator.visit_corners()
        
        print("\nTest complete!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        navigator.cleanup()

if __name__ == "__main__":
    main()