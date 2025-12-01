#!/usr/bin/env python3
"""
ESP32 Motor Controller Interface
Handles communication between Jetson Orin Nano and ESP32 for motor/actuator control.
"""

import serial
import json
import time
import threading
import queue
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass

@dataclass
class MotorCommand:
    """Represents a motor movement command."""
    motor_id: str  # "paddle_x", "paddle_y"
    position: int  # Target position in steps
    speed: int     # Speed (steps/second)
    acceleration: int = 1000  # Steps/second^2

@dataclass
class SystemStatus:
    """ESP32 system status information."""
    paddle_x_pos: int = 0
    paddle_y_pos: int = 0
    limit_switches: Dict[str, bool] = None
    blower_status: bool = False
    air_pressure: int = 0
    last_update: float = 0.0
    
    def __post_init__(self):
        if self.limit_switches is None:
            self.limit_switches = {
                "x_min": False, "x_max": False,
                "y_min": False, "y_max": False
            }

class ESP32Controller:
    """
    Main interface for ESP32 communication.
    Handles motor commands, sensor readings, and system status.
    """
    
    def __init__(self, port="/dev/ttyACM0", baudrate=115200, timeout=1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.connected = False
        
        # Threading for communication
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()
        self.status_queue = queue.Queue()
        
        self.comm_thread = None
        self.stop_event = threading.Event()
        
        # System state
        self.system_status = SystemStatus()
        self.last_command_id = 0
        
        # Paddle limits (in steps) - adjust based on your table size
        self.PADDLE_X_MIN = 0
        self.PADDLE_X_MAX = 3500  # 35cm table width
        self.PADDLE_Y_MIN = 0  
        self.PADDLE_Y_MAX = 7000  # 70cm table length
        
        # Emergency stop flag
        self.emergency_stop = False
    
    def connect(self) -> bool:
        """Establish connection to ESP32."""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            
            time.sleep(2)  # Wait for ESP32 to initialize
            
            # Send ping command to verify connection
            if self._send_command_sync({"cmd": "ping"}, timeout=3.0):
                self.connected = True
                
                # Start communication thread
                self.stop_event.clear()
                self.comm_thread = threading.Thread(target=self._communication_loop, daemon=True)
                self.comm_thread.start()
                
                print("ESP32 Controller connected successfully")
                return True
            else:
                print("ESP32 Controller: No response to ping")
                return False
                
        except Exception as e:
            print(f"ESP32 Controller connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close connection to ESP32."""
        self.connected = False
        self.stop_event.set()
        
        if self.comm_thread and self.comm_thread.is_alive():
            self.comm_thread.join(timeout=2)
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        
        print("ESP32 Controller disconnected")
    
    def _communication_loop(self):
        """Background thread for handling ESP32 communication."""
        while not self.stop_event.is_set() and self.connected:
            try:
                # Send queued commands
                if not self.command_queue.empty():
                    command = self.command_queue.get_nowait()
                    self._send_command_raw(command)
                
                # Read responses and status updates
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self._process_response(line)
                
                time.sleep(0.001)  # Small delay to prevent excessive CPU usage
                
            except Exception as e:
                print(f"ESP32 Communication error: {e}")
                time.sleep(0.1)
    
    def _send_command_raw(self, command: Dict[str, Any]):
        """Send raw command to ESP32."""
        if not self.connected or not self.serial_conn:
            return False
        
        try:
            command_str = json.dumps(command) + '\n'
            self.serial_conn.write(command_str.encode('utf-8'))
            self.serial_conn.flush()
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False
    
    def _send_command_sync(self, command: Dict[str, Any], timeout: float = 2.0) -> bool:
        """Send command and wait for acknowledgment."""
        if not self._send_command_raw(command):
            return False
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.serial_conn.in_waiting > 0:
                try:
                    response = self.serial_conn.readline().decode('utf-8').strip()
                    if response:
                        data = json.loads(response)
                        if data.get('status') == 'ok':
                            return True
                except:
                    pass
            time.sleep(0.01)
        
        return False
    
    def _process_response(self, line: str):
        """Process incoming response from ESP32."""
        try:
            data = json.loads(line)
            
            # Handle different response types
            if data.get('type') == 'status':
                self._update_system_status(data)
            elif data.get('type') == 'response':
                self.response_queue.put(data)
            elif data.get('type') == 'sensor':
                self._process_sensor_data(data)
            
        except json.JSONDecodeError:
            print(f"Invalid JSON response: {line}")
        except Exception as e:
            print(f"Error processing response: {e}")
    
    def _update_system_status(self, data: Dict[str, Any]):
        """Update system status from ESP32 data."""
        try:
            self.system_status.paddle_x_pos = data.get('paddle_x', 0)
            self.system_status.paddle_y_pos = data.get('paddle_y', 0)
            self.system_status.blower_status = data.get('blower', False)
            self.system_status.air_pressure = data.get('pressure', 0)
            self.system_status.last_update = time.time()
            
            # Update limit switches
            limits = data.get('limits', {})
            self.system_status.limit_switches.update(limits)
            
        except Exception as e:
            print(f"Error updating system status: {e}")
    
    def _process_sensor_data(self, data: Dict[str, Any]):
        """Process sensor data from ESP32."""
        # This will be used for IR sensors, limit switches, etc.
        sensor_type = data.get('sensor')
        
        if sensor_type == 'goal':
            # Goal detection from IR sensor
            goal_side = data.get('side', 'unknown')  # 'player' or 'robot'
            self.status_queue.put(('goal_detected', goal_side))
            
        elif sensor_type == 'coin':
            # Coin detection for game start
            self.status_queue.put(('coin_inserted', True))
            
        elif sensor_type == 'limit':
            # Limit switch activation
            switch = data.get('switch', 'unknown')
            state = data.get('state', False)
            self.status_queue.put(('limit_switch', {'switch': switch, 'state': state}))
    
    # === PUBLIC API METHODS ===
    
    def move_paddle(self, x: int, y: int, speed: int = 2000) -> bool:
        """
        Move paddle to absolute position.
        
        Args:
            x: X position in steps
            y: Y position in steps  
            speed: Movement speed in steps/second
            
        Returns:
            bool: True if command queued successfully
        """
        if self.emergency_stop:
            return False
        
        # Clamp positions to limits
        x = max(self.PADDLE_X_MIN, min(self.PADDLE_X_MAX, x))
        y = max(self.PADDLE_Y_MIN, min(self.PADDLE_Y_MAX, y))
        
        command = {
            "cmd": "move_paddle",
            "x": x,
            "y": y,
            "speed": speed,
            "id": self._get_next_command_id()
        }
        
        if self.connected:
            self.command_queue.put(command)
            return True
        
        return False
    
    def move_paddle_relative(self, dx: int, dy: int, speed: int = 2000) -> bool:
        """Move paddle relative to current position."""
        current_x = self.system_status.paddle_x_pos
        current_y = self.system_status.paddle_y_pos
        
        return self.move_paddle(current_x + dx, current_y + dy, speed)
    
    def set_blower(self, enable: bool) -> bool:
        """Control air blower fans."""
        command = {
            "cmd": "set_blower",
            "enable": enable,
            "id": self._get_next_command_id()
        }
        
        if self.connected:
            self.command_queue.put(command)
            return True
        
        return False
    
    def set_air_pressure(self, pressure: int) -> bool:
        """
        Set air pressure using potentiometer value.
        
        Args:
            pressure: Pressure level (0-100)
        """
        command = {
            "cmd": "set_pressure",
            "pressure": max(0, min(100, pressure)),
            "id": self._get_next_command_id()
        }
        
        if self.connected:
            self.command_queue.put(command)
            return True
        
        return False
    
    def home_paddle(self) -> bool:
        """Home the paddle to known position using limit switches."""
        command = {
            "cmd": "home_paddle",
            "id": self._get_next_command_id()
        }
        
        if self.connected:
            self.command_queue.put(command)
            return True
        
        return False
    
    def emergency_stop_enable(self):
        """Enable emergency stop - halts all motor movement."""
        self.emergency_stop = True
        command = {
            "cmd": "emergency_stop",
            "id": self._get_next_command_id()
        }
        
        if self.connected:
            self.command_queue.put(command)
    
    def emergency_stop_disable(self):
        """Disable emergency stop."""
        self.emergency_stop = False
        command = {
            "cmd": "emergency_clear",
            "id": self._get_next_command_id()
        }
        
        if self.connected:
            self.command_queue.put(command)
    
    def get_status(self) -> SystemStatus:
        """Get current system status."""
        return self.system_status
    
    def get_sensor_data(self) -> Optional[Tuple[str, Any]]:
        """Get latest sensor event (non-blocking)."""
        try:
            return self.status_queue.get_nowait()
        except queue.Empty:
            return None
    
    def wait_for_move_complete(self, timeout: float = 10.0) -> bool:
        """Wait for current move to complete."""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                response = self.response_queue.get(timeout=0.1)
                if response.get('cmd') == 'move_complete':
                    return True
            except queue.Empty:
                pass
        
        return False
    
    def _get_next_command_id(self) -> int:
        """Get next unique command ID."""
        self.last_command_id += 1
        return self.last_command_id

# Utility functions for position conversion
def pixels_to_steps(pixel_x: int, pixel_y: int, 
                   image_width: int = 1280, image_height: int = 720,
                   table_width_steps: int = 3500, table_height_steps: int = 7000) -> Tuple[int, int]:
    """
    Convert pixel coordinates from camera to stepper motor steps.
    
    Args:
        pixel_x, pixel_y: Pixel coordinates from camera
        image_width, image_height: Camera resolution
        table_width_steps, table_height_steps: Table size in motor steps
        
    Returns:
        Tuple of (steps_x, steps_y)
    """
    # Convert pixels to normalized coordinates (0-1)
    norm_x = pixel_x / image_width
    norm_y = pixel_y / image_height
    
    # Convert to motor steps
    steps_x = int(norm_x * table_width_steps)
    steps_y = int(norm_y * table_height_steps)
    
    return steps_x, steps_y

def steps_to_pixels(steps_x: int, steps_y: int,
                   image_width: int = 1280, image_height: int = 720,
                   table_width_steps: int = 3500, table_height_steps: int = 7000) -> Tuple[int, int]:
    """Convert motor steps to pixel coordinates."""
    # Convert steps to normalized coordinates
    norm_x = steps_x / table_width_steps  
    norm_y = steps_y / table_height_steps
    
    # Convert to pixels
    pixel_x = int(norm_x * image_width)
    pixel_y = int(norm_y * image_height)
    
    return pixel_x, pixel_y
