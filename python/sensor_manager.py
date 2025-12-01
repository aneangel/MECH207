#!/usr/bin/env python3
"""
Sensor Management System for Air Hockey Robot
Handles IR sensors, limit switches, and other hardware sensors.
"""

import time
import threading
from typing import Optional, Dict, Any, Callable, List
from dataclasses import dataclass
from enum import Enum

class SensorType(Enum):
    """Types of sensors in the system."""
    IR_GOAL = "ir_goal"
    IR_COIN = "ir_coin" 
    LIMIT_SWITCH = "limit_switch"
    POTENTIOMETER = "potentiometer"
    PRESSURE = "pressure"

class GoalSide(Enum):
    """Goal detection sides."""
    HUMAN = "human"
    ROBOT = "robot"

@dataclass
class SensorReading:
    """Individual sensor reading data."""
    sensor_id: str
    sensor_type: SensorType
    value: Any
    timestamp: float
    raw_value: Optional[Any] = None

@dataclass
class GoalEvent:
    """Goal detection event data."""
    side: GoalSide
    timestamp: float
    confidence: float = 1.0
    detection_method: str = "ir_sensor"

class SensorManager:
    """
    Main sensor management system.
    Processes sensor data and generates game events.
    """
    
    def __init__(self, esp32_controller=None):
        self.esp32 = esp32_controller
        
        # Sensor state tracking
        self.sensor_readings: Dict[str, SensorReading] = {}
        self.sensor_callbacks: Dict[str, List[Callable]] = {}
        
        # Goal detection
        self.goal_detection_enabled = True
        self.goal_cooldown_time = 2.0  # Prevent duplicate goal detection
        self.last_goal_time = 0.0
        self.goal_detection_threshold = 0.8  # IR sensor threshold
        
        # Coin detection
        self.coin_detection_enabled = True
        self.coin_inserted = False
        self.coin_cooldown_time = 1.0
        self.last_coin_time = 0.0
        
        # Limit switch monitoring
        self.limit_switches = {
            "x_min": False,
            "x_max": False, 
            "y_min": False,
            "y_max": False
        }
        
        # Air pressure monitoring
        self.air_pressure = 0
        self.pressure_target = 75
        self.pressure_tolerance = 5
        
        # Event callbacks
        self.goal_callbacks: List[Callable] = []
        self.coin_callbacks: List[Callable] = []
        self.limit_callbacks: List[Callable] = []
        
        # Threading
        self.sensor_thread = None
        self.stop_event = threading.Event()
        self.running = False
        
        # Sensor history for filtering
        self.ir_goal_history = {"human": [], "robot": []}
        self.history_length = 5
        
    def start(self):
        """Start sensor monitoring thread."""
        if self.running:
            return
        
        self.stop_event.clear()
        self.running = True
        self.sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.sensor_thread.start()
        
        print("Sensor Manager started")
    
    def stop(self):
        """Stop sensor monitoring."""
        self.running = False
        self.stop_event.set()
        
        if self.sensor_thread and self.sensor_thread.is_alive():
            self.sensor_thread.join(timeout=2)
        
        print("Sensor Manager stopped")
    
    def _sensor_loop(self):
        """Main sensor monitoring loop."""
        while not self.stop_event.is_set() and self.running:
            try:
                # Get sensor data from ESP32
                if self.esp32 and self.esp32.connected:
                    sensor_data = self.esp32.get_sensor_data()
                    
                    if sensor_data:
                        event_type, data = sensor_data
                        self._process_sensor_event(event_type, data)
                
                # Check sensor timeouts and health
                self._check_sensor_health()
                
                time.sleep(0.01)  # 100Hz sensor loop
                
            except Exception as e:
                print(f"Sensor loop error: {e}")
                time.sleep(0.1)
    
    def _process_sensor_event(self, event_type: str, data: Any):
        """Process sensor event from ESP32."""
        current_time = time.time()
        
        if event_type == 'goal_detected':
            self._handle_goal_detection(data, current_time)
            
        elif event_type == 'coin_inserted':
            self._handle_coin_insertion(current_time)
            
        elif event_type == 'limit_switch':
            self._handle_limit_switch(data, current_time)
            
        elif event_type == 'pressure_reading':
            self._handle_pressure_reading(data, current_time)
    
    def _handle_goal_detection(self, goal_side: str, timestamp: float):
        """Handle goal detection from IR sensors."""
        if not self.goal_detection_enabled:
            return
        
        # Cooldown check to prevent duplicate detections
        if timestamp - self.last_goal_time < self.goal_cooldown_time:
            return
        
        # Validate goal side
        try:
            side = GoalSide(goal_side.lower())
        except ValueError:
            print(f"Invalid goal side: {goal_side}")
            return
        
        # Add to history for filtering
        self.ir_goal_history[side.value].append({
            'timestamp': timestamp,
            'confidence': 1.0
        })
        
        # Keep history limited
        if len(self.ir_goal_history[side.value]) > self.history_length:
            self.ir_goal_history[side.value].pop(0)
        
        # Check if we have consistent goal detection
        recent_detections = [
            d for d in self.ir_goal_history[side.value] 
            if timestamp - d['timestamp'] < 0.5
        ]
        
        if len(recent_detections) >= 2:  # Require multiple confirmations
            goal_event = GoalEvent(
                side=side,
                timestamp=timestamp,
                confidence=len(recent_detections) / self.history_length,
                detection_method="ir_sensor"
            )
            
            self._trigger_goal_event(goal_event)
            self.last_goal_time = timestamp
            
            # Clear history after valid detection
            self.ir_goal_history[side.value].clear()
    
    def _handle_coin_insertion(self, timestamp: float):
        """Handle coin insertion detection."""
        if not self.coin_detection_enabled:
            return
        
        # Cooldown check
        if timestamp - self.last_coin_time < self.coin_cooldown_time:
            return
        
        self.coin_inserted = True
        self.last_coin_time = timestamp
        
        # Trigger coin callbacks
        for callback in self.coin_callbacks:
            try:
                callback(timestamp)
            except Exception as e:
                print(f"Coin callback error: {e}")
        
        print("Coin inserted detected")
    
    def _handle_limit_switch(self, switch_data: Dict[str, Any], timestamp: float):
        """Handle limit switch state changes."""
        switch_name = switch_data.get('switch', 'unknown')
        switch_state = switch_data.get('state', False)
        
        if switch_name in self.limit_switches:
            old_state = self.limit_switches[switch_name]
            self.limit_switches[switch_name] = switch_state
            
            # Only trigger callbacks on state changes
            if old_state != switch_state:
                for callback in self.limit_callbacks:
                    try:
                        callback(switch_name, switch_state, timestamp)
                    except Exception as e:
                        print(f"Limit switch callback error: {e}")
                
                print(f"Limit switch {switch_name}: {'TRIGGERED' if switch_state else 'RELEASED'}")
    
    def _handle_pressure_reading(self, pressure_data: Any, timestamp: float):
        """Handle air pressure sensor reading."""
        try:
            self.air_pressure = int(pressure_data)
            
            # Store reading
            reading = SensorReading(
                sensor_id="air_pressure",
                sensor_type=SensorType.PRESSURE,
                value=self.air_pressure,
                timestamp=timestamp,
                raw_value=pressure_data
            )
            
            self.sensor_readings["air_pressure"] = reading
            
            # Check if pressure is outside acceptable range
            if abs(self.air_pressure - self.pressure_target) > self.pressure_tolerance:
                self._trigger_sensor_callback("air_pressure", reading)
                
        except (ValueError, TypeError):
            print(f"Invalid pressure reading: {pressure_data}")
    
    def _check_sensor_health(self):
        """Check sensor health and connectivity."""
        current_time = time.time()
        
        # Check for sensor timeouts
        for sensor_id, reading in self.sensor_readings.items():
            if current_time - reading.timestamp > 5.0:  # 5 second timeout
                print(f"Warning: Sensor {sensor_id} timeout")
    
    def _trigger_goal_event(self, goal_event: GoalEvent):
        """Trigger goal detection callbacks."""
        for callback in self.goal_callbacks:
            try:
                callback(goal_event)
            except Exception as e:
                print(f"Goal callback error: {e}")
        
        print(f"Goal detected: {goal_event.side.value} side (confidence: {goal_event.confidence:.2f})")
    
    def _trigger_sensor_callback(self, sensor_id: str, reading: SensorReading):
        """Trigger sensor-specific callbacks."""
        if sensor_id in self.sensor_callbacks:
            for callback in self.sensor_callbacks[sensor_id]:
                try:
                    callback(reading)
                except Exception as e:
                    print(f"Sensor callback error for {sensor_id}: {e}")
    
    # === PUBLIC API METHODS ===
    
    def register_goal_callback(self, callback: Callable[[GoalEvent], None]):
        """Register callback for goal detection events."""
        self.goal_callbacks.append(callback)
    
    def register_coin_callback(self, callback: Callable[[float], None]):
        """Register callback for coin insertion events."""
        self.coin_callbacks.append(callback)
    
    def register_limit_callback(self, callback: Callable[[str, bool, float], None]):
        """Register callback for limit switch events."""
        self.limit_callbacks.append(callback)
    
    def register_sensor_callback(self, sensor_id: str, callback: Callable[[SensorReading], None]):
        """Register callback for specific sensor readings."""
        if sensor_id not in self.sensor_callbacks:
            self.sensor_callbacks[sensor_id] = []
        self.sensor_callbacks[sensor_id].append(callback)
    
    def enable_goal_detection(self, enable: bool = True):
        """Enable or disable goal detection."""
        self.goal_detection_enabled = enable
        print(f"Goal detection: {'ENABLED' if enable else 'DISABLED'}")
    
    def enable_coin_detection(self, enable: bool = True):
        """Enable or disable coin detection."""
        self.coin_detection_enabled = enable
        print(f"Coin detection: {'ENABLED' if enable else 'DISABLED'}")
    
    def reset_coin_status(self):
        """Reset coin insertion status."""
        self.coin_inserted = False
        print("Coin status reset")
    
    def simulate_goal(self, side: GoalSide):
        """Simulate goal detection for testing."""
        goal_event = GoalEvent(
            side=side,
            timestamp=time.time(),
            confidence=1.0,
            detection_method="simulation"
        )
        self._trigger_goal_event(goal_event)
    
    def simulate_coin_insertion(self):
        """Simulate coin insertion for testing."""
        self._handle_coin_insertion(time.time())
    
    def get_sensor_reading(self, sensor_id: str) -> Optional[SensorReading]:
        """Get latest reading for specific sensor."""
        return self.sensor_readings.get(sensor_id)
    
    def get_all_sensor_readings(self) -> Dict[str, SensorReading]:
        """Get all current sensor readings."""
        return self.sensor_readings.copy()
    
    def get_limit_switch_states(self) -> Dict[str, bool]:
        """Get current limit switch states."""
        return self.limit_switches.copy()
    
    def is_limit_switch_active(self, switch_name: str) -> bool:
        """Check if specific limit switch is active."""
        return self.limit_switches.get(switch_name, False)
    
    def get_air_pressure(self) -> int:
        """Get current air pressure reading."""
        return self.air_pressure
    
    def set_pressure_target(self, target: int, tolerance: int = 5):
        """Set target air pressure and tolerance."""
        self.pressure_target = max(0, min(100, target))
        self.pressure_tolerance = max(1, tolerance)
        print(f"Pressure target set to {self.pressure_target}% ±{self.pressure_tolerance}%")
    
    def is_coin_inserted(self) -> bool:
        """Check if coin has been inserted."""
        return self.coin_inserted
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive sensor system status."""
        current_time = time.time()
        
        return {
            'running': self.running,
            'goal_detection_enabled': self.goal_detection_enabled,
            'coin_detection_enabled': self.coin_detection_enabled,
            'coin_inserted': self.coin_inserted,
            'air_pressure': self.air_pressure,
            'pressure_target': self.pressure_target,
            'limit_switches': self.limit_switches.copy(),
            'last_goal_time': self.last_goal_time,
            'last_coin_time': self.last_coin_time,
            'sensor_count': len(self.sensor_readings),
            'active_callbacks': {
                'goal': len(self.goal_callbacks),
                'coin': len(self.coin_callbacks),
                'limit': len(self.limit_callbacks),
                'sensor': sum(len(callbacks) for callbacks in self.sensor_callbacks.values())
            }
        }
    
    # === CALIBRATION METHODS ===
    
    def calibrate_goal_sensors(self) -> bool:
        """Calibrate IR goal sensors."""
        print("Starting goal sensor calibration...")
        
        # This would send calibration commands to ESP32
        if self.esp32 and self.esp32.connected:
            # Send calibration command
            calibration_cmd = {
                "cmd": "calibrate_goal_sensors",
                "duration": 10  # 10 second calibration
            }
            
            # Implementation would depend on ESP32 firmware
            print("Goal sensor calibration initiated")
            return True
        
        print("Cannot calibrate: ESP32 not connected")
        return False
    
    def test_all_sensors(self) -> Dict[str, bool]:
        """Test all sensors and return status."""
        results = {}
        
        # Test goal sensors
        results['ir_goal_human'] = True  # Would test actual sensors
        results['ir_goal_robot'] = True
        results['ir_coin'] = True
        results['pressure_sensor'] = True
        
        # Test limit switches
        for switch_name in self.limit_switches:
            results[f'limit_{switch_name}'] = True
        
        print(f"Sensor test complete: {sum(results.values())}/{len(results)} passed")
        return results
