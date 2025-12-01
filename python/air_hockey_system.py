#!/usr/bin/env python3
"""
Air Hockey Robot System - Main Orchestrator
Coordinates all subsystems for autonomous air hockey gameplay.

This is the main entry point that brings together:
- Vision system (hockey_vision.py)
- ESP32 motor controller
- Game state management
- AI strategy
- Sensor management
"""

import sys
import time
import threading
import signal
from typing import Dict, Any, Optional
import json

# Import our modules
from esp32_controller import ESP32Controller, pixels_to_steps
from game_state import GameStateManager, GameState, PlayerSide
from ai_strategy import AIStrategy, Difficulty
from sensor_manager import SensorManager, GoalSide
from vision_integration import VisionIntegration

class AirHockeySystem:
    """
    Main system coordinator for the air hockey robot.
    Integrates all subsystems and manages the game loop.
    """
    
    def __init__(self):
        print("Initializing Air Hockey Robot System")
        print("=" * 60)
        
        # Initialize subsystems
        self.esp32 = ESP32Controller()
        self.game_state = GameStateManager()
        self.ai_strategy = AIStrategy(difficulty=Difficulty.MEDIUM)
        self.sensor_manager = SensorManager(self.esp32)
        
        # Vision system integration
        self.vision = VisionIntegration(system_callback=self.update_vision_data)
        self.vision_data = {}
        self.vision_active = False
        
        # System state
        self.running = False
        self.paused = False
        self.emergency_stop = False
        
        # Performance tracking
        self.loop_count = 0
        self.last_performance_check = time.time()
        self.loop_times = []
        
        # Setup callbacks
        self._setup_callbacks()
        
        print("Air Hockey System initialized")
    
    def _setup_callbacks(self):
        """Setup inter-system callbacks and event handlers."""
        
        # Game state callbacks
        self.game_state.register_state_callback(GameState.INITIALIZING, self._on_initializing)
        self.game_state.register_state_callback(GameState.HOMING, self._on_homing)
        self.game_state.register_state_callback(GameState.PLAYING, self._on_game_start)
        self.game_state.register_state_callback(GameState.GOAL_SCORED, self._on_goal_scored)
        self.game_state.register_state_callback(GameState.GAME_OVER, self._on_game_over)
        
        # Sensor callbacks
        self.sensor_manager.register_goal_callback(self._on_goal_detected)
        self.sensor_manager.register_coin_callback(self._on_coin_inserted)
        self.sensor_manager.register_limit_callback(self._on_limit_switch)
        
        print("System callbacks configured")
    
    def startup(self) -> bool:
        """
        Initialize and start all subsystems.
        
        Returns:
            bool: True if startup successful
        """
        print("\nStarting up systems...")
        
        # 1. Connect to ESP32 motor controller
        print("Connecting to ESP32 motor controller...")
        if not self.esp32.connect():
            print("ERROR: Failed to connect to ESP32")
            return False
        
        self.game_state.set_system_ready('esp32', True)
        
        # 2. Initialize motor systems
        print("Initializing motors and blower...")
        if not self._initialize_motors():
            print("ERROR: Motor initialization failed")
            return False
        
        self.game_state.set_system_ready('motors', True)
        
        # 3. Initialize and start vision system
        print("Starting vision system...")
        if not self.vision.start_vision_processing():
            print("WARNING: Vision system failed to start")
        else:
            self.game_state.set_system_ready('camera', True)
        
        # 4. Start sensor monitoring
        print("Starting sensor systems...")
        self.sensor_manager.start()
        
        # 5. Setup blower and air pressure
        print("Setting up air system...")
        self.esp32.set_blower(True)
        self.esp32.set_air_pressure(self.game_state.settings.air_pressure)
        self.game_state.set_system_ready('blower', True)
        
        # 6. Test all systems
        print("Running system tests...")
        if not self._run_system_tests():
            print("WARNING: Some system tests failed")
        
        print("System startup complete!")
        return True
    
    def shutdown(self):
        """Gracefully shutdown all systems."""
        print("\nShutting down Air Hockey System...")
        
        self.running = False
        
        # Stop blower for safety
        if self.esp32:
            self.esp32.set_blower(False)
            self.esp32.emergency_stop_enable()
        
        # Stop subsystems
        if self.vision:
            self.vision.stop_vision_processing()
        
        if self.sensor_manager:
            self.sensor_manager.stop()
        
        if self.esp32:
            self.esp32.disconnect()
        
        print("System shutdown complete")
    
    def run(self):
        """
        Main system loop.
        This is the heart of the air hockey robot system.
        """
        print("\nStarting main system loop...")
        print("Press Ctrl+C to stop")
        print("=" * 60)
        
        self.running = True
        
        try:
            while self.running:
                loop_start = time.time()
                
                # 1. Update game state
                state_changed = self.game_state.update()
                
                # 2. Process vision data (if available)
                self._process_vision_data()
                
                # 3. Update AI strategy
                ai_decision = self._update_ai_strategy()
                
                # 4. Execute motor commands
                if ai_decision and not self.paused and not self.emergency_stop:
                    self._execute_motor_command(ai_decision)
                
                # 5. Update displays/status
                self._update_status_display()
                
                # Performance tracking
                loop_time = time.time() - loop_start
                self.loop_times.append(loop_time)
                self.loop_count += 1
                
                # Maintain ~50Hz main loop
                target_loop_time = 0.02  # 50Hz
                if loop_time < target_loop_time:
                    time.sleep(target_loop_time - loop_time)
                
                # Performance monitoring
                if time.time() - self.last_performance_check > 5.0:
                    self._log_performance()
        
        except KeyboardInterrupt:
            print("\nSystem stopped by user")
        except Exception as e:
            print(f"\nSystem error: {e}")
            self.emergency_stop = True
        
        finally:
            self.shutdown()
    
    def _initialize_motors(self) -> bool:
        """Initialize motor systems."""
        try:
            # Home the paddle to known position
            print("Homing paddle...")
            if not self.esp32.home_paddle():
                return False
            
            # Wait for homing to complete
            if not self.esp32.wait_for_move_complete(timeout=10.0):
                print("Homing timeout")
                return False
            
            print("Paddle homed successfully")
            return True
            
        except Exception as e:
            print(f"Motor initialization error: {e}")
            return False
    
    def _run_system_tests(self) -> bool:
        """Run comprehensive system tests."""
        tests_passed = 0
        total_tests = 0
        
        # Test ESP32 communication
        total_tests += 1
        if self.esp32 and self.esp32.connected:
            tests_passed += 1
            print("✓ ESP32 communication test passed")
        else:
            print("✗ ESP32 communication test failed")
        
        # Test motor movement
        total_tests += 1
        try:
            # Small test movement
            current_status = self.esp32.get_status()
            test_x = current_status.paddle_x_pos + 100
            test_y = current_status.paddle_y_pos
            
            if self.esp32.move_paddle(test_x, test_y, 1000):
                if self.esp32.wait_for_move_complete(timeout=3.0):
                    tests_passed += 1
                    print("✓ Motor movement test passed")
                else:
                    print("✗ Motor movement test timeout")
            else:
                print("✗ Motor movement test failed to start")
        except:
            print("✗ Motor movement test failed")
        
        # Test sensor system
        total_tests += 1
        if self.sensor_manager and self.sensor_manager.running:
            tests_passed += 1
            print("✓ Sensor system test passed")
        else:
            print("✗ Sensor system test failed")
        
        # Test vision system
        total_tests += 1
        if self.vision and self.vision.is_running():
            tests_passed += 1
            print("✓ Vision system test passed")
        else:
            print("✗ Vision system test failed")
        
        # Test blower
        total_tests += 1
        status = self.esp32.get_status()
        if status.blower_status:
            tests_passed += 1
            print("✓ Blower system test passed")
        else:
            print("✗ Blower system test failed")
        
        success_rate = tests_passed / total_tests
        print(f"System tests: {tests_passed}/{total_tests} passed ({success_rate:.1%})")
        
        return success_rate >= 0.75  # Require 75% success rate
    
    def _process_vision_data(self):
        """Process latest vision data from vision integration module."""
        if self.vision and self.vision.is_running():
            # Get latest vision data
            latest_data = self.vision.get_latest_vision_data()
            if latest_data:
                self.vision_data = latest_data
                self.vision_active = True
            else:
                self.vision_active = False
        else:
            self.vision_active = False
    
    def update_vision_data(self, vision_data: Dict[str, Any]):
        """
        Update vision data from hockey_vision.py.
        This method would be called by the vision system.
        
        Args:
            vision_data: Dictionary containing vision analysis results
        """
        self.vision_data = vision_data.copy()
        self.vision_active = True
        
        # Update game state with vision information
        if 'puck_pos' in vision_data and vision_data['puck_pos']:
            self.game_state.last_puck_position = vision_data['puck_pos']
            self.game_state.puck_in_play = True
        else:
            self.game_state.puck_in_play = False
    
    def _update_ai_strategy(self) -> Optional[tuple]:
        """Update AI strategy and get movement decision."""
        if not self.vision_active or not self.game_state.is_game_active():
            return None
        
        # Get AI decision based on current game state and vision data
        decision = self.ai_strategy.update(
            vision_data=self.vision_data,
            game_active=self.game_state.is_game_active()
        )
        
        return decision
    
    def _execute_motor_command(self, command: tuple):
        """Execute motor movement command."""
        try:
            target_x_pixels, target_y_pixels, speed = command
            
            # Convert pixel coordinates to motor steps
            target_x_steps, target_y_steps = pixels_to_steps(
                target_x_pixels, target_y_pixels
            )
            
            # Send command to ESP32
            if self.esp32:
                self.esp32.move_paddle(target_x_steps, target_y_steps, speed)
        
        except Exception as e:
            print(f"Motor command execution error: {e}")
    
    def _update_status_display(self):
        """Update status display (terminal output)."""
        # Only update display every second to avoid spam
        if self.loop_count % 50 == 0:  # 50Hz loop, so every 50 iterations = 1 second
            self._print_system_status()
    
    def _print_system_status(self):
        """Print comprehensive system status to terminal."""
        print(f"\n{'='*80}")
        print(f"AIR HOCKEY ROBOT SYSTEM STATUS - Loop {self.loop_count}")
        print(f"{'='*80}")
        
        # Game state
        game_info = self.game_state.get_state_info()
        print(f"GAME STATE: {game_info['current_state'].upper()}")
        print(f"  Score: Human {game_info['score']['human']} - Robot {game_info['score']['robot']}")
        print(f"  Time Remaining: {game_info['game_time_remaining']:.1f}s")
        
        # Vision system
        if self.vision_active and self.vision_data:
            puck_pos = self.vision_data.get('puck_pos')
            processing_fps = self.vision_data.get('processing_fps', 0)
            
            print(f"VISION: FPS {processing_fps:.1f}")
            
            if puck_pos:
                print(f"  Puck: ({puck_pos[0]}, {puck_pos[1]})")
                
                velocity = self.vision_data.get('velocity', (None, None))
                if velocity[0] is not None:
                    speed = self.vision_data.get('speed', 0)
                    print(f"  Velocity: {velocity[0]:.1f}, {velocity[1]:.1f} px/s | Speed: {speed:.1f}")
                
                direction = self.vision_data.get('direction', 'STATIONARY')
                print(f"  Movement: {direction}")
                
                # Show predictions
                predictions = self.vision_data.get('predicted_positions', {})
                if predictions:
                    pred_text = " | ".join([f"{t}s: {pos}" for t, pos in predictions.items() if pos])
                    if pred_text:
                        print(f"  Predictions: {pred_text}")
            else:
                print("  Puck: NOT DETECTED")
            
            paddle_pos = self.vision_data.get('paddle_pos')
            if paddle_pos:
                print(f"  Paddle: ({paddle_pos[0]}, {paddle_pos[1]})")
            else:
                print("  Paddle: NOT DETECTED")
        else:
            print("VISION: Not active")
        
        # AI Strategy
        strategy_info = self.ai_strategy.get_current_strategy()
        print(f"AI STRATEGY: {strategy_info}")
        
        # Motor status
        if self.esp32 and self.esp32.connected:
            motor_status = self.esp32.get_status()
            print(f"PADDLE: Position ({motor_status.paddle_x_pos}, {motor_status.paddle_y_pos}) steps")
            print(f"  Blower: {'ON' if motor_status.blower_status else 'OFF'}")
            print(f"  Pressure: {motor_status.air_pressure}%")
        else:
            print("MOTORS: Not connected")
        
        # System health
        print(f"SYSTEM: {'EMERGENCY STOP' if self.emergency_stop else 'RUNNING'}")
        if self.loop_times:
            avg_loop_time = sum(self.loop_times[-50:]) / len(self.loop_times[-50:])
            print(f"  Loop Time: {avg_loop_time*1000:.1f}ms | FPS: {1/avg_loop_time:.1f}")
    
    def _log_performance(self):
        """Log system performance metrics."""
        if self.loop_times:
            avg_time = sum(self.loop_times[-250:]) / len(self.loop_times[-250:])  # Last 5 seconds
            fps = 1 / avg_time if avg_time > 0 else 0
            
            print(f"Performance: {fps:.1f} FPS | Avg Loop: {avg_time*1000:.1f}ms")
            
            # Clear old loop times to prevent memory buildup
            if len(self.loop_times) > 500:
                self.loop_times = self.loop_times[-250:]
        
        self.last_performance_check = time.time()
    
    # === EVENT HANDLERS ===
    
    def _on_initializing(self, old_state, new_state):
        """Handle transition to initializing state."""
        print("Initializing game systems...")
        
        # Reset all systems for new game
        if self.esp32:
            self.esp32.emergency_stop_disable()
        
        self.paused = False
        self.emergency_stop = False
    
    def _on_homing(self, old_state, new_state):
        """Handle transition to homing state."""
        print("Homing paddle to start position...")
        
        if self.esp32:
            self.esp32.home_paddle()
    
    def _on_game_start(self, old_state, new_state):
        """Handle game start."""
        print("Game starting!")
        
        # Enable all systems for active gameplay
        self.sensor_manager.enable_goal_detection(True)
        
        # Reset AI statistics
        self.ai_strategy = AIStrategy(self.ai_strategy.difficulty)
    
    def _on_goal_scored(self, old_state, new_state):
        """Handle goal scored."""
        score = self.game_state.get_score()
        print(f"Goal scored! Current score: Human {score.human_score} - Robot {score.robot_score}")
        
        # Brief pause after goal
        self.paused = True
        
        # Resume after delay (handled by game state manager)
        def resume_after_delay():
            time.sleep(2)
            self.paused = False
        
        threading.Thread(target=resume_after_delay, daemon=True).start()
    
    def _on_game_over(self, old_state, new_state):
        """Handle game over."""
        score = self.game_state.get_score()
        
        if score.human_score > score.robot_score:
            print("Game Over - Human wins!")
        elif score.robot_score > score.human_score:
            print("Game Over - Robot wins!")
        else:
            print("Game Over - It's a tie!")
        
        # Move paddle to safe position
        if self.esp32:
            self.esp32.move_paddle(1750, 500, 1000)  # Center, defensive position
    
    def _on_goal_detected(self, goal_event):
        """Handle goal detection from sensors."""
        side = PlayerSide.HUMAN if goal_event.side == GoalSide.HUMAN else PlayerSide.ROBOT
        
        if self.game_state.goal_scored(side):
            if side == PlayerSide.ROBOT:
                self.ai_strategy.record_aggressive_move()
            print(f"Sensor detected goal: {side.value}")
    
    def _on_coin_inserted(self, timestamp):
        """Handle coin insertion."""
        print(f"Coin detected at {timestamp}")
        self.game_state.insert_coin()
        self.sensor_manager.reset_coin_status()
    
    def _on_limit_switch(self, switch_name, state, timestamp):
        """Handle limit switch activation."""
        if state:  # Switch activated
            print(f"Limit switch activated: {switch_name}")
            
            # Emergency stop if any limit switch is hit during gameplay
            if self.game_state.is_game_active():
                print("Emergency stop: Limit switch hit during game")
                self.emergency_stop = True
                self.esp32.emergency_stop_enable()
    
    # === PUBLIC API METHODS ===
    
    def pause_system(self):
        """Pause the system."""
        self.paused = True
        if self.game_state.is_game_active():
            self.game_state.pause_game()
    
    def resume_system(self):
        """Resume the system."""
        self.paused = False
        if self.game_state.get_current_state() == GameState.PAUSED:
            self.game_state.resume_game()
    
    def emergency_stop_system(self):
        """Emergency stop all systems."""
        self.emergency_stop = True
        if self.esp32:
            self.esp32.emergency_stop_enable()
        print("EMERGENCY STOP ACTIVATED")
    
    def clear_emergency_stop(self):
        """Clear emergency stop."""
        self.emergency_stop = False
        if self.esp32:
            self.esp32.emergency_stop_disable()
        print("Emergency stop cleared")
    
    def set_ai_difficulty(self, difficulty: Difficulty):
        """Change AI difficulty."""
        self.ai_strategy.set_difficulty(difficulty)
        print(f"AI difficulty set to: {difficulty.value}")

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\nReceived shutdown signal...")
    sys.exit(0)

def main():
    """Main entry point for the air hockey system."""
    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and start the air hockey system
    system = AirHockeySystem()
    
    try:
        # Startup all subsystems
        if not system.startup():
            print("System startup failed!")
            return 1
        
        # Run the main system loop
        system.run()
        
    except Exception as e:
        print(f"System error: {e}")
        return 1
    
    finally:
        system.shutdown()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
