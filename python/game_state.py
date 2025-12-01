#!/usr/bin/env python3
"""
Air Hockey Game State Management
Handles game flow, scoring, and state transitions.
"""

import time
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Any, Callable

class GameState(Enum):
    """Game state enumeration."""
    IDLE = "idle"                    # Waiting for coin
    INITIALIZING = "initializing"    # Starting up systems
    HOMING = "homing"               # Homing paddle position
    READY = "ready"                 # Ready for game start
    PLAYING = "playing"             # Active gameplay
    GOAL_SCORED = "goal_scored"     # Goal celebration/reset
    GAME_OVER = "game_over"         # Game finished
    PAUSED = "paused"               # Game paused
    ERROR = "error"                 # System error state
    MAINTENANCE = "maintenance"      # Service mode

class PlayerSide(Enum):
    """Player identification."""
    HUMAN = "human"
    ROBOT = "robot"

@dataclass
class GameSettings:
    """Game configuration settings."""
    max_score: int = 7              # First to 7 wins
    game_time_limit: int = 300      # 5 minutes max per game
    puck_reset_delay: float = 2.0   # Delay after goal before new puck
    ai_difficulty: str = "medium"    # easy, medium, hard, expert
    air_pressure: int = 75          # Air pressure percentage
    paddle_speed: int = 2500        # Default paddle speed
    
@dataclass
class GameScore:
    """Game scoring information."""
    human_score: int = 0
    robot_score: int = 0
    game_start_time: float = 0.0
    last_goal_time: float = 0.0
    total_games: int = 0
    human_wins: int = 0
    robot_wins: int = 0

class GameStateManager:
    """
    Main game state management system.
    Controls game flow, scoring, and state transitions.
    """
    
    def __init__(self):
        self.current_state = GameState.IDLE
        self.previous_state = GameState.IDLE
        self.settings = GameSettings()
        self.score = GameScore()
        
        # State change callbacks
        self.state_callbacks: Dict[GameState, list] = {state: [] for state in GameState}
        
        # Game timing
        self.state_start_time = time.time()
        self.last_state_update = time.time()
        
        # Game events
        self.coin_inserted = False
        self.puck_in_play = False
        self.last_puck_position = None
        self.goal_cooldown = False
        
        # System status
        self.systems_ready = {
            'camera': False,
            'esp32': False,
            'motors': False,
            'blower': False
        }
    
    def update(self) -> bool:
        """
        Update game state machine.
        Call this regularly from main loop.
        
        Returns:
            bool: True if state changed
        """
        old_state = self.current_state
        current_time = time.time()
        
        # State machine logic
        if self.current_state == GameState.IDLE:
            self._handle_idle_state()
            
        elif self.current_state == GameState.INITIALIZING:
            self._handle_initializing_state()
            
        elif self.current_state == GameState.HOMING:
            self._handle_homing_state()
            
        elif self.current_state == GameState.READY:
            self._handle_ready_state()
            
        elif self.current_state == GameState.PLAYING:
            self._handle_playing_state()
            
        elif self.current_state == GameState.GOAL_SCORED:
            self._handle_goal_scored_state()
            
        elif self.current_state == GameState.GAME_OVER:
            self._handle_game_over_state()
            
        elif self.current_state == GameState.PAUSED:
            self._handle_paused_state()
            
        elif self.current_state == GameState.ERROR:
            self._handle_error_state()
            
        elif self.current_state == GameState.MAINTENANCE:
            self._handle_maintenance_state()
        
        # Check for state change
        if old_state != self.current_state:
            self._on_state_change(old_state, self.current_state)
            return True
        
        return False
    
    def _handle_idle_state(self):
        """Handle IDLE state - waiting for coin insertion."""
        if self.coin_inserted:
            self.transition_to_state(GameState.INITIALIZING)
    
    def _handle_initializing_state(self):
        """Handle INITIALIZING state - system startup."""
        # Check if all systems are ready
        if all(self.systems_ready.values()):
            self.transition_to_state(GameState.HOMING)
        
        # Timeout after 10 seconds
        if self._get_state_duration() > 10.0:
            self.transition_to_state(GameState.ERROR)
    
    def _handle_homing_state(self):
        """Handle HOMING state - paddle homing sequence."""
        # This will be controlled by the motor controller
        # For now, assume homing takes 3 seconds
        if self._get_state_duration() > 3.0:
            self.transition_to_state(GameState.READY)
    
    def _handle_ready_state(self):
        """Handle READY state - ready to start game."""
        # Game starts automatically after brief delay
        if self._get_state_duration() > 2.0:
            self.start_new_game()
            self.transition_to_state(GameState.PLAYING)
    
    def _handle_playing_state(self):
        """Handle PLAYING state - active gameplay."""
        current_time = time.time()
        
        # Check for game time limit
        if current_time - self.score.game_start_time > self.settings.game_time_limit:
            self.end_game("time_limit")
        
        # Check for max score reached
        if (self.score.human_score >= self.settings.max_score or 
            self.score.robot_score >= self.settings.max_score):
            self.end_game("max_score")
    
    def _handle_goal_scored_state(self):
        """Handle GOAL_SCORED state - goal celebration and reset."""
        if self._get_state_duration() > self.settings.puck_reset_delay:
            # Check if game should end
            if (self.score.human_score >= self.settings.max_score or 
                self.score.robot_score >= self.settings.max_score):
                self.transition_to_state(GameState.GAME_OVER)
            else:
                # Continue playing
                self.transition_to_state(GameState.PLAYING)
                self.goal_cooldown = False
    
    def _handle_game_over_state(self):
        """Handle GAME_OVER state - game finished."""
        # Show results for 10 seconds then return to idle
        if self._get_state_duration() > 10.0:
            self.reset_game()
            self.transition_to_state(GameState.IDLE)
    
    def _handle_paused_state(self):
        """Handle PAUSED state - game temporarily paused."""
        pass  # Wait for external resume command
    
    def _handle_error_state(self):
        """Handle ERROR state - system error occurred."""
        # Auto-recovery attempt after 5 seconds
        if self._get_state_duration() > 5.0:
            self.transition_to_state(GameState.IDLE)
    
    def _handle_maintenance_state(self):
        """Handle MAINTENANCE state - service/calibration mode."""
        pass  # Manual exit required
    
    def transition_to_state(self, new_state: GameState):
        """Transition to new game state."""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()
            
            print(f"Game State: {self.previous_state.value} -> {new_state.value}")
    
    def _on_state_change(self, old_state: GameState, new_state: GameState):
        """Handle state change callbacks."""
        # Execute callbacks for new state
        for callback in self.state_callbacks.get(new_state, []):
            try:
                callback(old_state, new_state)
            except Exception as e:
                print(f"State callback error: {e}")
    
    def _get_state_duration(self) -> float:
        """Get time spent in current state."""
        return time.time() - self.state_start_time
    
    # === PUBLIC API METHODS ===
    
    def register_state_callback(self, state: GameState, callback: Callable):
        """Register callback for state entry."""
        self.state_callbacks[state].append(callback)
    
    def set_system_ready(self, system: str, ready: bool):
        """Set system readiness status."""
        if system in self.systems_ready:
            self.systems_ready[system] = ready
            print(f"System '{system}' ready: {ready}")
    
    def insert_coin(self):
        """Handle coin insertion event."""
        if self.current_state == GameState.IDLE:
            self.coin_inserted = True
            print("Coin inserted - Starting new game")
    
    def goal_scored(self, scoring_side: PlayerSide):
        """Handle goal scoring event."""
        if self.current_state != GameState.PLAYING or self.goal_cooldown:
            return False
        
        current_time = time.time()
        
        # Update score
        if scoring_side == PlayerSide.HUMAN:
            self.score.human_score += 1
            print(f"Human scores! Score: Human {self.score.human_score} - Robot {self.score.robot_score}")
        else:
            self.score.robot_score += 1
            print(f"Robot scores! Score: Human {self.score.human_score} - Robot {self.score.robot_score}")
        
        self.score.last_goal_time = current_time
        self.goal_cooldown = True
        self.transition_to_state(GameState.GOAL_SCORED)
        
        return True
    
    def start_new_game(self):
        """Initialize a new game."""
        self.score.human_score = 0
        self.score.robot_score = 0
        self.score.game_start_time = time.time()
        self.puck_in_play = True
        self.goal_cooldown = False
        
        print("New game started!")
    
    def end_game(self, reason: str):
        """End current game."""
        # Determine winner
        if self.score.human_score > self.score.robot_score:
            winner = "Human"
            self.score.human_wins += 1
        elif self.score.robot_score > self.score.human_score:
            winner = "Robot"
            self.score.robot_wins += 1
        else:
            winner = "Tie"
        
        self.score.total_games += 1
        
        print(f"Game Over! Winner: {winner} (Reason: {reason})")
        print(f"Final Score: Human {self.score.human_score} - Robot {self.score.robot_score}")
        
        self.transition_to_state(GameState.GAME_OVER)
    
    def reset_game(self):
        """Reset game to initial state."""
        self.coin_inserted = False
        self.puck_in_play = False
        self.goal_cooldown = False
        
        print("Game reset - Ready for next player")
    
    def pause_game(self):
        """Pause current game."""
        if self.current_state == GameState.PLAYING:
            self.transition_to_state(GameState.PAUSED)
    
    def resume_game(self):
        """Resume paused game."""
        if self.current_state == GameState.PAUSED:
            self.transition_to_state(GameState.PLAYING)
    
    def enter_maintenance_mode(self):
        """Enter maintenance/service mode."""
        self.transition_to_state(GameState.MAINTENANCE)
    
    def exit_maintenance_mode(self):
        """Exit maintenance mode."""
        self.transition_to_state(GameState.IDLE)
    
    def trigger_error(self, error_msg: str):
        """Trigger error state."""
        print(f"System Error: {error_msg}")
        self.transition_to_state(GameState.ERROR)
    
    # === GETTERS ===
    
    def get_current_state(self) -> GameState:
        """Get current game state."""
        return self.current_state
    
    def get_score(self) -> GameScore:
        """Get current game score."""
        return self.score
    
    def get_settings(self) -> GameSettings:
        """Get game settings."""
        return self.settings
    
    def is_game_active(self) -> bool:
        """Check if game is currently active."""
        return self.current_state == GameState.PLAYING
    
    def is_system_ready(self) -> bool:
        """Check if all systems are ready."""
        return all(self.systems_ready.values())
    
    def get_game_time_remaining(self) -> float:
        """Get remaining game time in seconds."""
        if not self.is_game_active():
            return 0.0
        
        elapsed = time.time() - self.score.game_start_time
        return max(0.0, self.settings.game_time_limit - elapsed)
    
    def get_state_info(self) -> Dict[str, Any]:
        """Get comprehensive state information."""
        return {
            'current_state': self.current_state.value,
            'previous_state': self.previous_state.value,
            'state_duration': self._get_state_duration(),
            'game_time_remaining': self.get_game_time_remaining(),
            'score': {
                'human': self.score.human_score,
                'robot': self.score.robot_score
            },
            'systems_ready': self.systems_ready.copy(),
            'puck_in_play': self.puck_in_play,
            'goal_cooldown': self.goal_cooldown
        }
