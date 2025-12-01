#!/usr/bin/env python3
"""
AI Strategy Module for Air Hockey Robot
Implements intelligent decision-making for autonomous paddle control.
"""

import time
import math
import numpy as np
from typing import Optional, Tuple, Dict, List, Any
from dataclasses import dataclass
from enum import Enum

class AIMode(Enum):
    """AI behavior modes."""
    DEFENSIVE = "defensive"      # Focus on blocking goals
    OFFENSIVE = "offensive"      # Aggressive attacking play
    BALANCED = "balanced"        # Mix of defense and offense
    POSITIONING = "positioning"  # Strategic positioning
    INTERCEPT = "intercept"      # Active puck interception

class Difficulty(Enum):
    """AI difficulty levels."""
    EASY = "easy"
    MEDIUM = "medium" 
    HARD = "hard"
    EXPERT = "expert"

@dataclass
class PuckData:
    """Puck tracking information from vision system."""
    position: Optional[Tuple[int, int]] = None
    velocity: Optional[Tuple[float, float]] = None
    speed: float = 0.0
    direction: str = "STATIONARY"
    predicted_positions: Dict[float, Tuple[int, int]] = None
    distance_to_paddle: Optional[float] = None
    moving_towards_paddle: bool = False
    intercept_time: Optional[float] = None
    intercept_point: Optional[Tuple[int, int]] = None
    
    def __post_init__(self):
        if self.predicted_positions is None:
            self.predicted_positions = {}

@dataclass
class PaddleData:
    """Paddle position and status information."""
    position: Optional[Tuple[int, int]] = None
    target_position: Optional[Tuple[int, int]] = None
    is_moving: bool = False
    last_move_time: float = 0.0

@dataclass
class TableBounds:
    """Air hockey table boundaries in pixels."""
    width: int = 1280
    height: int = 720
    robot_goal_y: int = 50      # Robot's goal line
    human_goal_y: int = 670     # Human's goal line
    robot_zone_y: int = 240     # Robot's defensive zone
    center_y: int = 360         # Table center
    
class AIStrategy:
    """
    Main AI strategy controller.
    Makes decisions about paddle movement based on game state and puck tracking.
    """
    
    def __init__(self, difficulty: Difficulty = Difficulty.MEDIUM):
        self.difficulty = difficulty
        self.current_mode = AIMode.BALANCED
        
        # Game state tracking
        self.puck = PuckData()
        self.paddle = PaddleData()
        self.table = TableBounds()
        
        # AI parameters based on difficulty
        self._setup_difficulty_parameters()
        
        # Decision timing
        self.last_decision_time = 0.0
        self.decision_interval = 0.05  # 20Hz decision rate
        
        # Strategy state
        self.current_strategy = "defensive_positioning"
        self.strategy_change_time = 0.0
        self.last_puck_side = "center"  # "robot", "center", "human"
        
        # Performance tracking
        self.successful_blocks = 0
        self.missed_blocks = 0
        self.aggressive_moves = 0
    
    def _setup_difficulty_parameters(self):
        """Configure AI parameters based on difficulty level."""
        params = {
            Difficulty.EASY: {
                'reaction_delay': 0.3,
                'prediction_accuracy': 0.6,
                'max_speed_factor': 0.6,
                'aggression': 0.2,
                'positioning_precision': 0.7
            },
            Difficulty.MEDIUM: {
                'reaction_delay': 0.15,
                'prediction_accuracy': 0.8,
                'max_speed_factor': 0.8,
                'aggression': 0.4,
                'positioning_precision': 0.85
            },
            Difficulty.HARD: {
                'reaction_delay': 0.08,
                'prediction_accuracy': 0.9,
                'max_speed_factor': 0.95,
                'aggression': 0.6,
                'positioning_precision': 0.95
            },
            Difficulty.EXPERT: {
                'reaction_delay': 0.03,
                'prediction_accuracy': 0.98,
                'max_speed_factor': 1.0,
                'aggression': 0.8,
                'positioning_precision': 0.99
            }
        }
        
        self.params = params[self.difficulty]
        print(f"AI Strategy initialized - Difficulty: {self.difficulty.value}")
    
    def update(self, vision_data: Dict, game_active: bool = True) -> Optional[Tuple[int, int, int]]:
        """
        Main AI update function.
        
        Args:
            vision_data: Data from hockey_vision.py
            game_active: Whether game is currently active
            
        Returns:
            Optional[Tuple[int, int, int]]: (target_x, target_y, speed) or None
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_decision_time < self.decision_interval:
            return None
        
        self.last_decision_time = current_time
        
        # Update internal state from vision data
        self._update_from_vision(vision_data)
        
        if not game_active:
            return self._get_idle_position()
        
        # Determine current AI mode based on game situation
        self._update_ai_mode()
        
        # Make movement decision based on current mode
        decision = self._make_movement_decision()
        
        # Add difficulty-based modifications
        if decision:
            decision = self._apply_difficulty_modifiers(decision)
        
        return decision
    
    def _update_from_vision(self, vision_data: Dict):
        """Update internal state from vision system data."""
        # Update puck data
        if 'puck_pos' in vision_data and vision_data['puck_pos']:
            self.puck.position = vision_data['puck_pos']
            
        if 'velocity' in vision_data:
            vx, vy = vision_data['velocity']
            if vx is not None and vy is not None:
                self.puck.velocity = (vx, vy)
                self.puck.speed = math.sqrt(vx*vx + vy*vy)
        
        if 'direction' in vision_data:
            self.puck.direction = vision_data['direction']
            
        if 'predicted_positions' in vision_data:
            self.puck.predicted_positions = vision_data['predicted_positions']
            
        if 'distance_to_paddle' in vision_data:
            self.puck.distance_to_paddle = vision_data['distance_to_paddle']
            
        if 'intercept_time' in vision_data:
            self.puck.intercept_time = vision_data['intercept_time']
            
        if 'intercept_point' in vision_data:
            self.puck.intercept_point = vision_data['intercept_point']
        
        # Update paddle data
        if 'paddle_pos' in vision_data and vision_data['paddle_pos']:
            self.paddle.position = vision_data['paddle_pos']
    
    def _update_ai_mode(self):
        """Determine appropriate AI mode based on current situation."""
        if not self.puck.position:
            self.current_mode = AIMode.POSITIONING
            return
        
        puck_y = self.puck.position[1]
        
        # Determine which side of table puck is on
        if puck_y < self.table.robot_zone_y:
            puck_side = "robot"
        elif puck_y > self.table.human_goal_y - 100:
            puck_side = "human"
        else:
            puck_side = "center"
        
        # Mode decision logic
        if puck_side == "robot" and self.puck.speed > 100:
            # Fast puck in robot zone - defensive mode
            self.current_mode = AIMode.DEFENSIVE
            self.current_strategy = "emergency_defense"
            
        elif (self.puck.moving_towards_paddle and 
              self.puck.intercept_time and 
              self.puck.intercept_time < 1.0):
            # Puck coming towards robot - intercept mode
            self.current_mode = AIMode.INTERCEPT
            self.current_strategy = "intercept_puck"
            
        elif puck_side == "human" and self.puck.speed < 50:
            # Slow puck in human zone - offensive opportunity
            if self.params['aggression'] > 0.5:
                self.current_mode = AIMode.OFFENSIVE
                self.current_strategy = "offensive_strike"
            else:
                self.current_mode = AIMode.POSITIONING
                self.current_strategy = "defensive_positioning"
                
        elif puck_side == "center":
            # Puck in center - balanced play
            self.current_mode = AIMode.BALANCED
            self.current_strategy = "center_control"
            
        else:
            # Default positioning
            self.current_mode = AIMode.POSITIONING
            self.current_strategy = "defensive_positioning"
        
        # Track side changes for strategy adaptation
        if puck_side != self.last_puck_side:
            self.strategy_change_time = time.time()
            self.last_puck_side = puck_side
    
    def _make_movement_decision(self) -> Optional[Tuple[int, int, int]]:
        """Make movement decision based on current AI mode."""
        if self.current_mode == AIMode.DEFENSIVE:
            return self._defensive_strategy()
            
        elif self.current_mode == AIMode.OFFENSIVE:
            return self._offensive_strategy()
            
        elif self.current_mode == AIMode.BALANCED:
            return self._balanced_strategy()
            
        elif self.current_mode == AIMode.INTERCEPT:
            return self._intercept_strategy()
            
        elif self.current_mode == AIMode.POSITIONING:
            return self._positioning_strategy()
        
        return None
    
    def _defensive_strategy(self) -> Optional[Tuple[int, int, int]]:
        """Implement defensive strategy - block goals."""
        if not self.puck.position:
            return self._get_defensive_position()
        
        # If puck is moving towards goal, position to block
        if self.puck.velocity and self.puck.velocity[1] < -50:  # Moving towards robot goal
            # Calculate where puck will cross goal line
            puck_x, puck_y = self.puck.position
            vx, vy = self.puck.velocity
            
            if vy < 0:  # Moving towards robot
                # Calculate intersection with goal line
                time_to_goal = (puck_y - self.table.robot_goal_y) / abs(vy)
                intercept_x = puck_x + vx * time_to_goal
                
                # Clamp to table width
                intercept_x = max(100, min(self.table.width - 100, intercept_x))
                
                # Position paddle to block
                target_y = self.table.robot_goal_y + 80  # Slightly in front of goal
                speed = int(3000 * self.params['max_speed_factor'])
                
                return (int(intercept_x), target_y, speed)
        
        # Default defensive positioning
        return self._get_defensive_position()
    
    def _offensive_strategy(self) -> Optional[Tuple[int, int, int]]:
        """Implement offensive strategy - attack human goal."""
        if not self.puck.position or not self.paddle.position:
            return None
        
        puck_x, puck_y = self.puck.position
        paddle_x, paddle_y = self.paddle.position
        
        # If puck is slow and in human zone, move to strike
        if puck_y > self.table.center_y and self.puck.speed < 100:
            # Calculate strike position - slightly behind puck
            strike_x = puck_x
            strike_y = min(puck_y + 50, self.table.human_goal_y - 100)
            
            speed = int(2000 * self.params['max_speed_factor'])
            
            return (strike_x, strike_y, speed)
        
        # Move towards center for better position
        return (self.table.width // 2, self.table.center_y - 50, 1500)
    
    def _balanced_strategy(self) -> Optional[Tuple[int, int, int]]:
        """Implement balanced strategy - mix of offense and defense."""
        if not self.puck.position:
            return self._get_defensive_position()
        
        puck_x, puck_y = self.puck.position
        
        # If puck is in dangerous area, prioritize defense
        if puck_y < self.table.robot_zone_y:
            return self._defensive_strategy()
        
        # If puck is slow in human area, be slightly offensive
        if puck_y > self.table.center_y + 100 and self.puck.speed < 150:
            target_x = puck_x + (self.table.width // 2 - puck_x) * 0.3
            target_y = self.table.center_y - 30
            speed = int(1800 * self.params['max_speed_factor'])
            
            return (int(target_x), target_y, speed)
        
        # Default to center positioning
        return (self.table.width // 2, self.table.robot_zone_y, 1500)
    
    def _intercept_strategy(self) -> Optional[Tuple[int, int, int]]:
        """Implement intercept strategy - actively intercept puck."""
        if (self.puck.intercept_point and 
            self.puck.intercept_time and 
            self.puck.intercept_time > 0.1):
            
            intercept_x, intercept_y = self.puck.intercept_point
            
            # Ensure intercept point is in robot's area
            intercept_y = max(self.table.robot_goal_y + 50, intercept_y)
            intercept_y = min(self.table.robot_zone_y, intercept_y)
            
            # Calculate required speed to reach intercept point
            if self.paddle.position:
                paddle_x, paddle_y = self.paddle.position
                distance = math.sqrt((intercept_x - paddle_x)**2 + (intercept_y - paddle_y)**2)
                required_speed = distance / self.puck.intercept_time
                
                # Limit speed based on difficulty
                max_speed = 3500 * self.params['max_speed_factor']
                speed = min(int(required_speed * 1.2), int(max_speed))
                
                return (intercept_x, intercept_y, speed)
        
        # Fallback to defensive positioning
        return self._defensive_strategy()
    
    def _positioning_strategy(self) -> Optional[Tuple[int, int, int]]:
        """Implement positioning strategy - optimal table positioning."""
        return self._get_defensive_position()
    
    def _get_defensive_position(self) -> Tuple[int, int, int]:
        """Get optimal defensive position."""
        # Center of robot zone
        target_x = self.table.width // 2
        target_y = self.table.robot_zone_y - 20
        speed = int(1500 * self.params['max_speed_factor'])
        
        return (target_x, target_y, speed)
    
    def _get_idle_position(self) -> Tuple[int, int, int]:
        """Get position when game is not active."""
        # Park in safe position
        return (self.table.width // 2, self.table.robot_goal_y + 100, 1000)
    
    def _apply_difficulty_modifiers(self, decision: Tuple[int, int, int]) -> Tuple[int, int, int]:
        """Apply difficulty-based modifications to movement decision."""
        target_x, target_y, speed = decision
        
        # Add reaction delay (simulated by reducing precision)
        if self.params['reaction_delay'] > 0.1:
            # Add some positioning error for easier difficulties
            error_x = int(np.random.normal(0, (1 - self.params['positioning_precision']) * 50))
            error_y = int(np.random.normal(0, (1 - self.params['positioning_precision']) * 30))
            
            target_x = max(50, min(self.table.width - 50, target_x + error_x))
            target_y = max(self.table.robot_goal_y + 30, min(self.table.robot_zone_y, target_y + error_y))
        
        # Limit speed based on difficulty
        speed = int(speed * self.params['max_speed_factor'])
        
        return (target_x, target_y, speed)
    
    # === PUBLIC API METHODS ===
    
    def set_difficulty(self, difficulty: Difficulty):
        """Change AI difficulty level."""
        self.difficulty = difficulty
        self._setup_difficulty_parameters()
        print(f"AI difficulty changed to: {difficulty.value}")
    
    def get_current_strategy(self) -> str:
        """Get current strategy description."""
        return f"{self.current_mode.value}: {self.current_strategy}"
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """Get AI performance statistics."""
        total_defensive = self.successful_blocks + self.missed_blocks
        block_rate = self.successful_blocks / total_defensive if total_defensive > 0 else 0
        
        return {
            'difficulty': self.difficulty.value,
            'current_mode': self.current_mode.value,
            'current_strategy': self.current_strategy,
            'successful_blocks': self.successful_blocks,
            'missed_blocks': self.missed_blocks,
            'block_success_rate': block_rate,
            'aggressive_moves': self.aggressive_moves
        }
    
    def record_successful_block(self):
        """Record a successful defensive block."""
        self.successful_blocks += 1
    
    def record_missed_block(self):
        """Record a missed defensive opportunity."""
        self.missed_blocks += 1
    
    def record_aggressive_move(self):
        """Record an aggressive offensive move."""
        self.aggressive_moves += 1
