#!/usr/bin/env python3
"""
Air Hockey Robot System Launcher
Simple script to start the complete air hockey system.
"""

import sys
import argparse
from air_hockey_system import AirHockeySystem
from ai_strategy import Difficulty

def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Air Hockey Robot System")
    parser.add_argument('--difficulty', choices=['easy', 'medium', 'hard', 'expert'],
                       default='medium', help='AI difficulty level')
    parser.add_argument('--show-vision', action='store_true',
                       help='Show vision processing window (if display available)')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera index to use')
    
    args = parser.parse_args()
    
    print("Air Hockey Robot System")
    print("=" * 50)
    print(f"AI Difficulty: {args.difficulty}")
    print(f"Camera Index: {args.camera}")
    print(f"Vision Display: {'Enabled' if args.show_vision else 'Disabled'}")
    print("=" * 50)
    
    # Create the system
    system = AirHockeySystem()
    
    # Configure based on arguments
    difficulty_map = {
        'easy': Difficulty.EASY,
        'medium': Difficulty.MEDIUM,
        'hard': Difficulty.HARD,
        'expert': Difficulty.EXPERT
    }
    
    system.set_ai_difficulty(difficulty_map[args.difficulty])
    system.vision.set_camera_index(args.camera)
    
    if args.show_vision:
        system.vision.enable_display(True)
    
    try:
        # Start the system
        if not system.startup():
            print("System startup failed!")
            return 1
        
        print("\nAir Hockey System Ready!")
        print("Insert coin to start game...")
        
        # Run the main loop
        system.run()
        
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"System error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
