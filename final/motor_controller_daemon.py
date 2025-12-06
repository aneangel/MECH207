#!/usr/bin/env python3
"""
Motor Controller Daemon

Runs motor controller in background, automatically monitoring IR sensors.
Designed to run as a systemd service on Jetson startup.

Author: Air Hockey Team
Date: December 3, 2025
"""

import sys
import time
import signal
from motor_controller import SimpleMotorController

# Global controller instance for signal handling
controller = None
running = True

def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    global running, controller
    print(f"\n[Daemon] Received signal {signum}, shutting down...")
    running = False
    if controller:
        controller.close()
    sys.exit(0)

def main():
    """Main daemon loop."""
    global controller, running
    
    # Register signal handlers
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    
    print("="*60)
    print("MOTOR CONTROLLER DAEMON")
    print("="*60)
    print("Auto-starting motor controller with IR sensor monitoring...")
    print("Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    retry_count = 0
    max_retries = 5
    
    while running and retry_count < max_retries:
        try:
            # Initialize controller with IR monitoring enabled
            controller = SimpleMotorController(enable_ir_monitor=True)
            
            # Connect
            if not controller.connect():
                print(f"[Daemon] Connection failed. Retry {retry_count + 1}/{max_retries} in 5 seconds...")
                retry_count += 1
                time.sleep(5)
                continue
            
            # Enable motors
            if not controller.enable():
                print("[Daemon] Failed to enable motors. Retrying in 5 seconds...")
                retry_count += 1
                time.sleep(5)
                continue
            
            # Reset retry count on successful connection
            retry_count = 0
            
            # Check calibration status
            cal_status = controller.check_calibration()
            if not cal_status.get('calibrated', False):
                print("\n[Daemon] ⚠ WARNING: Workspace limits NOT calibrated!")
                print("[Daemon] Software endstops are DISABLED")
                print("[Daemon] Run motor_controller.py manually to calibrate\n")
            else:
                print(f"\n[Daemon] ✓ Workspace calibrated")
                print(f"[Daemon]   X: {cal_status.get('x_min', 0):.1f} to {cal_status.get('x_max', 350):.1f} mm")
                print(f"[Daemon]   Y: {cal_status.get('y_min', 0):.1f} to {cal_status.get('y_max', 350):.1f} mm\n")
            
            print("[Daemon] ✓ Motor controller ready and monitoring IR sensors")
            print("[Daemon] System is running in background...\n")
            
            # Keep running and monitoring
            while running:
                time.sleep(1)
                
                # Optionally send periodic status check
                # Uncomment if you want periodic heartbeat
                # if int(time.time()) % 60 == 0:  # Every 60 seconds
                #     status = controller.send_cmd({"cmd": "status"})
                #     if status:
                #         print(f"[Daemon] Status: {status}")
            
        except KeyboardInterrupt:
            print("\n[Daemon] Keyboard interrupt received")
            break
            
        except Exception as e:
            print(f"[Daemon] Error: {e}")
            retry_count += 1
            if retry_count < max_retries:
                print(f"[Daemon] Retrying in 10 seconds... ({retry_count}/{max_retries})")
                time.sleep(10)
            else:
                print(f"[Daemon] Max retries reached. Exiting.")
                break
    
    # Cleanup
    print("\n[Daemon] Shutting down...")
    if controller:
        controller.close()
    print("[Daemon] Stopped.")

if __name__ == "__main__":
    main()
