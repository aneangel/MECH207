import cv2
import time
import os
import numpy as np
import argparse
import sys

# --- Configuration ---
# Camera Index: 
# This is typically 0 for the first camera detected. If your Jetson Nano 
# has an onboard camera, your USB camera might be 1 or higher. 
# If it fails, try incrementing this number (1, 2, 3, etc.).
CAMERA_INDEX = 0

# Desired resolution (720P)
WIDTH = 1280
HEIGHT = 720
FPS = 30

# Program Mode Configuration
SAVE_INTERVAL = 2.0  # Save image every 2 seconds in run mode
OUTPUT_DIR = "camera_captures"
# --- End Configuration ---

def has_display():
    """
    Check if display is available for showing video feed.
    
    Returns:
        bool: True if display is available, False if headless
    """
    try:
        display = os.environ.get('DISPLAY')
        if display and display.strip():
            return True
        
        # Additional check for Wayland
        wayland = os.environ.get('WAYLAND_DISPLAY')
        if wayland and wayland.strip():
            return True
            
        return False
    except:
        return False

def calculate_focus_score(image):
    """
    Calculate focus score using Laplacian variance.
    Higher values indicate better focus.
    
    Args:
        image: OpenCV image (BGR format)
        
    Returns:
        float: Focus score (higher = better focus)
    """
    # Convert to grayscale for focus analysis
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Calculate Laplacian variance - measures edge intensity
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    
    return laplacian_var

def get_focus_quality_rating(score):
    """
    Convert focus score to human-readable rating.
    
    Args:
        score: Focus score from calculate_focus_score()
        
    Returns:
        str: Focus quality description
    """
    if score > 1000:
        return "EXCELLENT"
    elif score > 500:
        return "GOOD"
    elif score > 200:
        return "FAIR"
    elif score > 100:
        return "POOR"
    else:
        return "VERY POOR"

def focus_calibration_mode(cap):
    """
    Interactive focus calibration mode.
    Provides real-time focus feedback to help user adjust camera focus.
    Shows live video if display is available.
    
    Args:
        cap: OpenCV VideoCapture object
    """
    display_available = has_display()
    
    print("\n" + "="*60)
    print("FOCUS CALIBRATION MODE")
    print("="*60)
    print("Instructions:")
    print("- Manually adjust your camera focus while watching the scores")
    if display_available:
        print("- Live video feed will be displayed")
        print("- Press 'q' in video window OR terminal to exit")
    else:
        print("- Running in headless mode (no video display)")
        print("- Press 'q' to return to main menu")
    print("- Focus score above 500 is GOOD, above 1000 is EXCELLENT")
    print("- Press 's' to save a test image")
    print("- Press Ctrl+C to exit")
    print("="*60)
    
    best_score = 0
    frame_count = 0
    window_name = "Focus Calibration - Live Feed"
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Error: Failed to retrieve frame.")
                time.sleep(0.01)
                continue
            
            frame_count += 1
            
            # Calculate focus score every few frames to reduce CPU load
            if frame_count % 5 == 0:
                focus_score = calculate_focus_score(frame)
                quality = get_focus_quality_rating(focus_score)
                
                # Track best score achieved
                if focus_score > best_score:
                    best_score = focus_score
                
                # Add focus info overlay to frame if displaying
                if display_available:
                    # Add text overlay to video
                    cv2.putText(frame, f"Focus: {focus_score:.1f} ({quality})", 
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(frame, f"Best: {best_score:.1f}", 
                               (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                    cv2.putText(frame, "Press 'q' to quit, 's' to save", 
                               (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                else:
                    # Print stats to console in headless mode
                    print(f"\rFocus Score: {focus_score:7.1f} | Quality: {quality:10} | Best: {best_score:7.1f} | Press 's' to save, 'q' to quit", end="", flush=True)
            
            # Show video if display available
            if display_available:
                cv2.imshow(window_name, frame)
            
            # Check for key press
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\nReturning to main menu...")
                break
            elif key == ord('s'):
                # Save current frame with focus info
                focus_score = calculate_focus_score(frame)
                quality = get_focus_quality_rating(focus_score)
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"focus_test_{timestamp}_score_{focus_score:.1f}.jpg"
                cv2.imwrite(filename, frame)
                print(f"\nSaved test image: {filename}")
                print(f"Focus score: {focus_score:.1f} ({quality})")
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.033)  # ~30 FPS
            
    except KeyboardInterrupt:
        print("\nFocus calibration interrupted by user.")
    
    finally:
        if display_available:
            cv2.destroyWindow(window_name)

def testing_and_calibration_menu(cap):
    """
    Testing and calibration submenu.
    
    Args:
        cap: OpenCV VideoCapture object
    """
    while True:
        display_available = has_display()
        print("\n" + "="*50)
        print("TESTING AND CALIBRATION MENU")
        print("="*50)
        print(f"Display Status: {'Available' if display_available else 'Headless Mode'}")
        print("="*50)
        print("1. Focus Calibration")
        print("2. Take Test Shot")
        print("3. Camera Information")
        print("4. Live Video Preview" + (" (Display Available)" if display_available else " (Not Available)"))
        print("5. Back to Main Menu")
        print("="*50)
        
        choice = input("Enter your choice (1-5): ").strip()
        
        if choice == '1':
            focus_calibration_mode(cap)
        elif choice == '2':
            take_test_shot(cap)
        elif choice == '3':
            show_camera_info(cap)
        elif choice == '4':
            if display_available:
                live_video_preview(cap)
            else:
                print("Live video preview requires a display. Currently running in headless mode.")
        elif choice == '5':
            break
        else:
            print("Invalid choice. Please enter 1-5.")

def take_test_shot(cap):
    """
    Take a single test shot and analyze it.
    
    Args:
        cap: OpenCV VideoCapture object
    """
    print("\nTaking test shot...")
    
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        return
    
    focus_score = calculate_focus_score(frame)
    quality = get_focus_quality_rating(focus_score)
    
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"test_shot_{timestamp}.jpg"
    cv2.imwrite(filename, frame)
    
    print(f"Test shot saved: {filename}")
    print(f"Focus Score: {focus_score:.1f}")
    print(f"Focus Quality: {quality}")
    print(f"Resolution: {frame.shape[1]}x{frame.shape[0]}")

def show_camera_info(cap):
    """
    Display detailed camera information.
    
    Args:
        cap: OpenCV VideoCapture object
    """
    print("\n" + "="*40)
    print("CAMERA INFORMATION")
    print("="*40)
    
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
    contrast = cap.get(cv2.CAP_PROP_CONTRAST)
    saturation = cap.get(cv2.CAP_PROP_SATURATION)
    
    print(f"Resolution: {width}x{height}")
    print(f"FPS: {fps}")
    print(f"Brightness: {brightness}")
    print(f"Contrast: {contrast}")
    print(f"Saturation: {saturation}")
    print(f"Display Available: {has_display()}")
    print("="*40)

def live_video_preview(cap):
    """
    Show live video preview when display is available.
    
    Args:
        cap: OpenCV VideoCapture object
    """
    if not has_display():
        print("Error: No display available for video preview.")
        return
    
    print("\n" + "="*60)
    print("LIVE VIDEO PREVIEW")
    print("="*60)
    print("Showing live camera feed...")
    print("Press 'q' in video window to return to menu")
    print("Press 'f' to show focus score overlay")
    print("Press 's' to save current frame")
    print("="*60)
    
    window_name = "Air Hockey Robot - Live Camera Feed"
    show_focus = False
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Error: Failed to retrieve frame.")
                time.sleep(0.01)
                continue
            
            frame_count += 1
            
            # Add focus overlay if enabled
            if show_focus and frame_count % 5 == 0:
                focus_score = calculate_focus_score(frame)
                quality = get_focus_quality_rating(focus_score)
                
                cv2.putText(frame, f"Focus: {focus_score:.1f} ({quality})", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Add control info
            cv2.putText(frame, "Press 'q' to quit", 
                       (10, frame.shape[0] - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, "Press 'f' for focus info, 's' to save", 
                       (10, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow(window_name, frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('f'):
                show_focus = not show_focus
                print(f"Focus overlay: {'ON' if show_focus else 'OFF'}")
            elif key == ord('s'):
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"live_preview_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Saved frame: {filename}")
                
                if show_focus:
                    focus_score = calculate_focus_score(frame)
                    quality = get_focus_quality_rating(focus_score)
                    print(f"Focus score: {focus_score:.1f} ({quality})")
    
    except KeyboardInterrupt:
        print("\nLive preview interrupted by user.")
    
    finally:
        cv2.destroyWindow(window_name)
        print("Live preview closed.")


def run_program_mode(cap):
    """
    Main program mode - continuous capture for robotics system.
    Shows live video if display is available, saves images at intervals.
    
    Args:
        cap: OpenCV VideoCapture object
    """
    display_available = has_display()
    
    print("\n" + "="*60)
    print("RUNNING AIR HOCKEY ROBOT CAMERA SYSTEM")
    print("="*60)
    print(f"Display Mode: {'Live Video + Capture' if display_available else 'Headless Capture Only'}")
    print(f"Saving images every {SAVE_INTERVAL}s to '{OUTPUT_DIR}/'")
    
    if display_available:
        print("Live video feed will be displayed")
        print("Press 'q' in video window OR Ctrl+C to stop")
    else:
        print("Press Ctrl+C to stop and return to menu")
    
    print("="*60)
    
    # Create output directory
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)
        print(f"Created output directory: {OUTPUT_DIR}")
    
    window_name = "Air Hockey Robot - Live Feed"
    last_save_time = 0
    frame_count = 0
    
    try:
        while True:
            ret, frame = cap.read()

            if not ret:
                print("Error: Failed to retrieve frame.")
                time.sleep(0.01)
                continue

            current_time = time.time()
            
            # Save image at specified intervals
            if current_time - last_save_time >= SAVE_INTERVAL:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f"{OUTPUT_DIR}/capture_{timestamp}.jpg"
                cv2.imwrite(filename, frame)
                frame_count += 1
                
                # Calculate and display focus score with each save
                focus_score = calculate_focus_score(frame)
                quality = get_focus_quality_rating(focus_score)
                
                print(f"Frame {frame_count}: {filename} | Focus: {focus_score:.1f} ({quality})")
                last_save_time = current_time
            
            # Show video if display available
            if display_available:
                # Add status overlay
                display_frame = frame.copy()
                cv2.putText(display_frame, f"Frames Saved: {frame_count}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Saving every {SAVE_INTERVAL}s", 
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                cv2.putText(display_frame, "Press 'q' to quit", 
                           (10, display_frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow(window_name, display_frame)
                
                # Check for quit key
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print(f"\nProgram stopped via video window. Captured {frame_count} frames.")
                    break
            
            time.sleep(0.033)  # ~30 FPS equivalent
            
    except KeyboardInterrupt:
        print(f"\nProgram stopped. Captured {frame_count} frames.")
    
    finally:
        if display_available:
            cv2.destroyWindow(window_name)


def main_menu():
    """
    Main CLI menu interface.
    """
    display_available = has_display()
    print("\n" + "="*60)
    print("AIR HOCKEY ROBOT CAMERA SYSTEM")
    print("Jetson Orin Nano - Robotics Vision Module")
    print("="*60)
    print(f"Display Status: {'Available - Live Video Enabled' if display_available else 'Headless Mode - Capture Only'}")
    print("="*60)
    print("1. Testing and Calibration")
    print("2. Run Program Mode")
    print("3. Exit")
    print("="*60)

def initialize_camera():
    """
    Initialize and configure the camera.
    
    Returns:
        cv2.VideoCapture: Configured camera object or None if failed
    """
    print(f"Attempting to open camera with index: {CAMERA_INDEX}")
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    if not cap.isOpened():
        print(f"FATAL ERROR: Could not open camera with index {CAMERA_INDEX}.")
        print("Suggestion: Please check camera index, permissions, or try a different port.")
        return None

    # Configure camera parameters
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    
    # Verify settings
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    print("-" * 40)
    print("Camera initialized successfully.")
    print(f"Actual Resolution: {actual_width}x{actual_height}")
    print(f"Display Available: {has_display()}")
    print("-" * 40)
    
    return cap

def main():
    """
    Main program entry point with CLI interface.
    """
    # Initialize camera once at startup
    cap = initialize_camera()
    if cap is None:
        sys.exit(1)
    
    try:
        while True:
            main_menu()
            choice = input("Enter your choice (1-3): ").strip()
            
            if choice == '1':
                testing_and_calibration_menu(cap)
            elif choice == '2':
                run_program_mode(cap)
            elif choice == '3':
                print("Shutting down camera system...")
                break
            else:
                print("Invalid choice. Please enter 1-3.")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    
    finally:
        # Clean up resources
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        print("Camera resources released. Goodbye!")


if __name__ == '__main__':
    main()