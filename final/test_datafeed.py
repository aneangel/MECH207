"""
Test computation.py with actual camera feed (HEADLESS MODE) - CAMERA FPS FIX

Forces camera to stream at consistent FPS without blocking.
"""

import cv2
import sys
import time
import os
import threading
from queue import Queue
from computation import TrajectoryCalculator
from camera import detect_black_objects, initialize_camera

def has_display():
    """Check if display is available."""
    try:
        display = os.environ.get('DISPLAY')
        if display and display.strip():
            return True
        return False
    except:
        return False


class VideoStreamThread:
    """Threaded video capture to prevent blocking on cap.read()"""
    
    def __init__(self, cap):
        self.cap = cap
        self.frame_queue = Queue(maxsize=2)
        self.stopped = False
        self.thread = threading.Thread(target=self._update, daemon=True)
        
    def start(self):
        """Start the thread"""
        self.thread.start()
        return self
    
    def _update(self):
        """Continuously read frames in background thread"""
        while not self.stopped:
            if not self.frame_queue.full():
                ret, frame = self.cap.read()
                if ret:
                    # If queue is full, remove old frame
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except:
                            pass
                    self.frame_queue.put(frame)
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.001)
    
    def read(self):
        """Get latest frame from queue"""
        if not self.frame_queue.empty():
            return True, self.frame_queue.get()
        return False, None
    
    def stop(self):
        """Stop the thread"""
        self.stopped = True
        self.thread.join()


def test_with_live_camera():
    """Test trajectory calculator with live camera feed"""
    print("\n" + "="*60)
    print("TESTING COMPUTATION WITH LIVE CAMERA (HEADLESS MODE)")
    print("="*60)
    
    # Check display status
    display_available = has_display()
    print(f"Display Available: {display_available}")
    
    # Initialize camera
    cap = initialize_camera()
    if cap is None:
        print("Failed to initialize camera!")
        return
    
    # Set camera buffer to minimum
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    # Start threaded video capture to prevent blocking
    print("Starting threaded video capture...")
    video_stream = VideoStreamThread(cap).start()
    time.sleep(1.0)  # Let camera warm up
    
    # Initialize trajectory calculator
    calc = TrajectoryCalculator(motor_controller=None)
    
    print("\nStarting live detection test...")
    print("Place a black object in view and move it")
    print("Press Ctrl+C to quit")
    print("-" * 60)
    
    frame_count = 0
    detection_count = 0
    interception_count = 0
    no_frame_count = 0
    
    # Statistics tracking
    start_time = time.time()
    last_stats_time = time.time()
    last_frame_time = time.time()
    stats_interval = 2.0  # Print stats every 2 seconds
    
    try:
        while True:
            # Non-blocking read from queue
            ret, frame = video_stream.read()
            current_time = time.time()
            
            if not ret or frame is None:
                no_frame_count += 1
                if no_frame_count % 100 == 0:
                    print(f"Waiting for frames... ({no_frame_count} empty reads)")
                time.sleep(0.001)  # Very short sleep
                continue
            
            # Reset no-frame counter
            no_frame_count = 0
            frame_count += 1
            
            # Calculate instantaneous FPS
            frame_time = current_time - last_frame_time
            instant_fps = 1.0 / frame_time if frame_time > 0 else 0
            last_frame_time = current_time
            
            # Detect black objects
            detections, annotated_frame = detect_black_objects(frame)
            
            if detections:
                detection_count += 1
                
                # Use largest detection (most likely the puck)
                largest = max(detections, key=lambda d: d['area'])
                
                print(f"\n[Frame {frame_count}] Detection #{detection_count} (FPS: {instant_fps:.1f}):")
                print(f"  Pixel Coords: {largest['center']}")
                print(f"  Area: {largest['area']} px²")
                
                # Update trajectory calculator
                calc.update_puck_position(largest)
                
                if calc.current_position:
                    x_mm, y_mm = calc.current_position
                    print(f"  Physical Position: ({x_mm:.2f}, {y_mm:.2f}) mm")
                
                if calc.current_velocity:
                    vx, vy = calc.current_velocity
                    speed = (vx**2 + vy**2) ** 0.5
                    print(f"  Velocity: ({vx:.1f}, {vy:.1f}) mm/s | Speed: {speed:.1f} mm/s")
                    
                    # Predict future position
                    future = calc.predict_future_position(0.5)
                    if future:
                        print(f"  Predicted (0.5s): ({future[0]:.2f}, {future[1]:.2f}) mm")
                    
                    # Try to calculate interception
                    intercept = calc.calculate_interception_point()
                    if intercept:
                        interception_count += 1
                        print(f"  >>> INTERCEPTION #{interception_count} NEEDED <<<")
                        print(f"      Target: ({intercept[0]:.2f}, {intercept[1]:.2f}) mm")
                        
                        # Calculate required motor movements
                        revs_a, revs_b = calc.corexy_inverse_kinematics(intercept[0], intercept[1])
                        
                        # Calculate time to intercept
                        if vy > 0.1:
                            time_to_intercept = (intercept[1] - y_mm) / vy
                            rpm_a = calc.calculate_motor_rpm(revs_a, time_to_intercept)
                            rpm_b = calc.calculate_motor_rpm(revs_b, time_to_intercept)
                            
                            print(f"      Time to Intercept: {time_to_intercept:.3f} s")
                            print(f"      Motor A (Right): {revs_a:.3f} revs @ {rpm_a:.1f} RPM")
                            print(f"      Motor B (Left):  {revs_b:.3f} revs @ {rpm_b:.1f} RPM")
                
                # Save annotated frame periodically
                if detection_count % 10 == 0:
                    filename = f"detection_{detection_count:04d}.jpg"
                    cv2.imwrite(filename, annotated_frame)
                    print(f"  Saved: {filename}")
            else:
                # Print periodic "no detection" message
                if frame_count % 30 == 0:
                    print(f"[Frame {frame_count}] No detection (FPS: {instant_fps:.1f})")
            
            # Print periodic statistics
            if current_time - last_stats_time >= stats_interval:
                elapsed_time = current_time - start_time
                avg_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
                status = calc.get_status()
                
                print(f"\n--- STATS (Frame {frame_count}) ---")
                print(f"  Elapsed Time: {elapsed_time:.1f} s")
                print(f"  Average FPS: {avg_fps:.1f}")
                print(f"  Instant FPS: {instant_fps:.1f}")
                print(f"  Total Detections: {detection_count}")
                print(f"  Interceptions Needed: {interception_count}")
                print(f"  Tracking Points: {status['tracking_points']}")
                if status['puck_position']:
                    print(f"  Last Puck Position: ({status['puck_position'][0]:.2f}, {status['puck_position'][1]:.2f}) mm")
                if status['puck_velocity']:
                    print(f"  Last Velocity: ({status['puck_velocity'][0]:.1f}, {status['puck_velocity'][1]:.1f}) mm/s")
                print("-" * 40)
                
                last_stats_time = current_time
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    finally:
        video_stream.stop()
        cap.release()
        cv2.destroyAllWindows()
        
        # Final statistics
        total_time = time.time() - start_time
        print("\n" + "="*60)
        print("TEST COMPLETE - FINAL STATISTICS")
        print("="*60)
        print(f"Total Runtime: {total_time:.1f} seconds")
        print(f"Total Frames Processed: {frame_count}")
        print(f"Average FPS: {frame_count / total_time if total_time > 0 else 0:.1f}")
        print(f"Total Detections: {detection_count}")
        print(f"Interceptions Needed: {interception_count}")
        
        if frame_count > 0:
            detection_rate = (detection_count / frame_count) * 100
            print(f"Detection Rate: {detection_rate:.1f}%")
        
        status = calc.get_status()
        print(f"\nFinal System Status:")
        print(f"  Robot Position: {status['robot_position']}")
        print(f"  Tracking Points in Buffer: {status['tracking_points']}")
        print("="*60)


def test_single_frame():
    """Quick test with a single frame capture"""
    print("\n" + "="*60)
    print("SINGLE FRAME TEST")
    print("="*60)
    
    cap = initialize_camera()
    if cap is None:
        print("Failed to initialize camera!")
        return
    
    calc = TrajectoryCalculator(motor_controller=None)
    
    print("Capturing single frame...")
    ret, frame = cap.read()
    
    if ret:
        detections, annotated_frame = detect_black_objects(frame)
        
        print(f"Frame captured: {frame.shape}")
        print(f"Detections found: {len(detections)}")
        
        if detections:
            for i, det in enumerate(detections):
                print(f"\nDetection {i+1}:")
                print(f"  Center: {det['center']}")
                print(f"  Area: {det['area']}")
                
                calc.update_puck_position(det)
                if calc.current_position:
                    print(f"  Physical: ({calc.current_position[0]:.2f}, {calc.current_position[1]:.2f}) mm")
        
        # Save annotated frame
        cv2.imwrite("test_single_frame.jpg", annotated_frame)
        print("\nSaved: test_single_frame.jpg")
    else:
        print("Failed to capture frame!")
    
    cap.release()
    print("="*60)


def test_conversion_accuracy():
    """Test pixel to mm conversion with known points"""
    print("\n" + "="*60)
    print("CONVERSION ACCURACY TEST")
    print("="*60)
    
    calc = TrajectoryCalculator(motor_controller=None)
    
    test_points = [
        (0, 0, "Top-Left Corner"),
        (1280, 0, "Top-Right Corner"),
        (0, 720, "Bottom-Left Corner"),
        (1280, 720, "Bottom-Right Corner"),
        (640, 360, "Center"),
    ]
    
    print(f"\nTable Dimensions: {calc.table_width_mm} x {calc.table_height_mm} mm")
    print(f"Camera Resolution: {calc.camera_width} x {calc.camera_height} px")
    print(f"Conversion: {calc.px_to_mm_x:.6f} mm/px (X), {calc.px_to_mm_y:.6f} mm/px (Y)")
    print("\nTest Points:")
    print("-" * 60)
    
    for px_x, px_y, label in test_points:
        mm_x, mm_y = calc.pixel_to_mm(px_x, px_y)
        print(f"{label:20s}: ({px_x:4d}, {px_y:3d}) px -> ({mm_x:6.2f}, {mm_y:6.2f}) mm")
    
    print("="*60)


def test_detection_only():
    """Test detection without trajectory calculation - lightweight mode"""
    print("\n" + "="*60)
    print("DETECTION ONLY TEST (LIGHTWEIGHT)")
    print("="*60)
    
    cap = initialize_camera()
    if cap is None:
        print("Failed to initialize camera!")
        return
    
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    # Use threaded capture
    video_stream = VideoStreamThread(cap).start()
    time.sleep(1.0)
    
    print("\nStarting detection-only test...")
    print("Press Ctrl+C to quit")
    print("-" * 60)
    
    frame_count = 0
    detection_count = 0
    start_time = time.time()
    last_frame_time = time.time()
    
    try:
        while True:
            ret, frame = video_stream.read()
            current_time = time.time()
            
            if not ret or frame is None:
                time.sleep(0.001)
                continue
            
            frame_count += 1
            frame_time = current_time - last_frame_time
            fps = 1.0 / frame_time if frame_time > 0 else 0
            last_frame_time = current_time
            
            detections, _ = detect_black_objects(frame)
            
            if detections:
                detection_count += 1
                largest = max(detections, key=lambda d: d['area'])
                print(f"[{frame_count:4d}] Detection: {largest['center']}, Area: {largest['area']:.0f} px² (FPS: {fps:.1f})")
            elif frame_count % 30 == 0:
                print(f"[{frame_count:4d}] No detection (FPS: {fps:.1f})")
    
    except KeyboardInterrupt:
        print("\n\nTest stopped")
    
    finally:
        video_stream.stop()
        cap.release()
        elapsed = time.time() - start_time
        print(f"\nProcessed {frame_count} frames in {elapsed:.1f}s ({frame_count/elapsed:.1f} FPS)")
        print(f"Detections: {detection_count}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Test trajectory calculator with camera')
    parser.add_argument('--mode', choices=['live', 'single', 'conversion', 'detect'], 
                       default='live',
                       help='Test mode: live (full tracking), single (one frame), conversion (calculations), detect (detection only)')
    
    args = parser.parse_args()
    
    if args.mode == 'live':
        test_with_live_camera()
    elif args.mode == 'single':
        test_single_frame()
    elif args.mode == 'conversion':
        test_conversion_accuracy()
    elif args.mode == 'detect':
        test_detection_only()