"""Quick camera FPS debug"""
import cv2
import time
from camera import initialize_camera

cap = initialize_camera()
if not cap:
    print("Camera failed")
    exit()

cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

print("Testing camera FPS...")
start = time.time()
frames = 0

for i in range(100):
    ret, frame = cap.read()
    if ret:
        frames += 1
    else:
        print(f"Frame {i} failed!")

elapsed = time.time() - start
print(f"Captured {frames} frames in {elapsed:.2f}s = {frames/elapsed:.1f} FPS")
cap.release()