import sys

import cv2
import time

# Initialize the camera
cap = cv2.VideoCapture(0)  # 0 is usually the default camera

time.sleep(1)  # Give the camera time to warm up
# Capture a single frame
ret, frame = cap.read()

# Check if the frame was captured successfully
if ret:
    # Save the frame as an image file
    cv2.imwrite(f'data/{sys.argv[1]}', frame)
    print("Image captured and saved!")
else:
    RuntimeError("Error capturing an image")

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()