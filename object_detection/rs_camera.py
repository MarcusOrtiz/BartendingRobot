import cv2
import pyrealsense2
import time
import numpy as np
from realsense_depth import *
import os

os.remove('data/environment_photo.jpg') if os.path.exists('data/environment_photo.jpg') else None

dc = DepthCamera()

time.sleep(1)

ret, depth_image, color_image = dc.get_frame()

# Save color image
cv2.imwrite('data/environment_photo.jpg', color_image)

