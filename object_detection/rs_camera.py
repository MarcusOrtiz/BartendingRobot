import cv2
import pyrealsense2
import time
import numpy as np
from realsense_depth import *

dc = DepthCamera()

time.sleep(1)

ret, depth_image, color_image = dc.get_frame()

# Save color image
cv2.imwrite('data/environment_photo.jpg', color_image)

