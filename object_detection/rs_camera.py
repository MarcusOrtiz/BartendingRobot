import cv2
import pyrealsense2
import time
import numpy as np
from realsense_depth import *
import os


def capture_image():
    """
    Capture an image from the Realsense d435i camera and save it to the data folder
    """
    os.remove('data/environment_photo.jpg') if os.path.exists('data/environment_photo.jpg') else None # remove old image
    dc = DepthCamera()
    time.sleep(1)
    ret, depth_image, color_image = dc.get_frame()
    cv2.imwrite('data/environment_photo.jpg', color_image)


if __name__ == 'main':
    capture_image()
