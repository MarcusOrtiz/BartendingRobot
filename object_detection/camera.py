"""

"""


import os
import sys
import time
import cv2
from realsense_depth import *

ENV_IMAGE = 'data/environment_photo.jpg'

def capture_rs_image():
    """
    Capture an image from the Realsense d435i camera and save it to the data folder
    """
    os.remove(ENV_IMAGE) if os.path.exists(ENV_IMAGE) else None  # remove old image if it exists
    dc = DepthCamera()
    time.sleep(1)
    ret, depth_image, color_image = dc.get_frame()
    write_image(ret, color_image)


def capture_image():
    """
    Capture an image from the default camera and save it to the data folder
    """
    cap = cv2.VideoCapture(0)  # 0 is the default camera
    time.sleep(1)  # Give the camera time to warm up
    ret, frame = cap.read()
    write_image(ret, frame)

    cap.release()
    cv2.destroyAllWindows()


def write_image(ret, frame):
    """
    Write the image to the data folder if it was captured

    Parameters:
        ret (bool): True if the image was captured, False otherwise
        frame (numpy.ndarray): The frame to write
    """
    if ret:
        cv2.imwrite(ENV_IMAGE, frame)
        print("Image captured and saved!")
    else:
        RuntimeError("Error capturing an image")


if __name__ == 'main':
    # If rs use depth camera, else default
    if sys.argv[1] == 'rs':
        capture_rs_image()
    else:
        capture_image()