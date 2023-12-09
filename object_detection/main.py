"""
This script runs the pipeline for object detection and motion planning
1. Captures an image from either the default or realsense camera
2. Detects and saves bounding boxes of cup, green bottle, and blue bottle in captured image
3. Predicts the real distance between the cup and the green bottle and the blue bottle
4. Calls right or/and left motion planning scripts to move the robot to the correct position
Usage:
    python3 main.py
"""


import subprocess
import pixel_converter as pc

CAMERA = "rs"
IMG_PATH = 'data/environment_photo.jpg'
PRED_PATH = './data/predictions/environment_photo.txt'
WEIGHTS = 'yolov5/bottle_weights/all_resizing600.pt'
IMG_HEIGHT = 720
IMG_WIDTH = 1280
CONF = 0.85
MOTION_RIGHT = "../catkin_ws/src/ur5e_gripper_gazebo/moveit/motion_planning_right.py"
MOTION_LEFT = "../catkin_ws/src/ur5e_gripper_gazebo/moveit/motion_planning_left.py"
ENVIRONMENT = "2"  # 1 for sim, 2 for real


def capture(camera_type):
    """
    Runs python script to capture image from rs or default camera

    Parameters:
        camera_type (str): 'rs' for realsense camera, anything else for default camera
    """
    camera_command = ["python3", "camera.py", camera_type]
    subprocess.run(camera_command)


def detect():
    """
    Runs python script to detect and save bounding boxes of cup, green bottle, and blue bottle in captured image
    Uses Yolov5 model, trained on simulated and real images of the cup, green bottle, and blue bottle
    """
    detection_command = ['python3', 'yolov5/detect.py', "--weights",
                         WEIGHTS,
                         "--imgsz", IMG_WIDTH, IMG_HEIGHT, "--conf", CONF,
                         "--source", IMG_PATH, "--save-txt"]
    subprocess.run(detection_command)


def predict():
    """
    Runs python script to predict the distance between the cup and the green bottle and the blue bottle
    """
    bottle_dims = {}

    # Open and read file with bounding box dimensions
    with open(PRED_PATH, 'r') as file:
        for line in file:
            parts = line.strip().split()
            # Check if line length is valid
            if len(parts) >= 5:
                obj = parts[0]
                dimensions = tuple(parts[1:5])
                bottle_dims[obj] = dimensions

    pixel_to_cm = {45: [1 / 9.3824, 1 / 11.159, 1 / 13.0542], 60: [1 / 10.5602, 1 / 10.019, 1 / 10.9312]}

    # Use width of green bottle to determine distance object distance from base
    row = 45 if float(bottle_dims['green_bottle'][2]) < 120 else 60

    # Calculate relative px distance between cup and bottles and then convert to cm
    g_bottle_relative = float(bottle_dims['green_bottle'][0]) - float(bottle_dims['cup'][0])
    b_bottle_relative = float(bottle_dims['blue_bottle'][0]) - float(bottle_dims['cup'][0])
    pixel_to_cm = pixel_to_cm[row]
    g_bottle_pos = (pc.sign(g_bottle_relative) *
                    pc.pixel_to_cm(abs(g_bottle_relative), pixel_to_cm[0], pixel_to_cm[1], pixel_to_cm[2]))
    b_bottle_pos = (pc.sign(b_bottle_relative) *
                    pc.pixel_to_cm(abs(b_bottle_relative), pixel_to_cm[0], pixel_to_cm[1], pixel_to_cm[2]))

    print(f'row {row} \n green: {g_bottle_pos}, blue: {b_bottle_pos}')
    return row, g_bottle_pos, b_bottle_pos


def move(row, g_bottle_pos, b_bottle_pos):
    """
    Using positions of bottles, calls right or/and left motion planning scripts

    Parameters:
        row (int): 45 or 60, the distance of the objects from the base of robot
        g_bottle_pos (float): position of green bottle relative to cup
        b_bottle_pos (float): position of blue bottle relative to cup
    """
    right_command = ["python3", MOTION_LEFT]
    left_command = ["python3", MOTION_RIGHT]

    # Green bottle is on the right
    if g_bottle_pos and g_bottle_pos < 0:
        right_command = right_command + ["2", f"{g_bottle_pos / 100}", f"{row / 100}"]
        subprocess.run(right_command)
    # Blue bottle is on the right
    elif b_bottle_pos is not None and b_bottle_pos < 0:
        right_command = right_command + ["2", f"{b_bottle_pos / 100}", f"{row / 100}"]
        subprocess.run(right_command)
    # Green bottle is on the left
    if g_bottle_pos and g_bottle_pos > 0:
        pass
    # Blue bottle is on the left
    elif b_bottle_pos and b_bottle_pos > 0:
        pass


if __name__ == '__main__':
    capture(CAMERA)
    detect()
    r, g, b = predict()
    move(r, g, b)
