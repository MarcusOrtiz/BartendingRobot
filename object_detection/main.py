import subprocess
import pixelConverter as pc

camera_command = ["python", "camera.py", "environment_photo.jpg"]
subprocess.run(camera_command)

detection_command = ["python", "yolov5/detect.py", "--weights",
                     "yolov5/bottle_weights/all_resizing600.pt",
                     "--imgsz", "1280", "720", "--conf", "0.7",
                     "--source", "data/environment_photo.png",
                     "--save-txt"]
subprocess.run(detection_command)

# Initialize an empty dictionary
bottle_dims = {}

# Open the file
with open('./data/predictions/environment_photo.txt', 'r') as file:
    # Iterate over each line
    for line in file:
        # Split the line into parts
        parts = line.strip().split()  # Assuming space is the delimiter

        # Ensure there are at least four parts
        if len(parts) >= 5:
            # First part is the key, next three are the tuple values
            obj = parts[0]
            dimensions = tuple(parts[1:5])

            # Add to the dictionary
            bottle_dims[obj] = dimensions

pixel_to_cm = {45: (1/4.6912, 1/4.4415, 1/4.738)}, {60: (1/5.2801, 1/5.0095, 1/5.4656)}
row = 45 if float(bottle_dims['green_bottle'][2]) < 120 else 60

g_bottle_relative = abs(float(bottle_dims['green_bottle'][0]) - float(bottle_dims['cup'][0]))
b_bottle_relative = abs(float(bottle_dims['blue_bottle'][0]) - float(bottle_dims['cup'][0]))
g_bottle_pos = (pc.sign(g_bottle_relative) /
                pc.pixel_to_cm(g_bottle_relative, pixel_to_cm[row][0], pixel_to_cm[row][1], pixel_to_cm[row][2]))
b_bottle_pos = (pc.sign(b_bottle_relative) /
                pc.pixel_to_cm(b_bottle_relative, pixel_to_cm[row][0], pixel_to_cm[row][1], pixel_to_cm[row][2]))

print(g_bottle_pos, b_bottle_pos)

# if g_bottle_pos > 0:
right_command = ["python", "../catkin_ws/src/ur5e_gripper_gazebo/moveit/motion_planning_right.py"]
# else:
subprocess.run(right_command)


