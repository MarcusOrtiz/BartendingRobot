import subprocess
import pixelConverter as pc

camera_command = ["python3", "rs_camera.py"]
subprocess.run(camera_command)

detection_command = ['python3', 'yolov5/detect.py', "--weights",
                     'yolov5/bottle_weights/all_resizing600.pt',
                     "--imgsz", "1280", "720", "--conf", "0.85",
                     "--source", 'data/environment_photo.jpg',
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

pixel_to_cm = {45: [1/9.3824, 1/11.159, 1/13.0542], 60: [1/10.5602, 1/10.019, 1/10.9312]}
# for row, conversions in pixel_to_cm.items(): 
#     for i in range(len(conversions)): conversions[i] = conversions[i] / 2 
row = 45 if float(bottle_dims['green_bottle'][2]) < 120 else 60

# print(f'y-axis: {row}')

g_bottle_relative = float(bottle_dims['green_bottle'][0]) - float(bottle_dims['cup'][0])
b_bottle_relative = float(bottle_dims['blue_bottle'][0]) - float(bottle_dims['cup'][0])

# print(f'x-g: {g_bottle_relative}')
# print(f'x-g: {b_bottle_relative}')



pixel_to_cm = pixel_to_cm[row]
g_bottle_pos = (pc.sign(g_bottle_relative) *
                pc.pixel_to_cm(abs(g_bottle_relative), pixel_to_cm[0], pixel_to_cm[1], pixel_to_cm[2]))
b_bottle_pos = (pc.sign(b_bottle_relative) *
                pc.pixel_to_cm(abs(b_bottle_relative), pixel_to_cm[0], pixel_to_cm[1], pixel_to_cm[2]))

print(row)
print(f'green: {g_bottle_pos}, blue: {b_bottle_pos}')

right_command = ["python3", "../catkin_ws/src/ur5e_gripper_gazebo/moveit/motion_planning_right.py"]
left_command = ["python3", "../catkin_ws/src/ur5e_gripper_gazebo/moveit/motion_planning_left.py"]

if g_bottle_pos and g_bottle_pos < 0:
    right_command = right_command + ["2", f"{g_bottle_pos/100}", f"{row/100}"]
    subprocess.run(right_command)
elif b_bottle_pos is not None and b_bottle_pos < 0:
    right_command = right_command + ["2", f"{b_bottle_pos/100}", f"{row/100}"]
    
    subprocess.run(right_command)
if g_bottle_pos and g_bottle_pos > 0:
    pass
elif b_bottle_pos and b_bottle_pos > 0:
    pass

# right_command = ["python", "../catkin_ws/src/ur5e_gripper_gazebo/moveit/motion_planning_right.py"]
# # else:
# subprocess.run(right_command)


