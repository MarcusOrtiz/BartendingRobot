import numpy as np
import sys

if __name__ == "__main__":
    pass


'''
# Example: Loading depth data from a file
file_path = 'data/measurement_data/2_Depth.raw'
depth_data = np.fromfile(file_path, dtype=np.uint16)

# Assuming the data is from an image sensor with a known resolution
height = 720  # Example height
width = 1280   # Example width
depth_data = depth_data.reshape((height, width))

# Convert to distances if necessary
# Example: if the data is in millimeters, you might convert it to meters
# depth_in_meters = depth_data / 1000.0

# Now, depth_in_meters is a 2D array with the depth in meters for each pixel

print(depth_data[100, 100])
'''