from PIL import Image
import numpy as np
import math

# Map properties
map_resolution = 0.05               # meters per pixel
map_origin = (-100, -100)           # Origin in real-world coordinates
map_width, map_height = 4000, 4000  # Map size in pixels

real_world_coords = []              # real-world coordinates list to store (tuple) data
file_path = '/home/esirem/Desktop/M2_Learning/3-M2_ViBot/1-Robotics/3-Robotics_Project/lane_tracking_sim_ws/src/lane_tracking_pkg/scripts/pose_data.txt'

# Open and read the Robot Position and orientation data file
with open(file_path, 'r') as f:
    next(f)                                               # Skip the header
    for line in f:
        line = line.strip()                               # Remove any leading/trailing whitespace
        
        if line and line.startswith("(") and line.endswith("),"): # Check if the line contains a tuple
            line = line[:-1]                              # Remove the trailing comma
            
            position_tuple = eval(line)                   # Convert the string into a tuple

            real_world_coords.append(position_tuple)      # append the list

print("length of tuple is: ", len(real_world_coords))

# Function to convert real-world coordinates to map coordinates:
def real_world_to_map(real_x, real_y, map_origin, map_resolution):
    
    pixel_x = int((real_x - map_origin[0]) / map_resolution)
    pixel_y = map_height - int((real_y - map_origin[1]) / map_resolution) # Invert Y-axis
    
    return pixel_x, pixel_y

# Load the PGM map
img = Image.open('/home/esirem/Desktop/M2_Learning/3-M2_ViBot/1-Robotics/3-Robotics_Project/slam_lane_tracking_sim_ws/src/slam_lane_tracking_pkg/maps/my_map.pgm')
map_pixels = np.array(img)

# Set all obstacles to free space before adding lines as virtual obstacles
map_pixels[map_pixels == 0] = 255 

variable_upper = 0.25
variable_lower = 0.15
map_adjustment = 38

# Loop through real-world coordinates and convert to map coordinates
for (real_x, real_y, pose) in real_world_coords:
    
    """
    Based on the data of the robot position and orientation, we can find out the position of upper and lower lines
    upper line x =  robot x + a fixed height * cos(orientation of the robot - 90 degrees)
    lower line x =  robot x + a fixed height * cos(orientation of the robot + 90 degrees)
    """
    real_upper_x = real_x + variable_upper * math.cos(pose-1.5708) 
    real_upper_y = real_y + variable_upper * math.sin(pose-1.5708)
    map_upper_x, map_upper_y = real_world_to_map(real_upper_x, real_upper_y, map_origin, map_resolution)
    
    real_lower_x = real_x + variable_lower * math.cos(pose+1.5708)
    real_lower_y = real_y + variable_lower * math.sin(pose+1.5708)
    map_lower_x, map_lower_y = real_world_to_map(real_lower_x, real_lower_y, map_origin, map_resolution)
    
    # map_upper_x =  map_upper_x - 2
    # map_lower_x =  map_lower_x - 2
    map_upper_y =  map_upper_y - map_adjustment
    map_lower_y =  map_lower_y - map_adjustment

    # Check if coordinates are within map bounds
    if 0 <= map_upper_x < map_width and 0 <= map_upper_y < map_height:
        # Mark the pixel as an obstacle (e.g., set it to black)
        map_pixels[map_upper_y, map_upper_x] = 0  # PGM is grayscale, so 0 is black

    # Check if coordinates are within map bounds
    if 0 <= map_lower_x < map_width and 0 <= map_lower_y < map_height:
        # Mark the pixel as an obstacle (e.g., set it to black)
        map_pixels[map_lower_y, map_lower_x] = 0  # PGM is grayscale, so 0 is black

# Saving the modified map as a PGM file
modified_img = Image.fromarray(map_pixels)
modified_img.save('/home/esirem/Desktop/M2_Learning/3-M2_ViBot/1-Robotics/3-Robotics_Project/slam_lane_tracking_sim_ws/src/slam_lane_tracking_pkg/maps/new_map.pgm')
print("Modified map saved as 'new_map.pgm'")