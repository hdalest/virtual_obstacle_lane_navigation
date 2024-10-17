# Virtual Obstacle Lane Navigation with Autonomous Mapping
This repository features a custom ROS package, slam_lane_tracking_pkg, designed to enable Turtlebot3 to autonomously navigate an autorace environment by tracking lanes and simultaneously mapping the surroundings. The package equips Turtlebot3 with the ability to follow lanes using camera sensor data, utilizing the cv_bridge package to process images and detect yellow and white lines.

Once the map is created, the robot’s pose data is used to designate the detected lanes as virtual obstacles, ensuring they are avoided during autonomous navigation. For more information on how the robot pose data is generated, please refer to the following repository: [autonomous lane tracking.](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation.git)


### Demo Video
You can watch the demo video by clicking on the below image
[![Watch the video](https://github.com/EhtishamAshraf/Ros_Lane_Tracking/blob/main/src/lane_tracking_pkg/Images/simulation.png)](https://youtu.be/AojJgdawQTQ)

## Gazebo World
Below image shows the Gazebo world used in this project. The world contains white and yellow lines.

![Gazebo World](https://github.com/EhtishamAshraf/Ros_Lane_Tracking/blob/main/src/lane_tracking_pkg/Images/autorace_world.png)

### Note 
1.  Details about cloning the repository are given at the end of this **readme file**

## Lane Tracking Logic
  - Convert RGB image to HSV
  - Define the HSV range for detecting white and yellow color lines in the image
  - Create and display the masked image (highlighting the white and yellow detected lines)
  - Combine the white and yellow masks using a bitwise OR operation, allowing both lane 
    lines to be   
    visible in a single image
  - Find the contours in the masked image
  - If contours exist:
      - Get the second and third biggest contour based on the area
      - apply bitwise_and to extract the region where yellow and white lines are present 
        in the 
        original image
      - draw the contour for visualization
      - find the moments of the selected contours
      - find the centroids of the selected contours from the moments
      - draw a circle to highlight the centroid

![Cmera Output](https://github.com/EhtishamAshraf/Ros_Lane_Tracking/blob/main/src/lane_tracking_pkg/Images/camera_output.png)
        
## Robot's Movement Control using PD Controller
  - calculate the error by finding the difference b/w camera_center and the x position of the   
    centroids of both lines
  - calculate the PD controller output based on the proportional, and derivative terms
  - track the lane using PD controller
### Formula for Desired Center

The desired center is calculated as the midpoint between the yellow and white lines. The formula is:

desired_center = (line_left['x'] + line_right['x']) / 2

Where:
- `line_left['x']`  is the x-coordinate of the left line {line_left and line_right are lists}.
- `line_right['x']` is the x-coordinate of the right line.

## Robot's Pose data
While the robot is tracking a lane, its position and orientation data are continuously recorded in a .txt file. This stored data is later utilized to mark the lanes as virtual obstacles for Autonomous Navigation.
For part 2 of the project, have a look at the following repository:
[virtual_obstacle_lane_navigation](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation.git)

## Running the Simulation
To run the simulation, launch the lane_tracking launch file, In order to launch the launch file (you should first navigate inside the workspace and then, use the following command):
```bash
roslaunch lane_tracking_pkg lane_tracking.launch
```
### Note: 
I have used cv_bridge, turtlebot3_gazebo, gazebo_ros packages, so plesae download all necessary packages before cloning the repository.

# Clone the repository
```bash
sudo apt-get update
```
```bash
sudo apt-get install git
```
```bash
git clone https://github.com/EhtishamAshraf/turtlebot3_lane_tracking.git
```
```bash
cd turtlebot3_lane_tracking
```
Run the below commands in root folder of the workspace
```bash
catkin_make 
```
```bash
source devel/setup.bash 
```
Navigate to the Scripts folder inside the package and make the Python files executable by running the following command:
```bash
chmod +x *.py
```
There are two Python scripts located in the scripts folder:
1. This script is responsible for tracking the lane without saving the robot's pose data.
```bash
lane_tracking.py
```
2. This script tracks the lane while simultaneously saving the robot's pose data.
```bash
lane_with_centroids.py
```

Now navigate to the launch folder inside the package
```bash
roslaunch lane_tracking.launch
```
or

```bash
roslaunch lane_tracking.launch log_pose_to_file:=true
```
