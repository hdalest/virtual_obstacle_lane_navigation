# Virtual Obstacle Lane Navigation with Autonomous Mapping
This repository features a custom ROS package, slam_lane_tracking_pkg, designed to enable Turtlebot3 to autonomously navigate an autorace environment by tracking lanes and simultaneously mapping the surroundings. The package equips Turtlebot3 with the ability to follow lanes using camera sensor data, utilizing the cv_bridge package to process images and detect yellow and white lines.

Once the map is created, the robot’s pose data is used to designate the detected lanes as virtual obstacles, ensuring they are avoided during autonomous navigation. 
For more information on "lane tracking" and "how the robot pose data is generated", please refer to the following repository: [autonomous lane tracking.](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation.git)


### Demo Video
You can watch the demo video by clicking on the below image
[![Watch the video]()](https://youtu.be/AojJgdawQTQ)

## Gazebo World
Below image shows the Gazebo world used in this project. The world contains white and yellow lines.

![Gazebo World](https://github.com/EhtishamAshraf/Ros_Lane_Tracking/blob/main/src/lane_tracking_pkg/Images/autorace_world.png)

### Note 
1.  Details about cloning the repository are given at the end of this **readme file**

## Lane Tracking Logic & Robot's Movement Control using PD Controller
The logic behind lane tracking algorithm and Robot movement control is explained in detail in the following repo: [autonomous lane tracking.](https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation.git)

![Cmera Output](https://github.com/EhtishamAshraf/Ros_Lane_Tracking/blob/main/src/lane_tracking_pkg/Images/camera_output.png)

## Robot's Pose data
While the robot is tracking a lane, its position and orientation data are continuously recorded in a .txt file. This stored data is later utilized to mark the lanes as virtual obstacles for Autonomous Navigation. The Robot pose data is stored and saved in the previous repo.

## Running the Simulation
To run the simulation, launch the slam_lane_tracking launch file, In order to launch the launch file (you should first navigate inside the workspace and then, use the following command).
### 1. Mapping
```bash
roslaunch slam_lane_tracking_pkg slam_lane_tracking.launch
```
### 1. Navigation
```bash
roslaunch slam_lane_tracking_pkg navigation.launch
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
git clone https://github.com/EhtishamAshraf/virtual_obstacle_lane_navigation.git
```
```bash
cd virtual_obstacle_lane_navigation
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

This script is responsible for tracking the lane without saving the robot's pose data.
```bash
lane_tracking.py
```

Now navigate to the launch folder inside the package
```bash
roslaunch slam_lane_tracking.launch
```
or

```bash
roslaunch navigation.launch 
```
