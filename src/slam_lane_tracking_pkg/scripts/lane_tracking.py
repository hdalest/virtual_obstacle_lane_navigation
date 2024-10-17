#!/usr/bin/env python3

"""
                        --- Lane Tracking using Turtlebot3 Robot ---
This ROS node allows the Turtlebot3 Burger Robot to track the lane.


The node performs the following functions:

1. **Subscribes to the `/camera/cimage` topic, where it receives image data. 
   Robot detects and then track the lane autonomously using the image data.
   use rqt_image_view tool to figure out the exact topic where camera feed is being published.

2. **Publishes on the `/cmd_vel` topic, where velocity commands are published. 
   Robot moves based on the velocity commands.

3. **OpenCV integration - Lane Tracking**:
   - Once the lines are detected, openCV is used for Lane Tracking.
   - After detecting the lines, a contour is drawn on each line and the centroids for each line are found out.
   - The robot then adjusts it's movement using PD controller to track the lane efficiently.
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# PD components for Lane Tracking:
derivative = 0 
previous_error = 0

# Callback function to process data published on /camera/image topic
def ImageCallback(image_msg):
    bridge = CvBridge()
    try:
        rgb_image = bridge.imgmsg_to_cv2(image_msg, "bgr8") # ros image message converted to rgb image
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return
    
    """
    Lane Tracking logic:
    - Convert RGB image to HSV
    - Define the HSV range for detecting white and yellow color lines in the image
    - Create and display the masked image (highlighting the white and yellow detected lines)
    - Combine the white and yellow masks using a bitwise OR operation, allowing both lane lines to be visible in a single image
    - Find the contours in the masked image
    - If contours exist:
        - Get the second and third biggest contour based on the area
        - apply bitwise_and to extract the region where yellow and white lines are present in the original image
        - draw the contour for visualization
        - find the moments of the selected contours
        - find the centroids of the selected contours from the moments
        - draw a circle to highlight the centroid 
        - calculate the error by finding the difference b/w camera_center and the x position of the centroids of both lines
        - calculate the PD controller output based on the proportional, and derivative terms
        - Track the lane using PD controller
    """
    
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV) 
    
    lower_white = np.array([0, 0, 160])
    upper_white = np.array([180, 50, 255])
    mask_white = cv2.inRange(hsv_image, lower_white, upper_white)
    
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
    
    final_mask = cv2.bitwise_or(mask_white, mask_yellow)
    
    contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    numContours = len(contours)
    rospy.loginfo("Total number of detected Contours are: %d", numContours) 
    
    # creating two dictionaries to store x and y position of the centroids of both lines
    line_right = {} 
    line_left = {}
    
    if contours:
        # Sorting the contours by area in descending order and selecting the second and third largest contours
        largest_contours = sorted(contours, key=cv2.contourArea, reverse=True)[1:3]
        rospy.loginfo("--------------------")
        rospy.loginfo("Total Contours: %d", len(largest_contours))
        
        highlighted_lane_lines = cv2.bitwise_and(rgb_image, rgb_image, mask = final_mask)
        
        if len(largest_contours) > 0: # Second largest contour
            rospy.loginfo("Second Contour: %d", cv2.contourArea(largest_contours[0]))
            cv2.drawContours(highlighted_lane_lines, [largest_contours[0]], -1, (0, 0, 255), thickness=1)
            
            Moment_right = cv2.moments(largest_contours[0])
            line_right['x'] = int(Moment_right["m10"]/Moment_right["m00"])
            line_right['y'] = int(Moment_right["m01"]/Moment_right["m00"])
            cv2.circle(highlighted_lane_lines, (line_right['x'], line_right['y']), 3, (255, 0, 0), 5)
            
        if len(largest_contours) > 1: # Third largest contour
            rospy.loginfo("Third Contour: %d", cv2.contourArea(largest_contours[1]))
            cv2.drawContours(highlighted_lane_lines, [largest_contours[1]], -1, (0, 255, 0), thickness=1)
            
            Moment_left = cv2.moments(largest_contours[1])
            line_left['x'] = int(Moment_left["m10"]/Moment_left["m00"])
            line_left['y'] = int(Moment_left["m01"]/Moment_left["m00"])
            cv2.circle(highlighted_lane_lines, (line_left['x'], line_left['y']), 3, (0, 255, 0), 5)
            
        else:
            rospy.loginfo("No contours found!")
 
        rospy.loginfo("--------------------")
        
        cv2.imshow("Contours with Centroid", highlighted_lane_lines)
        cv2.waitKey(1) 
        
        _, width, _ = highlighted_lane_lines.shape
    
        camera_center = width/2         # finding the center of the camera
        
        if len(largest_contours) > 0 and len(largest_contours) < 3:
            desired_center = (line_left['x'] + line_right['x']) / 2
        else:
            desired_center = camera_center
    
        error = float(desired_center - camera_center)

        global derivative, previous_error

        derivative = error - previous_error 
        previous_error = error

        # For linear speed of 0.05, Kp = -0.0099 and Kd = -0.007 
        Kp = -0.039
        Kd = -0.032
        
        proportional_component = Kp * error     # proportional component of PID
        Derivative_component = Kd * derivative  # derivative component of PID
        PD_controller = proportional_component + Derivative_component # PD controller
                
        rospy.loginfo("Error: {}, P_component: {}, D_component: {}, PD: {}".format(error, proportional_component, Derivative_component, PD_controller))

        twist = Twist() # creating an instance of twist message
        
        # moving the robot accordingly
        twist.linear.x = 0.15
        twist.angular.z = PD_controller

        pub.publish(twist)  # Publish the twist message to the /cmd_vel topic, to control the robot's movements
    else:
        rospy.loginfo("No contours found!")
        twist = Twist()
        twist.linear.x = 0.09
        twist.angular.z = 0.05

def main():
    global pub    
    rospy.init_node('lane_tracking', anonymous=True)                    # initializing rosnode
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)            # create a publisher for the '/cmd_vel'
    sub_camera = rospy.Subscriber("camera/image", Image, ImageCallback) # create a subscriber for the '/camera/image'
    
    rospy.spin()                                                        # Keep the node running until shut down

if __name__ == '__main__':
    try:
        main()                          # Call the main function to start the node
    except rospy.ROSInterruptException: # Handle the case where ROS is interrupted (e.g., by shutting down)
        pass