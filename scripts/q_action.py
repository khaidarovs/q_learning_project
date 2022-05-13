#!/usr/bin/env python3

from turtle import color
import rospy, cv2, cv_bridge
import numpy as np
import os

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveObjectToTag, QLearningReward, QMatrix
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import moveit_commander
import math


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"
path_prefix_q = os.path.dirname(__file__) + "/"

def find_mask(hsv, upper, lower):
    mask = cv2.inRange(hsv, lower, upper)
    return mask

class QAction(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_action")

        # Fetch pre-built action matrix. 
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
        
        #Load our Q-matrix. 
        self.q = np.genfromtxt(path_prefix_q + "q_matrix.csv", delimiter=',')
        #Initialize state to 0
        self.state = 0
        self.iteration = 1

        # Fetch actions.
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        # Fetch states. 
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # Initialize the object and the tag we need
        self.object = -1
        self.tag = -1
        # load DICT_4X4_50
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.taking_to_tag = False

        # Robot arm movement
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)
        # Global variable to control for distance to object
        self.robotpos = 0
                
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
                
        arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
        self.move_group_arm.go(arm_joint_goal)
        self.move_group_arm.stop()
        rospy.sleep(5)

        gripper_joint_goal = [0.01, -0.01]
        self.move_group_gripper.go(gripper_joint_goal)
        self.move_group_gripper.stop()
        rospy.sleep(2)

        # Setup publishers and subscribers
 
        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Publish robot movements
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.initialized = False
        self.color = False
        self.iterations_object = 0


    def choose_next_action(self):
        actions = [] # in the case when there are multiple actions with the same reward
        max_reward = max(self.q[self.state])
        for action, reward in enumerate(self.q[self.state]):
            if reward == max_reward:
                actions.append(action)
        next_action = np.random.choice(actions)
        
        new_state = self.action_matrix[self.state].tolist().index(next_action)
        self.state = new_state
        self.object = self.actions[next_action]["object"]
        self.tag = int(self.actions[next_action]["tag"])
        
        print(self.object)
        self.initialized = True
        return 
    
    def image_callback(self, msg):

        if (not self.initialized):
           
            return

        if (self.taking_to_tag): # When we have the dumbell and travelling to the tag
            

            # take the ROS message with the image and turn it into a format cv2 can use
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # turn the image into a grayscale
            grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)
            
          
            if self.robotpos == 1:
                my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.1))
                self.robot_movement_pub.publish(my_twist)
               
           
            if (ids is not None) and [self.tag] in ids:
                index_of_id = ids.tolist().index([self.tag])
                sum_x = 0
                sum_y = 0
                for i in range(4):
                    sum_x += corners[index_of_id][0][i][0]
                    sum_y += corners[index_of_id][0][i][1]
                cx = sum_x / 4
                cy = sum_y / 4

                if self.robotpos == 1: # Robot will move until it's close enough to the tag
                
                    my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0.001*(-cx + 160)))
                    self.robot_movement_pub.publish(my_twist)

                
            
        else:
            
            
            # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

           # print("Color is", self.object)

            lower_blue = np.array([90, 60, 60]) 
            upper_blue = np.array([90, 255, 255]) 

            lower_green = np.array([30, 60, 60]) 
            upper_green = np.array([45, 255, 255])

            lower_pink = np.array([120, 60, 60]) 
            upper_pink = np.array([170, 255, 255])

            # this erases all pixels that aren't the right color (the default as blue doesn't do anything except avoid an error when the color hasn't been initialized yet) 
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            
            if self.object == "pink":
                mask = cv2.inRange(hsv, lower_pink, upper_pink)
            elif self.object == "green":
                mask = cv2.inRange(hsv, lower_green, upper_green)
            else:
                mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # this limits our search scope to only view a slice of the image near the ground
            h, w, d = image.shape

            # using moments() function, the center of the pixels is determined
            M = cv2.moments(mask)
            
           
            # if there are no pixels found keep turning until we do find pixels of that color
            
            if M['m00'] > 0 and self.robotpos == 0:
                self.color = True
                # center of the pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

            
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                my_twist = Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0.002*(-cx + 160)))                                
                self.robot_movement_pub.publish(my_twist)
            elif self.robotpos == 0:
                my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0.05))                                
                self.robot_movement_pub.publish(my_twist)

                        
          


    def process_scan(self, data):

        if (not self.initialized):
           
            return

        if (self.taking_to_tag):# Taking to tag case
            for i in range (5):
                r = data.ranges[-i]
                l = data.ranges[i]
               
                if ((r <= 0.4 and r > 0.35) or (l <= 0.4 and l > 0.35)) and self.robotpos == 1: 
                   
                    self.robotpos = 2
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist) # stop
                    # Put the dumbell down
                
                
                    arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
                    self.move_group_arm.go(arm_joint_goal)
                    self.move_group_arm.stop()
                    rospy.sleep(7)

                    gripper_joint_goal = [0.01, -0.01]
                    self.move_group_gripper.go(gripper_joint_goal)
                    self.move_group_gripper.stop()
                    rospy.sleep(5)

                    arm_joint_goal = [0.0, math.radians(-20), 0.0, 0.0]
                    self.move_group_arm.go(arm_joint_goal)
                    self.move_group_arm.stop()
                    rospy.sleep(10)

                    #drive back and start turning
                    my_twist = Twist(linear=Vector3(-0.2, 0, 0), angular=Vector3(0, 0, 0.6))
                    self.robot_movement_pub.publish(my_twist)
                    
                    rospy.sleep(2)
               
                    my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)
                    rospy.sleep(10)
                    # Reset parameters and choose next action
                    self.robotpos = 0
                    self.initialized = False
                    self.choose_next_action()
                
                    self.taking_to_tag = False
   
                    
                    rospy.sleep(2)
        else: # When we're looking for dumbells 
            r = 0
            l = 0
        
            for i in range (10):
                r = data.ranges[-i]
                l = data.ranges[i]
              
                if ((r <= 0.22 and r > 0.2) or (l <= 0.22 and l >0.2)) and self.robotpos == 0 and self.taking_to_tag == False and self.color == True:
                    
                    self.robotpos = 1
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)

                    arm_joint_goal = [math.radians(min(r, l)), math.radians(20.0), 0.0, 0.0]
                    self.move_group_arm.go(arm_joint_goal)
                    self.move_group_arm.stop()
                    rospy.sleep(5)
                                
                    gripper_joint_goal = [-0.01, 0.01]
                    self.move_group_gripper.go(gripper_joint_goal, wait=True)
                    self.move_group_gripper.stop()

                    arm_joint_goal = [0.0, math.radians(-75), 0.0, 0.0]
                    self.move_group_arm.go(arm_joint_goal)
                    self.move_group_arm.stop()
                    rospy.sleep(10)

                    my_twist = Twist(linear=Vector3(-0.2, 0, 0), angular=Vector3(0, 0, 0.5))
                    self.robot_movement_pub.publish(my_twist)
                   
                    rospy.sleep(1)
                    my_twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)
                    rospy.sleep(5)

                    self.taking_to_tag = True
                    self.color = False
              

    
    def run(self):
        # Give time for all publishers to initialize
        rospy.sleep(3)
        # Choose the first action
        self.choose_next_action()
        # Give time to initialize
     
        # Keep the program running for 3 iterations
        rospy.spin()
   

if __name__ == "__main__":
    node = QAction()
    node.run()
