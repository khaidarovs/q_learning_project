#!/usr/bin/env python3

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

class QAction(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_action")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")
        
        #Initialize our Q-matrix as a series of zeros over 64 rows for each state
        #and 9 columns for each possible action
        self.q = np.genfromtxt(path_prefix_q + "q_matrix.csv")
        self.state = 0

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # Initialize the object and the tag we need
        self.object = -1
        self.tag = 1
        # load DICT_4X4_50
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.taking_to_tag = True

        # Robot arm movement
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)
        self.robotpos = 0
        print(0)
        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)
                
                
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
                
        arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
        self.move_group_arm.go(arm_joint_goal)
        self.move_group_arm.stop()
        rospy.sleep(2)

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

        # self.seen_first_image = False

        self.initalized = True


    def choose_next_action(self):
        next_action = self.q[self.state].tolist().index(max(self.q[self.state]))
        new_state = self.action_matrix[self.state].tolist().index(next_action)
        self.state = new_state
        # Publish the action 
        # my_action = RobotMoveObjectToTag(robot_object = self.actions[next_action]["object"], tag_id = self.actions[next_action]["tag"])
        # self.action_pub.publish(my_action)
        self.object = self.actions[next_action]["object"]
        self.tag = int(self.actions[next_action]["tag"])
        print("tag is initialized and it is", self.tag)
        return 
    
    def image_callback(self, msg):


        

        if (self.taking_to_tag):

            self.seen_first_image = True

            # take the ROS message with the image and turn it into a format cv2 can use
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # turn the image into a grayscale
            grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

            index_of_id = ids.tolist().index([self.tag])
            sum_x = 0
            sum_y = 0
            for i in range(4):
                sum_x += corners[index_of_id][0][i][0]
                sum_y += corners[index_of_id][0][i][1]
            cx = sum_x / 4
            cy = sum_y / 4

            if self.robotpos == 0: # will stop when close to the tag
                my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0.001*(-cx + 160)))
                self.robot_movement_pub.publish(my_twist)
            
            cv2.imshow("window", image) 
            cv2.waitKey(3)
        else:
            my_twist = Twist(linear=Vector3(0.0, 0, 0))
            self.robot_movement_pub.publish(my_twist)
            # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            lower_blue = np.array([90, 63, 60]) #TODO
            upper_blue = np.array([90, 255, 255]) #TODO

            # this erases all pixels that aren't blue
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

            # this limits our search scope to only view a slice of the image near the ground
            h, w, d = image.shape

            # using moments() function, the center of the pixels is determined
            M = cv2.moments(mask)
            # print(M)
            # if there are any pixels found
            if M['m00'] > 0 and self.robotpos == 0:
                # center of the pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                print(cx, cy)
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0.001*(-cx + 160)))                                
                self.robot_movement_pub.publish(my_twist)
                        
            # shows the debugging window
            # hint: you might want to disable this once you're able to get a red circle in the debugging window
            
            cv2.imshow("window", image) 
            cv2.waitKey(3)


    def process_scan(self, data):
        if (self.taking_to_tag):
            for i in range (20):
                r = data.ranges[-i]
                l = data.ranges[i]
                if ((r <= 0.22 and r > 0.1) or (l <= 0.22 and l >0.1)) and self.robotpos==0:
                    print("ready")
                    self.robotpos = 1
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)
                    # put the dumbell down and turn 180 degrees or turn until we see the color we want
                    self.robotpos = 0
                    self.taking_to_tag = False
        else:
            for i in range (20):
                r = data.ranges[-i]
                l = data.ranges[i]
                if ((r <= 0.22 and r > 0.1) or (l <= 0.22 and l >0.1)) and self.robotpos==0:
                    print("ready")
                    self.robotpos = 1
                    my_twist = Twist(linear=Vector3(0.0, 0, 0), angular=Vector3(0, 0, 0))
                    self.robot_movement_pub.publish(my_twist)

                    arm_joint_goal = [math.radians(min(r, l)), math.radians(20.0), 0.0, 0.0]
                    self.move_group_arm.go(arm_joint_goal)
                    self.move_group_arm.stop()
                    rospy.sleep(2)
                                
                    gripper_joint_goal = [-0.01, 0.01]
                    self.move_group_gripper.go(gripper_joint_goal, wait=True)
                    self.move_group_gripper.stop()

                    arm_joint_goal = [0.0, math.radians(-75), 0.0, 0.0]
                    self.move_group_arm.go(arm_joint_goal)
                    self.move_group_arm.stop()

                    taking_to_tag = True
                    # turn around 180 degrees or until we see the tag

    
    def run(self):
        # Give time for all publishers to initialize
        rospy.sleep(3)
        # self.choose_next_action()
        self.choose_next_action()
        rospy.spin()


# class ExecuteAction(object):
#     def __init__(self):
#         # Initialize this node
#         rospy.init_node("execute_action")

#         # Initialize the object and the tag we need
#         self.object = NULL
#         self.tag = NULL
#         # load DICT_4X4_50
#         self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

#         # Setup publishers and subscribers

#         # subscribe to the action topic
#         rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.action_received)

#         # subscribe to the robot's RGB camera data stream
#         self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
#                 Image, self.image_callback)

#         # Publish robot movements
#         self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#         self.seen_first_image = False

#         self.initalized = True

#     def action_received(self, data):
#         self.object = data.robot_object
#         self.tag = int(data.tag_id)
#         return

#     def image_callback(self, msg):

#         if (not self.initalized):
#             return
        
#         if (not self.seen_first_image):

#             self.seen_first_image = True

#             # take the ROS message with the image and turn it into a format cv2 can use
#             img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

#             # turn the image into a grayscale
#             grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#             # search for tags from DICT_4X4_50 in a GRAYSCALE image
#             corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, aruco_dict)

#             index_of_id = ids.tolist().index([self.tag])
#             sum_x = 0
#             sum_y = 0
#             for i in range(4):
#                 sum_x += corners[index_of_id][0][i][0]
#                 sum_y += corners[index_of_id][0][i][1]
#             cx = sum_x / 4
#             cy = sum_y / 4

#             my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0.001*(-cx + 160)))
#             self.robot_movement_pub.publish(my_twist)
     
#     def run(self):
#         # Give time for all publishers to initialize
#         rospy.spin()

if __name__ == "__main__":
    node = QAction()
    node.run()
    #node2.run()