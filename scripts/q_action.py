#!/usr/bin/env python3

from asyncio.windows_events import NULL
import rospy, cv2, cv_bridge
import numpy as np
import os
import pandas as pd

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveObjectToTag, QLearningReward, QMatrix
from std_msgs.msg import Header
from sensor_msgs.msg import Image

import moveit_commander
import math
# import the custom message
from class_meeting_08_kinematics.msg import Traffic

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

        # Setup publishers and subscribers

        # publish the action 
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)


    def choose_next_action(self):
        next_action = self.q[self.state].tolist().index(max(self.q[self.state]))
        new_state = self.action_matrix[self.state].tolist().index(next_action)
        self.state = new_state
        # Publish the action 
        my_action = RobotMoveObjectToTag(robot_object = self.actions[next_action]["object"], tag_id = self.actions[next_action]["tag"])
        self.action_pub.publish(my_action)
        return 
    
    def run(self):
        # Give time for all publishers to initialize
        rospy.sleep(3)
        self.choose_next_action()


class ExecuteAction(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("execute_action")

        # Initialize the object and the tag we need
        self.object = NULL
        self.tag = NULL

        # Setup publishers and subscribers

        # subscribe to the action topic
        rospy.Subscriber("/q_learning/robot_action", RobotMoveObjectToTag, self.action_received)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)

        # Publish robot movements
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def action_received(self, data):
        self.object = data.robot_object
        self.tag = data.tag_id
        return

    def image_callback(self, msg):

        # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # TODO: define the upper and lower bounds for what should be considered 'yellow'
        # Note: see class page for hints on this

        lower_yellow = numpy.array([14.9, 0, 50]) #TODO
        upper_yellow = numpy.array([14.9, 255, 255]) #TODO

        # orange_min = numpy.uint8([[[0,102,204 ]]])
        # orange_max = numpy.uint8([[[153,204,255 ]]])
        # hsv_orange_min = cv2.cvtColor(orange_min,cv2.COLOR_BGR2HSV)
        # hsv_orange_max = cv2.cvtColor(orange_max,cv2.COLOR_BGR2HSV)
        
        # this erases all pixels that aren't yellow
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # this limits our search scope to only view a slice of the image near the ground
        h, w, d = image.shape
        search_top = int(3*h/4)
        search_bot = int(3*h/4 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)
        # if there are any yellow pixels found
        if M['m00'] > 0:
                # center of the yellow pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                print(cx, cy)
                # a red circle is visualized in the debugging window to indicate
                # the center point of the yellow pixels
                # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                my_twist = Twist(linear=Vector3(0.05, 0, 0), angular=Vector3(0, 0, 0.001*(-cx + 160)))
                self.robot_movement_pub.publish(my_twist)
                # TODO: based on the location of the line (approximated
                #       by the center of the yellow pixels), implement
                #       proportional control to have the robot follow
                #       the yellow line

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        cv2.imshow("window", image) 
        cv2.waitKey(3)
    
    def run(self):
        # Give time for all publishers to initialize
        rospy.spin()

if __name__ == "__main__":
    node1 = QAction()
    node2 = ExecuteAction()
    node1.run()
    node2.run()