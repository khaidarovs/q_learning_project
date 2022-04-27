#!/usr/bin/env python3

import rospy
import numpy as np
import os

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from q_learning_project.msg import RobotMoveObjectToTag, QLearningReward, QMatrix

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

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
        self.q = np.loadtxt(path_prefix + "init_q.txt")
        self.reward = 0

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

        # publish the current Q-matrix
        self.q_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)

        # subscribe to the reward topic
        rospy.Subscriber("/q_learning/q_matrix", QLearningReward, self.reward_received)

        # publish the action 
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)

    def update_q_matrix(self):
        q_copy = self.q
        iterations = 0
        error = 0.001
        state = 0
        while iterations < 10:
            actions = []
            for action in self.action_matrix[state]:
                if action != -1:
                    actions.append(action)
            action = np.random.choice(actions)
            state = self.action_matrix[state].index(action)
            # get the reward 
            for row in self.q:
                for col in self.q:

            #update the matrix
            if diff < error:
                iterations += 1
        return 

    def reward_received(self):



    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        return

if __name__ == "__main__":
    node = QLearning()
