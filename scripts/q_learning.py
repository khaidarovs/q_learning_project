#!/usr/bin/env python3

import rospy
import numpy as np
import os
import pandas as pd

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
        self.reward_rcv = False

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
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.reward_received)

        # publish the action 
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size=10)


    
    def update_q_matrix(self):
        iterations = 0
        count = 0
        # changed = False
        state = 0
        while iterations < 100 or count < 1000:
            self.reward_rcv = False
            if count % 3 == 0:
                state = 0
            # print("State is", state)
            actions = []
            for action in self.action_matrix[state]:
                if action != -1:
                    actions.append(action)
            action = int(np.random.choice(actions))
            # print("Action is", action)
            count += 1
            new_state = self.action_matrix[state].tolist().index(action)
            # Publish the action we selected
            my_action = RobotMoveObjectToTag(robot_object = self.actions[action]["object"], tag_id = self.actions[action]["tag"])
            # print("Object is", self.actions[action]["object"])
            # print("tag is", self.actions[action]["tag"])
            self.action_pub.publish(my_action)
            r = rospy.Rate(5)
            while not self.reward_rcv:
                r.sleep()
            old_val = self.q[state][action]
            # print("reward is", self.reward)
            self.q[state][action] = old_val + 1 * (self.reward + 0.5 * max(self.q[new_state]) - self.q[state][action])
            # print("old val is", old_val)
            if self.reward == 100:
                print("new val:", self.q[state][action])
            if self.q[state][action] == old_val:
                # changed = False
                iterations += 1
            else:
                # changed = True
                iterations = 0
            state = new_state
        print(self.q)
        return 

    def reward_received(self, data):
        # print("data reward is", data.reward)
        self.reward = data.reward
        self.reward_rcv = True
        return

    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        df = pd.DataFrame(self.q)
        print("cwd is", os.getcwd())
        print("df", df)
        with open("q_matrix.csv", 'w') as csv_file: #saves it correctly but doesn't save in the correct directory
            df.to_csv(path_or_buf = csv_file)
        print("done")
        # pd.DataFrame(self.q).to_csv(os.getcwd() + "q_matrix.csv")
        return
    
    def run(self):
        # Keep the program alive.
        rospy.sleep(3)
        self.update_q_matrix()
        self.save_q_matrix()

if __name__ == "__main__":
    node = QLearning()
    node.run()
