# q_learning_project

## Implementation plan

### Team members:
Sanzhar Khaidarov, Max Lederman

#### Q-learning algorithm:
##### Executing the Q-learning algorithm:
- We will implement the same Q-learning reinforcement learning algorithm that we went over in Class 9, that we will run using the phantom robot to train our Q-matrix. 
##### Determining when the Q-matrix has converged:
- In order to determine when the Q matrix has converged we will look if the values are within a small enough range for a number of iterations, and if that's the case it means the matrix has converged. 
##### Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward:
- Depending on the current state, the robot will choose the action with the highest Q-value in order to maximize the future reward. 

#### Robot perception:
##### Determining the identities and locations of the three colored objects:
- In order to determine the identities and locations of the three colored objects we will utilize the robot's /scan and /camera/rgb/image_raw ROS topics in a similar way to what we did in the line follower lab. Depending on the RGB color we want, we will navigate the robot to an object of that color. 
##### Determining the identities and locations of the three AR tags:
- In order to determine the identities of the three AR tags we will use the aruco module in OpenCV. Once we have loaded the dictionary and ran detectMarkers, we will look at the order of the tags in the ids array and determine the location and the identity of the tag from there. 

#### Robot manipulation & movement:
##### Picking up and putting down the colored objects with the OpenMANIPULATOR arm:
- ...
##### Navigating to the appropriate locations to pick up and put down the colored objects:
- ...

### Timeline

- Lab E: Start the writeup and Q-matrix implementation
- Weekend before class 11: Finish and test training.launch file and store a converged Q-matrix in a .csv file
- Lab F: Robot arm programming
- Before Class 12: Write the action part of the code
- Weekend before it's due: Final tests and corrections

