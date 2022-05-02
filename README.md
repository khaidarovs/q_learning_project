# q_learning_project

## Implementation plan

### Team members:
Sanzhar Khaidarov, Max Lederman

#### Q-learning algorithm:
##### Executing the Q-learning algorithm:
- We will implement the same Q-learning reinforcement learning algorithm that we went over in Class 9, that we will run using the phantom robot to train our Q-matrix. 
- To test if the Q-learning algorithm works, we will use the phantom robot to see each iteration, so that we can verify that at least the first steps give us the correct results that are in line with our calculations. 
##### Determining when the Q-matrix has converged:
- In order to determine when the Q matrix has converged we will look if the values are within a small enough range for a number of iterations, and if that's the case it means the matrix has converged. 
- By using the virtual reset world to iterate through our algorithm, we will ideally be able to quickly see that the results eventually stop fluctuating. If they widely vary even after a large number of iterations, then we will have to modify our algorithm.
##### Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward:
- Depending on the current state, the robot will choose the action with the highest Q-value in order to maximize the future reward. 
- By testing an individual state with pre-determined Q-values, we can see if the robot picks the correct action. We can then test this for longer sequences to verify that the robot continues to choose the Q-value that maximizes reward. 

#### Robot perception:
##### Determining the identities and locations of the three colored objects:
- In order to determine the identities and locations of the three colored objects we will utilize the robot's /scan and /camera/rgb/image_raw ROS topics in a similar way to what we did in the line follower lab. Depending on the RGB color we want, we will navigate the robot to an object of that color. 
- We will first test this by seeing if the robot can correctly identify a colored object by itself to see if it can even do basic identification. Once it succeeds at this, we can hardcode a pre-determined color for the robot to test if it picks it out of multiple different colored objects. 
##### Determining the identities and locations of the three AR tags:
- In order to determine the identities of the three AR tags we will use the aruco module in OpenCV. Once we have loaded the dictionary and ran detectMarkers, we will look at the order of the tags in the ids array and determine the location and the identity of the tag from there. 
- We will use the same test process as with the colored objects, except that this time it will be with the AR tags. 

#### Robot manipulation & movement:
##### Picking up and putting down the colored objects with the OpenMANIPULATOR arm:
- After locating which object to pick up, we can use the MoveIt ROS package to lower the arm and close around the object by adjusting the joint angles and gripper. From there we can use the same process to rain the arm and then lower it again to put the object down.
- We will individually test if the robot can pick up and put down an object properly outside of the Q-learning element before implementing it into the rest of the project.
##### Navigating to the appropriate locations to pick up and put down the colored objects:
- To navigate to the right locations, we will use the same concept as in our wall follower code in the warmup project, except this time it specifically moves towards the intended object or tag based off of the results of the robot perception. 
- We will test this by making it move towards a pre-determined location with the same formatting as what would be given either for the objects or the tags. Once this works, we can see test that the robot moves properly in tandem with the results from the robot perception.
### Timeline

- Lab E: Start the writeup and Q-matrix implementation
- Weekend before class 11: Finish and test training.launch file and store a converged Q-matrix in a .csv file
- Lab F: Robot arm programming
- Before Class 12: Write the action part of the code
- Weekend before it's due: Final tests and corrections

## Writeup

### Objectives description

### High-level description

### Q-learning algorithm description

#### Selecting and executing actions for the robot (or phantom robot) to take

#### Updating the Q-matrix

#### Determining when to stop iterating through the Q-learning algorithm
