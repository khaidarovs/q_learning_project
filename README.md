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

The objective of this project was to take a predetermined final state that is unknown to us that grants a reward and have our Q-learning algorithm continuously loop through until it converges upon a Q-matrix that provides a reward path for the robot to reach the final goal. Once the Q-matrix has been correctly achieved, we then need to translate it to an action sequence for the robot to take to properly move the objects to the correct AR codes after having identified these. 

### High-level description

Based off a fixed number of iterations without change in updated reward values (or after a maximum number of total itierations), we loop our Q-learning algorithm. Within this algorithm, we begin by randomly choosing a valid action within our state that brings us to our next state, and then using the current and next state, update our Q-matrix using the equation given to us, with 𝛂 at 1 and 𝛾 at 0.5. After this, we update our iteration value depending on if the Q-matrix changes, and if we have reach a final state (the algorithm has looped 3 times in a row), then we reset the state to 0. Once we have our Q-matrix, we launch our Q-action file, where we randomly choose one of the valid nex states offered. Once we choose our next state, our robot turns until it recognizes the correct object, then goes to it, picks it up, before turning until it find the correct AR tag, where it drops off the object. Then, the action matrix once again chooses a random valid next state and restarts the process.

### Q-learning algorithm description

#### Selecting and executing actions for the robot (or phantom robot) to take
-	Location: inside the update_q_matrix function
-	Description: In order to determine what action the robot should select, we first create a list of valid actions for the current state of the robot by traversing through the action_matrix and selecting the valid state transitions and corresponding actions for that specific state. We then randomly choose one of the valid actions. 

#### Updating the Q-matrix
-	Location: inside the update_q_matrix function
-	Description: Once we have selected and executed the action, we publish it using the corresponding publisher, receive the reward and then update the q-matrix according to the formula covered in class. We set alpha equal to 1 and gamma equal to 0.5. We then update the state and repeat the process for that state. 

#### Determining when to stop iterating through the Q-learning algorithm
-	Location: inside the update_q_matrix function
-	Description: There are two ways in which we would decide when to stop iterating through the algorithm. Firstly, the variable iterations measures the number of iterations since the matrix was last updated. Through trial and error we found that 75 is a suitable number of such iterations to make sure that the matrix has converged. The second way is the count variable which measures the total number of iterations through the algorithm. We keep iterating until count reaches 1000, which we also decided would be a suitable cap on the number of iterations (through a series of experiments) to make sure that the matrix has converged. Therefore, if there are constantly less than 75 iterations without an update, we will stop iterating after 1000 iterations.


### Robot perception description

#### Identifying the locations and identities of each of the colored objects
- Location: else portion of both process_scan and image_callback
- Description: To identify and go to the colored object, we addapted the code used in Lab B: Line follower, using the HSV documentation to supplement our color finding. Taking into account the entire screen formed instead of just the floor, our robot would identify the center of all the pixels of the given color so as to place the assumed position of the object in regards to the robot. We would only use the scanners to identify if there is something directly in front of the robot, within the grabber's reach, which should only be the case if it reached the correct object.

#### Identifying the locations and identities of each of the AR tags
- Location: if (self.taking_to_tag) portion of both process_scan and image_callback
- Description:

### Robot manipulation and movement

#### Moving to the right spot in order to pick up a colored object
- Location: else portion of both process_scan and image_callback
- Description: Our robot will continuously turn. Once it identifies the colors of the given object, it will then turn based off the cx coordinate of the center of all the pixels of that color, while simultaneously driving forward at a constant pace. Simultaneously, the scanner will verify whether the object is within the grabber's reach, and if it is, the robot will stop (both moving and looking for colors). 

#### Picking up the colored object
- Location: else portion of both process_scan
- Description: As mentioned above, the robot stops once the scanner detects the object is right in front of it. Once that happens, the arm immediately descends to grab the the midportion of the object, when the gripper closes, before the arm brings the object parallel, over the robot. There are significant usage of rospy.sleep to make sure the arm finishes its movements properly before going to the next movement. 

#### Moving to the desired destination (AR tag) with the colored object

#### Putting the colored object back down at the desired destination


### Challenges
Although we didn't face many challenges comparitavely in the q_learning section, q_action proved to be incredibly buggy for us. For starters, we use a lot of different checking methods to verify which state of the code we're in (whether we are searching for the tag or found the object, etc) which got confusing and was complicated to implement. This portion was incredibly lengthy to solve because our robot would keep going into a state when it wasn't supposed to (the primary issue was after dropping the object, the robot would transition to finding the new object but then would immediately go to move its arm, which took us hours to deal with). In addition, for reasons that we still are not sure of, we would continuously have issues where we would run our code, it would work, and then after making no changes, it would break if we ran it again, which would then take us a solid hour to get back to the previous state we were in.   

### Future work
Our state transitions were incredibly bulky, and so if we had more time, I think we would have entirely overhauled our code to use our robot's positional understanding. Instead of our robot turning to find objects and AR tags, we would have our movements be more similar to the demo, where the robot between states would always move to the middle before turning to face whatever direction it is meant to go to next. Since this problem was so overwhelming, and entirely getting rid of it would have meant effectively rewriting a large portion of our action file, I think this is really all we would do if we had even more time. 

### Takeaways
- Transitionning between states, although incredibly difficult, is something that we both now have a much better understanding of now. In future projects, we should be able to more efficiently use flags to do these transitions, allowing our robot to do a better job of properly multitasking.
- The other important thing we learned was how to properly use message files to publish and receive information between different nodes. This was useful when moving from our Q-matrix to choosing actions, and which should prove to be a good foundation for our final project, where we will have the robot play tic-tac-toe. 


## Videos


![Alt Text](https://github.com/khaidarovs/q_learning_project/blob/main/q1.gif)

![Alt Text](https://github.com/khaidarovs/q_learning_project/blob/main/q2.gif)

![Alt Text](https://github.com/khaidarovs/q_learning_project/blob/main/q3.gif)

![Alt Text](https://github.com/khaidarovs/q_learning_project/blob/main/q4.gif)

![Alt Text](https://github.com/khaidarovs/q_learning_project/blob/main/q5.gif)

![Alt Text](https://github.com/khaidarovs/q_learning_project/blob/main/q6.gif)
