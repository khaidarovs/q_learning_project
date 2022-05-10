#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
# msg needed for /scan.
from sensor_msgs.msg import LaserScan
import moveit_commander
import math



class Follower:

        def __init__(self):
                
                # set up ROS / OpenCV bridge
                self.bridge = cv_bridge.CvBridge()

                # initalize the debugging window
                cv2.namedWindow("window", 1)
                self.robotpos = 0
                print(0)
                # subscribe to the robot's RGB camera data stream
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)
                rospy.Subscriber("/scan", LaserScan, self.process_scan)
                
                
                self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
                self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
                
                # wait=True ensures that the movement is synchronous
                arm_joint_goal = [0.0, 0.0, 0.0, 0.0]
                self.move_group_arm.go(arm_joint_goal)
                # Calling ``stop()`` ensures that there is no residual movement
                self.move_group_arm.stop()
                rospy.sleep(2)

                gripper_joint_goal = [0.0, 0.0]
                self.move_group_gripper.go(gripper_joint_goal)
                self.move_group_gripper.stop()
                rospy.sleep(2)
                

                # TODO: set up cmd_vel publisher 
                self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        def process_scan(self, data):
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

                   
                               
                                
                        


        def image_callback(self, msg):
                #print(1)
                my_twist = Twist(linear=Vector3(0.0, 0, 0))
                self.robot_movement_pub.publish(my_twist)
                # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                # TODO: define the upper and lower bounds for what should be considered 'yellow'
                # Note: see class page for hints on this

                lower_blue = numpy.array([90, 63, 60]) #TODO
                upper_blue = numpy.array([90, 255, 255]) #TODO

                

                
                # this erases all pixels that aren't blue
                mask = cv2.inRange(hsv, lower_blue, upper_blue)

                # this limits our search scope to only view a slice of the image near the ground
                h, w, d = image.shape
               # print(h, w, d)
            #    search_top = int(h/2)
            #    search_bot = int(3*h/4 + 20)
                #mask[0:1, 0:w] = 0
                #mask[h-1:h, 0:w] = 0

                # using moments() function, the center of the pixels is determined
                M = cv2.moments(mask)
               # print(M)
                # if there are any pixels found
                if M['m00'] > 0 and self.robotpos == 0:
                        # center of the pixels in the image
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

                        print(cx, cy)
                        # a red circle is visualized in the debugging window to indicate
                        # the center point of the pixels
                        # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
                        cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                       # print(2)
                        my_twist = Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0.001*(-cx + 160)))
                                                
                        self.robot_movement_pub.publish(my_twist)
                        

                        
                        # TODO: based on the location of the line (approximated
                        #       by the center of the pixels), implement
                        #       proportional control to have the robot follow
                        #       the line

                # shows the debugging window
                # hint: you might want to disable this once you're able to get a red circle in the debugging window
                cv2.imshow("window", image) 
                cv2.waitKey(3)

        


        def run(self):
                rospy.spin()
                
if __name__ == '__main__':

        rospy.init_node('line_follower')
        follower = Follower()
        follower.run()
