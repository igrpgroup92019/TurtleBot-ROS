#! /usr/bin/env python

import rospy
import actionlib
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
import tf.transformations
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import tan, cos, sin, pi, sqrt, pow, fabs
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry

class MainNavigation:

    #x y and theta coordinates for goals
    x = 0.0
    y = 0.0
    theta = 0.0

    current_x = 0.0
    current_y = 0.0
    current_theta = 0.0

    def __init__(self):
        #Initializes node
        rospy.init_node('map_navigation')
        sub_goal = rospy.Subscriber('/rtabmap/goal_out', PoseStamped, self.goal_listener)
        self.publisher = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        sub_laser = rospy.Subscriber('/scan', LaserScan, self.laser_listener)
        self.pub_marker = rospy.Publisher('rviz_marker', Marker, queue_size=1)
        sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_listener)

        self.goal_received = False
        self.alien_detected = False

        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0

        # Creates the SimpleActionClient
        self.action_server_name = '/move_base'
        self.move_base_client = actionlib.SimpleActionClient(self.action_server_name, MoveBaseAction)

        self.start_navigation()

    # definition of the feedback callback. This will be called when feedback
    # is received from the action server
    # it prints the current location of the robot based on the x,y and theta coordinates
    def feedback_callback(self, feedback):
        global current_x
        global current_y
        global current_theta

        current_x = feedback.base_position.pose.position.x
        current_y = feedback.base_position.pose.position.y
        c_or = feedback.base_position.pose.orientation
        (roll,pitch,current_theta) = euler_from_quaternion([c_or.x, c_or.y, c_or.z, c_or.w])
        rospy.loginfo('feedback received => Current Robot Position: x = %f, y = %f, theta = %f', current_x, current_y, current_theta)

    def laser_listener(self, d):
        if d.ranges[len(d.ranges)/2] < 0.1:
            self.alien_detected = True

    def goal_listener(self, coordinates):
        global x, y, theta
        x = coordinates.pose.position.x
        y = coordinates.pose.position.y
        orient = coordinates.pose.orientation
        (roll,pitch,theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.goal_received = True
        print "Goal received"

    def odom_listener(self, data):
        self.x_curr = data.pose.pose.position.x
        self.y_curr = data.pose.pose.position.y
        orient = data.pose.pose.orientation
        (roll,pitch,self.theta_curr) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])

    def start_navigation(self):
        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo('Waiting for action Server ' + self.action_server_name)
        self.move_base_client.wait_for_server()
        rospy.loginfo('Action Server Found...' + self.action_server_name)
        while not rospy.is_shutdown():
            if self.goal_received == True:
                self.send_goal(x, y, theta)
                rospy.loginfo("Sending goal to main algorithm -- x:%f, y:%f, theta:%f",x,y,theta)


    def send_goal(self, x, y, theta):
        #current navigation goal coordinates
        curr_nav_x = x
        curr_nav_y = y
        curr_nav_theta = theta

        self.goal_received = False

        # Creates a goal to send to the action server.
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        pose.orientation = geometry_msgs.msg.Quaternion(*q)
        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        #Creates an rviz marker for the goal
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = rospy.Time.now()
        robot_marker.action = Marker.ADD
        robot_marker.scale.x = 0.9
        robot_marker.scale.y = 0.05
        robot_marker.scale.z = 0.1
        robot_marker.pose = pose
        robot_marker.color.a = 1.0
        robot_marker.color.b = 0.0
        robot_marker.type = Marker.ARROW
        robot_marker.color.r = 1.0
        robot_marker.color.g = 0.0

        # Sends the goal to the action server.
        rospy.loginfo('Sending goal to action server: %s', goal)
        self.move_base_client.send_goal(goal, feedback_cb=self.feedback_callback)

        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = self.move_base_client.get_state()

        rate = rospy.Rate(10)

        rospy.loginfo("state_result: "+str(state_result))

        # We create some constants with the corresponing vaules from the SimpleGoalState class
        PENDING = 0
        ACTIVE = 1
        DONE = 2
        WARN = 3
        ERROR = 4

        while state_result < DONE:
            rospy.loginfo("Checking for alien while performing navigation....")

            #check if a goal is being published and it is not the same as the current one, then cancel navigation and start new one
            if self.goal_received == True and curr_nav_x != x and curr_nav_y != y:
                self.move_base_client.cancel_goal()
                print "Cancelling goal..."
                self.marker_goal = False
                self.send_goal(x,y,theta)
                rospy.loginfo("Sending goal to main algorithm -- x:%f, y:%f, theta:%f",x,y,theta)

            #publish rviz marker for goal
            self.pub_marker.publish(robot_marker)

            rate.sleep()
            state_result = self.move_base_client.get_state()
            rospy.loginfo("state_result: "+str(state_result))

        rospy.loginfo("[Result] State: "+str(state_result))
        if state_result == ERROR:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == WARN:
            rospy.logwarn("There is a warning in the Server Side")
        else:
          # Waits for the server to finish performing the action.
          print 'Waiting for result...'
          self.move_base_client.wait_for_result()
        rospy.loginfo("Goal Reached! Success!")


        move = Twist()
        distance_travelled = 0.0
        while not rospy.is_shutdown():
            initial_x = 0.0
            initial_y = 0.0
            a = "Dock"
            b = "undock"
            c = "continue"
            print "What do you want to do?"
            print "A) Dock"
            print "B) Undock"
            print "C) Set new navigation goal"
            result = input()
            self.alien_detected = False

            if result=="dock" or result=="Dock" or result=="A" or result=="a":
                initial_x = self.x_curr
                initial_y = self.y_curr
                move.linear.x = 0.3
                while self.alien_detected == False:
                    print self.alien_detected
                    print "one"
                    self.publisher.publish(move)
                    rate.sleep()
                print "two"
                move.linear.x = 0
                distance_travelled = sqrt(pow(self.x_curr - initial_x, 2) + pow(self.y_curr - initial_y, 2))
                while not rospy.is_shutdown():
                    connections = self.publisher.get_num_connections()
                    if connections > 0:
                        self.publisher.publish(move)
                        break
                    else:
                        rate.sleep()
            elif result=="undock" or result=="Undock" or result=="B" or result=="b":
                initial_theta = self.theta_curr
                move.angular.z = 1.0
                difference = 0.0
                while difference < pi:
                    print "enter"
                    if initial_theta > 0 and self.theta_curr <= 0:
                        self.theta_curr = self.theta_curr + 2*pi
                    difference = fabs(self.theta_curr - initial_theta)
                    print initial_theta*180/pi
                    print self.theta_curr*180/pi
                    print difference*180/pi
                    if difference > 2.97:
                        move.angular.z = 0.03
                    self.publisher.publish(move)
                    rate.sleep()
                move.angular.z = 0.0
                while not rospy.is_shutdown():
                    connections = self.publisher.get_num_connections()
                    if connections > 0:
                        self.publisher.publish(move)
                        break
                    else:
                        rate.sleep()
                move.linear.x = 0.3
                initial_x = self.x_curr
                initial_y = self.y_curr
                distance_back = 0.0
                while distance_back < distance_travelled:
                    print distance_back
                    print distance_travelled
                    distance_back = sqrt(pow(self.x_curr - initial_x, 2) + pow(self.y_curr - initial_y, 2))
                    self.publisher.publish(move)
                    rate.sleep()
                move.linear.x = 0
                while not rospy.is_shutdown():
                    connections = self.publisher.get_num_connections()
                    if connections > 0:
                        self.publisher.publish(move)
                        break
                    else:
                        rate.sleep()
            elif result=="continue" or result=="Continue" or result=="C" or result=="c":
                break
            else:
                print "Wrong command"

        return self.move_base_client.get_result()


if __name__ == '__main__':
    MainNavigation()
