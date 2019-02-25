#! /usr/bin/env python

import rospy
import actionlib
import geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations
import time
from park.py import Park

class MainNavigation:
    # We create some constants with the corresponing vaules from the SimpleGoalState class
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

    def __init__(self):
        #Initializes node
        rospy.init_node('map_navigation')
        choice = choose()
        if choice == 'q':
            rospy.loginfo("No navigation goal was set, restart the node.")
        else:
            start_navigation(choice)

    #function to choose the navigation goal
    def choose(self):
      choice='q'

      rospy.loginfo("|-------------------------------|")
      rospy.loginfo("|PRESSE A KEY:")
      rospy.loginfo("|'0': Location 1 ")
      rospy.loginfo("|'1': Location 2")
      rospy.loginfo("|'2': Location 3 ")
      rospy.loginfo("|'3': Location 4 ")
      rospy.loginfo("|'q': Quit ")
      rospy.loginfo("|-------------------------------|")
      rospy.loginfo("|WHERE TO GO?")
      choice = input()
      return choice

    # definition of the feedback callback. This will be called when feedback
    # is received from the action server
    # it just prints a message indicating a new message has been received
    def feedback_callback(feedback):
        current_theta = 0.0
        current_x = feedback.pose.position.x
        current_y = feedback.pose.position.y
        c_or = feedback.pose.orientation
        (roll,pitch,theta) = euler_from_quaternion([c_or.x, c_or.y, c_or.z, c_or.w])
        print('feedback received => x = %c, y = %d, theta = %e', current_x, current_y, current_theta)

    def start_navigation(self, choice):
        # Creates the SimpleActionClient
        action_server_name = '/move_base'
        move_base_client = actionlib.SimpleActionClient(action_server_name, MoveBaseAction)

        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo('Waiting for action Server ' + action_server_name)
        client.wait_for_server()
        rospy.loginfo('Action Server Found...' + action_server_name)

        # declare the coordinates of interest
        x0 = 15.50
        y0 = 10.20
        theta0 = 0.0
        x1 = 27.70
        y1 = 12.50
        theta1 = 0.0
        x2 = 30.44
        y2 = 12.50
        theta2 = 0.0
        x3 = 35.20
        y3 = 13.50
        theta3 = 0.0

        #initializes variables for goal pose
        x = 0.0
        y = 0.0
        theta = 0.0

        #based on choice made, sets up variables for goal pose
        if (choice == 0):
          x = x0
          y = y0
          theta = theta0
        elif (choice == 1):
          x = x1
          y = y1
          theta = theta1
        elif (choice == 2):
          x = x1
          y = y1
          theta = theta1
        elif (choice == 3):
          x = x1
          y = y1
          theta = theta1

        # Creates a goal to send to the action server.
        pose = geometry_msgs.msg.Pose()
        pose.position.x = x
        pose.position.y = y
        theta = 0.0  #in radians
        q = quaternion_from_euler(0, 0, theta)
        pose.orientation = geometry_msgs.msg.Quaternion(*q)
        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        # Sends the goal to the action server.
        rospy.loginfo('Sending goal to action server: %s', goal)
        try:
            move_base_client.send_goal(goal, feedback_cb=feedback_callback)
        expect rospy.ROSInterruptException:
            print "program interrupted before completion"

        # You can access the SimpleAction Variable "simple_state", that will be 1 if active, and 2 when finished.
        #Its a variable, better use a function like get_state.
        #state = client.simple_state
        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = client.get_state()

        rate = rospy.Rate(1)

        rospy.loginfo("state_result: "+str(state_result))

        while state_result < DONE:
            rospy.loginfo("Checking for alien while performing navigation....")
            #here code for checking for alien robot
            #when robot sees the alien robot, cancel navigation goal
            parking = Park()
            move_base_client.cancel_goal()
            rate.sleep()
            state_result = client.get_state()
            rospy.loginfo("state_result: "+str(state_result))

        rospy.loginfo("[Result] State: "+str(state_result))
        if state_result == ERROR:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == WARN:
            rospy.logwarn("There is a warning in the Server Side")
        else:
          # Waits for the server to finish performing the action.
          print 'Waiting for result...'
          move_base_client.wait_for_result()

        return move_base_client.get_result()

if __name__ == '__main__':
    try:
        MainNavigation()
    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
