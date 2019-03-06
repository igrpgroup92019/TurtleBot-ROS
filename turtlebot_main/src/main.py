#! /usr/bin/env python

import rospy
import actionlib
import geometry_msgs
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

class MainNavigation:
    #x y and theta coordinates for goals
    x = 0.0
    y = 0.0
    theta = 0.0

    #x y and theta coordinates for marker
    m_x = 0.0
    m_y = 0.0
    m_theta = 0.0

    def __init__(self):
        #Initializes node
        rospy.init_node('map_navigation')
        sub_goal = rospy.Subscriber('/rtabmap/goal_out', PoseStamped, self.goal_listener)
        sub_marker = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.marker_listener)

        self.goal_received = False
        self.marker_detected = False

        # Creates the SimpleActionClient
        self.action_server_name = '/move_base'
        self.move_base_client = actionlib.SimpleActionClient(self.action_server_name, MoveBaseAction)

        self.start_navigation()

    # definition of the feedback callback. This will be called when feedback
    # is received from the action server
    # it prints the current location of the robot based on the x,y and theta coordinates
    def feedback_callback(self, feedback):
        current_theta = 0.0
        current_x = feedback.base_position.pose.position.x
        current_y = feedback.base_position.pose.position.y
        c_or = feedback.base_position.pose.orientation
        (roll,pitch,current_theta) = euler_from_quaternion([c_or.x, c_or.y, c_or.z, c_or.w])
        print('feedback received => Current Robot Position: x = %d, y = %d, theta = %d', current_x, current_y, current_theta)

    def goal_listener(self, coordinates):
        global x
        global y
        global theta
        x = coordinates.pose.position.x
        y = coordinates.pose.position.y
        orient = coordinates.pose.orientation
        (roll,pitch,theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.goal_received = True
        print "Goal received"

    def marker_listener(self, marker_pose):
        global m_x
        global m_y
        global m_theta
        m_x = marker_pose.pose.position.x
        m_y = marker_pose.pose.position.y
        m_orient = marker_pose.pose.orientation
        (roll,pitch,m_theta) = euler_from_quaternion([m_orient.x, m_orient.y, m_orient.z, m_orient.w])
        self.marker_detected = True

    def start_navigation(self):
        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo('Waiting for action Server ' + self.action_server_name)
        self.move_base_client.wait_for_server()
        rospy.loginfo('Action Server Found...' + self.action_server_name)
        while not rospy.is_shutdown():
            if self.goal_received == True:
                self.send_goal(x, y, theta)
                rospy.loginfo("Sending goal to main algorithm -- x:%d, y:%d, theta:%d",x,y,theta)


    def send_goal(self, x, y, theta):
        self.goal_received = False
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
        self.move_base_client.send_goal(goal, feedback_cb=self.feedback_callback)

        # state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        state_result = self.move_base_client.get_state()

        rate = rospy.Rate(1)

        rospy.loginfo("state_result: "+str(state_result))

        # We create some constants with the corresponing vaules from the SimpleGoalState class
        PENDING = 0
        ACTIVE = 1
        DONE = 2
        WARN = 3
        ERROR = 4

        while state_result < DONE:
            rospy.loginfo("Checking for alien while performing navigation....")
            if self.marker_detected == True:
                self.move_base_client.cancel_goal()
                self.marker_detected = False
            #here algorithm for goal recalculation
            #send new goal send_goal(new_x,new_y,new_theta)
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
        return self.move_base_client.get_result()


if __name__ == '__main__':
    MainNavigation()
