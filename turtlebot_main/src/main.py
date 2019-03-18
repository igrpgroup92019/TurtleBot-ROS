#! /usr/bin/env python

import rospy
import actionlib
import geometry_msgs
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from math import tan, cos, sin, pi
from visualization_msgs.msg import Marker

class MainNavigation:

    #current x y and theta coordinates
    current_x = 0.0
    current_y = 0.0
    current_theta = 0.0

    #x y and theta coordinates for goals
    x = 0.0
    y = 0.0
    theta = 0.0

    #x y and theta coordinates for marker
    m_x = 0.0
    m_y = 0.0
    m_theta = 0.0
    m_pitch = 0.0

    def __init__(self):
        #Initializes node
        rospy.init_node('map_navigation')
        sub_goal = rospy.Subscriber('/rtabmap/goal_out', PoseStamped, self.goal_listener)
        self.sub_marker = rospy.Subscriber('/aruco_single/pose', PoseStamped, self.marker_listener)
        self.pub_marker = rospy.Publisher('rviz_marker', Marker, queue_size=1)

        #bool variables for checking
        self.goal_received = False
        self.marker_detected = False

        #variable to check if goal is normal or towards aruco marker
        self.marker_goal = False

        # Creates the SimpleActionClient
        self.action_server_name = '/move_base'
        self.move_base_client = actionlib.SimpleActionClient(self.action_server_name, MoveBaseAction)

        self.start_navigation()

    # definition of the feedback callback. This will be called when feedback
    # is received from the action server
    # it prints the current location of the robot based on the x,y and theta coordinates
    def feedback_callback(self, feedback):
        global current_x, current_y, current_theta
        current_x = feedback.base_position.pose.position.x
        current_y = feedback.base_position.pose.position.y
        c_or = feedback.base_position.pose.orientation
        (roll,pitch,current_theta) = euler_from_quaternion([c_or.x, c_or.y, c_or.z, c_or.w])
        rospy.loginfo('feedback received => Current Robot Position: x = %d, y = %d, theta = %d', current_x, current_y, current_theta)

    def goal_listener(self, coordinates):
        global x, y, theta
        x = coordinates.pose.position.x
        y = coordinates.pose.position.y
        orient = coordinates.pose.orientation
        (roll,pitch,theta) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
        self.goal_received = True
        print "Goal received"

    def marker_listener(self, marker_pose):
        global m_x, m_y, m_theta, m_pitch
        m_x = marker_pose.pose.position.x
        m_y = marker_pose.pose.position.y
        m_orient = marker_pose.pose.orientation
        (roll,m_pitch,m_theta) = euler_from_quaternion([m_orient.x, m_orient.y, m_orient.z, m_orient.w])
        self.marker_detected = True

    def reach_marker(self):
        print "Recalculating goal..."
        #aruco_height = 0.2
        #print m_pitch
        #distance = aruco_height / tan(m_pitch)
        #rospy.loginfo("Distance to alien is %d", distance)
        #change_x = distance * sin(m_theta)
        #print change_x
        #change_y = distance * cos(m_theta)
        #print change_y
        #new_x = current_x + change_x
        #new_y = current_y + change_y
        rospy.loginfo("x distance is %d, while y distance is %d", m_x, m_y)
        new_x = current_x - m_x
        if m_y > 0:
            new_y = current_y + m_y
        else:
            new_y = current_y - m_y
        new_theta = pi
        self.marker_goal = True
        self.send_goal(new_x, new_y, new_theta)
        #new theta should be an orientation which either faces the alien or 180 degrees from it ready to go back

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
        robot_marker.scale.x = 1.0
        robot_marker.scale.y = 1.0
        robot_marker.scale.z = 1.0
        robot_marker.pose = pose
        robot_marker.color.a = 1.0
        robot_marker.color.b = 0.0
        if self.marker_detected == 0:
            robot_marker.type = Marker.SPHERE
            robot_marker.color.r = 0.0
            robot_marker.color.g = 1.0
        else:
            robot_marker.type = Marker.CUBE
            robot_marker.color.r = 1.0
            robot_marker.color.g = 0.0

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

            #publish rviz marker for goal
            self.pub_marker.publish(robot_marker)

            #goal cancelling can only be possible when goal is normal not when going towards marker
            if self.marker_detected == True and self.marker_goal == False:
                self.move_base_client.cancel_goal()
                print "Alien has been detected!"
                self.marker_detected = False
                #goal recalculation
                self.reach_marker()
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
        self.marker_goal = False
        return self.move_base_client.get_result()


if __name__ == '__main__':
    MainNavigation()
