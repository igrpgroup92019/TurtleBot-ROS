#! /usr/bin/env python

import rospy
import actionlib
import geometry_msgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations
import time

#node to create the aruco markers, detect them and make new navigation goal
class Park:
