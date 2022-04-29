#!/usr/bin/env python

import rospy
import smach
import smach_ros
from xarm_msgs.srv import ClearErr
from xarm_msgs.srv import GetErr
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest
import sys
import select
import tty
import termios

import actionlib
from moveit_msgs.msg import MoveGroupAction
from moveit_msgs.msg import MoveGroupGoal
from moveit_msgs.msg import MoveGroupResult
from moveit_msgs.msg import MoveGroupFeedback
from moveit_msgs.msg import MotionPlanRequest


if __name__ == '__main__':
    rospy.init_node('move_group_client_client')
    client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    client.wait_for_server()

    goal = MoveGroupGoal()
    goal.group_name = "xarm7"
    goal. 
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))