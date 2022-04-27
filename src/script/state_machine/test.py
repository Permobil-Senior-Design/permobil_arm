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

CLEAR_ERR_SRV_NAME=  "/xarm/moveit_clear_err"
GET_ERR_SRV_NAME = "/xarm/get_err"
SWITCH_CTRLLER_SRV_NAME = "/xarm/controller_manager/switch_controller"

rospy.wait_for_service(SWITCH_CTRLLER_SRV_NAME)
switch_controller_srv = rospy.ServiceProxy(SWITCH_CTRLLER_SRV_NAME, SwitchController)
#"start_controllers: ['joint_group_velocity_controller'] stop_controllers: ['xarm7_traj_controller_velocity'] strictness: 0 start_asap: false timeout: 0.0"
#SwitchControllerRequest( 'joint_group_velocity_controller','xarm7_traj_controller_velocity',0,True,0)

req = SwitchControllerRequest(start_controllers=['joint_group_velocity_controller'],
                              stop_controllers=['xarm7_traj_controller_velocity'])
switch_controller_srv(req)

