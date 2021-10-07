#!/usr/bin/env python

#Subscribes to the joy topic, converts this into a twist velocity to be commanded using
#xArm's cartesian velocity control.

#insert joystick control instructions here

import sys
import time
import rospy
from xarm_msgs.srv import *
from geometry_msgs.msg import Twist

rospy.init_node('joy_teleop')

twist=[0,0,0,0,0,0]
ref_frame=0 #0 for base coord frame, 1 for tool coord frame
duration=.1 #might need to change based on jerkyness
r = rospy.Rate(10) # 100hz


def twist_listen(data):
    #print("received a twist")
    #print(data.linear.x)
    global twist 
    twist = [data.linear.x,data.linear.y,data.linear.z,data.angular.x,data.angular.y,data.angular.z]
    #print(twist)
    #xue ye stuff

if __name__ == "__main__":
    rospy.Subscriber("TeleopXArm/cmd_vel",Twist,twist_listen)

    rospy.wait_for_service('/xarm/velo_move_line_timed')
    rospy.set_param('/xarm/wait_for_finish', True)  # return after motion service finish

    #initialize robot to home position
    motion_en = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
    set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
    set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
    home = rospy.ServiceProxy('/xarm/go_home', Move)
    velocity_cmd = rospy.ServiceProxy('/xarm/velo_move_line_timed',MoveVelocity)
    set_acc = rospy.ServiceProxy('/xarm/set_max_acc_line', SetFloat32)

    """
    try:
        motion_en(8, 1)
        set_mode(0)
        set_state(0)
        req = MoveRequest()  # MoveRequest for go_home
        req.mvvelo = .7
        req.mvacc = 3.5
        req.mvtime = 0
        home(req)
        print("running aaaa")

    except rospy.ServiceException as e:
        print("Failed to zero position, service call failed: %s" % e)
        exit(-1)
    """
    
    """
    try:
        #go to cartesian velocity mode
        motion_en(8, 1)
        set_mode(5)
        set_state(0)
        r.sleep()
        #move robot in accordance to joystick command\
        v_cmd = MoveVelocityRequest()
        v_cmd.is_sync= 1
        v_cmd.duration = .2
        v_cmd.speeds=[30,0,0,0,0,0]
        v_cmd.is_tool_coord=ref_frame
        print(v_cmd)
        velocity_cmd(v_cmd)
        
        print("finished cmd")

        #rosservice call /xarm/velo_move_line_timed [30,0,0,0,0,0] 0 1 0.2
    except rospy.ServiceException as e:
        print("Failed to move line timed: %s" % e)
        exit(-1)
    """
    
    motion_en(8, 1)
    set_mode(5)
    set_state(0)
    set_acc(5000)
    
    v_cmd = MoveVelocityRequest()
    v_cmd.is_sync= 1
    v_cmd.duration = 0
    # v_cmd.speeds=[30,0,0,0,0,0]

    while not rospy.is_shutdown():
        print(twist)
        v_cmd.speeds=twist
        v_cmd.is_tool_coord=ref_frame
        velocity_cmd(v_cmd)
        r.sleep()
    

