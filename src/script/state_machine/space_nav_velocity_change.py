#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import dynamic_reconfigure.client
from xarm_msgs.srv import *
#import keyboard


increment = 0.1
linSpeedParam = '/servo_server/scale/linear'


# only add or subtract on toggles from 0 to 1

prevLeft = 0
prevRight = 0

# 0 - gripper mode
# 1 - velocity change mode
mode = 0

def callback(data):
    global prevLeft, prevRight,mode

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
    rightButton = data.buttons[0]
    leftButton = data.buttons[1]
    
    

    
    curLinSpeed = rospy.get_param(linSpeedParam)

    #need to implement this into state machine
    if( not (rightButton == leftButton) ):
        
        if(leftButton == 1 and prevLeft == 0 ):
            if mode == 1:
                client.update_configuration({"linear":curLinSpeed + increment })
            elif mode == 0:
                gripperMove(850)
        elif(rightButton == 1 and prevRight == 0 ):
            if mode == 1:
                client.update_configuration({"linear":curLinSpeed - increment })
            elif mode == 0:
                gripperMove(0)
        
        prevLeft = leftButton
        prevRight = rightButton
    

    


    
def listener():
    
    rospy.init_node('space_nav_velocity_change', anonymous=False)
    #wait for dynamic reconfigure param to spin up

    rospy.sleep(1)

    global client
    client = dynamic_reconfigure.client.Client("servo_server/scale")

    global gripperMove
    gripperMove = rospy.ServiceProxy('/xarm/gripper_move', GripperMove)


    rospy.Subscriber("/spacenav/joy", Joy, callback)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()