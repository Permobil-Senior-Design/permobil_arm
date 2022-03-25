#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import dynamic_reconfigure.client


increment = 0.1
linSpeedParam = '/servo_server/scale/linear'

client = dynamic_reconfigure.client.Client("servo_server")


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
    leftButton = data.buttons[0]
    rightButton = data.buttons[1]
    
    curLinSpeed = rospy.get_param(linSpeedParam)

    
    if(leftButton):
        # decrease speed
        #rosparam get /servo_server/xarm7/scale/linear
        #rospy.set_param(linSpeedParam, curLinSpeed - increment )
        client.update_configuration({"linear":curLinSpeed + increment })
    if(rightButton):
        client.update_configuration({"linear":curLinSpeed - increment })
    


    
def listener():

    rospy.init_node('space_nav_velocity_change', anonymous=False)

    rospy.Subscriber("/spacenav/joy", Joy, callback)



    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()