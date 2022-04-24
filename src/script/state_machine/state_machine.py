#!/usr/bin/env python

import rospy
import smach
import smach_ros
from xarm_msgs.srv import ClearErr
from xarm_msgs.msg import RobotMsg

CLEAR_ERR_SRV_NAME=  "/xarm/moveit_clear_err"

'''
class Manual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'],input_keys=['input'])
        #rospy.init_node('manual_state_monitor', anonymous=True)
        #rospy.Subscriber("/xarm/xarm_states",RobotMsg,self.cb)

        #self.xarm_error = False
    
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Manual')
        # monitor  for error

        while not rospy.is_shutdown():
            if userdata.input == 'm':
                return 'outcome2'
        # elif keypress:

        # monitor keyboard press

        #enter error if listening on xarm server and recieve error message

'''



class Automatic(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])
        # change the contoller type


    def execute(self, userdata):
        rospy.loginfo('Executing state Automatic')
        return 'outcome2'


class Error(smach.State):

    def __init__(self):
        #clear the error
        #rosservice call /xarm/moveit_clear_err
        #rosservice call /xarm/clear_err
        
        rospy.wait_for_service(CLEAR_ERR_SRV_NAME)
        self.clear_err_srv = rospy.ServiceProxy(CLEAR_ERR_SRV_NAME, ClearErr)

        smach.State.__init__(self, outcomes=['outcome6'])

    def execute(self,userdata):
        rospy.loginfo('Executing state Error, clearing error now')
        
        # recover 
        try:
            resp1 = self.clear_err_srv()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        rospy.sleep(1)

        return 'outcome6'

# main

def monitor_cb(ud, msg):
    if msg.err != 0:
        print("error msg is ")

        print(msg.err)
        return False
    else:
        return True

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome5'])
    sm.userdata.input = 1
    # Open the container
    with sm:
        # Add states to the container
        #smach.StateMachine.add('Manual', Manual(), transitions={'outcome1':'Manual', 'outcome2':'Error'})

        #smach.StateMachine.add('Automatic', Automatic(), transitions={'outcome2':'Manual'})
        smach.StateMachine.add('Manual', smach_ros.MonitorState("/xarm/xarm_states", RobotMsg, monitor_cb,1),
                            transitions={'invalid':'Error', 'valid':'Manual', 'preempted':'Manual'})
        smach.StateMachine.add('Error', Error(), 
                            transitions={'outcome6':'Manual'})
        
    
    # Execute SMACH plan
    outcome = sm.execute()

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()