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

old_settings = termios.tcgetattr(sys.stdin)

CLEAR_ERR_SRV_NAME = "/xarm/moveit_clear_err"
GET_ERR_SRV_NAME = "/xarm/get_err"
SWITCH_CTRLLER_SRV_NAME = "/xarm/controller_manager/switch_controller"


def getKey():
    c = ''
    try:
        tty.setcbreak(sys.stdin.fileno())

        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            c = sys.stdin.read(1)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    if c != '':
        print(c)
        print(c)

    return c.lower()


class SpaceMouseManual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trigger_auto', 'trigger_moveit_manual','trigger_error'])

        # declare services
        '''
        rospy.wait_for_service(GET_ERR_SRV_NAME)
        self.get_err_srv = rospy.ServiceProxy(GET_ERR_SRV_NAME, GetErr)

        rospy.wait_for_service(SWITCH_CTRLLER_SRV_NAME)
        self.switch_controller_srv = rospy.ServiceProxy(
            SWITCH_CTRLLER_SRV_NAME, SwitchController)
        '''


    def execute(self, userdata):
        rospy.loginfo('Executing state Manual')
        # monitor  for error
        # change to the group velocity controller and then remain in this state until a key is pressed

        '''
        self.switch_controller_srv(SwitchControllerRequest(
                                        start_controllers=['joint_group_velocity_controller'],
                                        stop_controllers=['xarm7_traj_controller_velocity']))
        '''



        while not rospy.is_shutdown():
            # call error service to check if error exists
            '''
            res=self.get_err_srv()

            # rosservice call /xarm/get_err
            if res.err:
                return 'trigger_error'
            
            '''


            key=getKey()

            if(key == 'm'):
                # switch to auto
                print("m pressed")
                return 'trigger_moveit_manual'


class MoveitManual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trigger_auto','trigger_spacemouse_manual'])

        '''
        rospy.wait_for_service(SWITCH_CTRLLER_SRV_NAME)
        self.switch_controller_srv=rospy.ServiceProxy(
            SWITCH_CTRLLER_SRV_NAME, SwitchController)
        '''


    def execute(self, userdata):
        rospy.loginfo('Executing state Automatic')
        # switch to trajectory controller

        '''
        self.switch_controller_srv(SwitchControllerRequest(
                                start_controllers=['joint_group_velocity_controller'],
                                stop_controllers=['xarm7_traj_controller_velocity']))
        '''


        while not rospy.is_shutdown():
            key=getKey()

            if(key == 's'):
                print("s pressed")
                return 'trigger_moveit_manual'


class Error(smach.State):

    def __init__(self):
        '''
        rospy.wait_for_service(CLEAR_ERR_SRV_NAME)
        self.clear_err_srv=rospy.ServiceProxy(CLEAR_ERR_SRV_NAME, ClearErr)
        '''


        smach.State.__init__(self, outcomes=['manual_error_cleared'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Error, clearing error now')
        rospy.sleep(1)

        try:
            pass
            #resp1=self.clear_err_srv()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))




        return 'manual_error_cleared'

class Automatic(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])
        # change the contoller type


    def execute(self, userdata):
        rospy.loginfo('Executing state Automatic')

        while not rospy.is_shutdown():
            # call error service to check if error exists

            # res = self.get_err_srv()
            # print(res.err)

            # rosservice call /xarm/get_err
            # if res.err:
            #    return 'trigger_error'
            key=getKey().upper()
            if(key == 'm'):
            # switch to auto
                print("a pressed")

        return 'outcome2'





        print(msg.err)
        return False
    else:
        return True

def main():
    rospy.init_node('smach_example_state_machine')

    # connect the services needed


    # Create a SMACH state machine
    sm=smach.StateMachine(outcomes=['outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SpaceMouseManual', SpaceMouseManual(),
                            transitions={'trigger_auto': 'Automatic',
                                        'trigger_moveit_manual': 'MoveitManual',
                                        'trigger_error': 'Error'})

        smach.StateMachine.add('MoveitManual', MoveitManual(),
                    transitions={'trigger_auto': 'Automatic',
                                'trigger_spacemouse_manual': 'SpaceMouseManual'})
        smach.StateMachine.add('Automatic', Automatic(),
                            transitions={'outcome2': 'SpaceMouseManual'})

        smach.StateMachine.add('Error', Error(),
                            transitions={'manual_error_cleared': 'SpaceMouseManual'})


    # Execute SMACH plan
    outcome=sm.execute()

    sis=smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
