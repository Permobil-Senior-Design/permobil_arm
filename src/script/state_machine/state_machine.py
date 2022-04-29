#!/usr/bin/env python
import rospy
import smach
import smach_ros
from xarm_msgs.srv import *
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8
from moveit_msgs.srv import *
from std_srvs.srv import Empty
import dynamic_reconfigure.client
from actionlib.simple_action_client import SimpleActionClient, GoalStatus
import sys
import os
import roslaunch


from KBHit import KBHit



CLEAR_ERR_SRV_NAME = "/xarm/moveit_clear_err"
GET_ERR_SRV_NAME = "/xarm/get_err"
SWITCH_CTRLLER_SRV_NAME = "/xarm/controller_manager/switch_controller"
SET_MODE_SRV_NAME = "/xarm/set_mode"
SET_STATE_SRV_NAME = "/xarm/set_state"
GRIPPER_MOVE_SRV_NAME = "/xarm/gripper_move"
GET_GRIPPER_STATE_SRV_NAME = "/xarm/gripper_state"
SET_GRIPPER_CONFIG_SRV_NAME = "/xarm/gripper_config"
RESET_SERVO_SERVER_STATUS_SRV_NAME = "/servo_server/reset_servo_status"
kb = KBHit()

def getKey():
    c =''
    if kb.kbhit():
        c = kb.getch()

    return c.lower()

# connect the services needed

rospy.wait_for_service(CLEAR_ERR_SRV_NAME, timeout=1)
rospy.wait_for_service(GET_ERR_SRV_NAME, timeout=1)
rospy.wait_for_service(SWITCH_CTRLLER_SRV_NAME, timeout=1)
rospy.wait_for_service(SET_MODE_SRV_NAME, timeout=1)
rospy.wait_for_service(GRIPPER_MOVE_SRV_NAME, timeout=1)
rospy.wait_for_service(GET_GRIPPER_STATE_SRV_NAME, timeout=1)
rospy.wait_for_service(SET_GRIPPER_CONFIG_SRV_NAME, timeout=1)
rospy.wait_for_service(RESET_SERVO_SERVER_STATUS_SRV_NAME, timeout=1)

switch_controller_srv=rospy.ServiceProxy(SWITCH_CTRLLER_SRV_NAME, SwitchController)
set_mode_srv=rospy.ServiceProxy(SET_MODE_SRV_NAME, SetInt16)
set_state_srv=rospy.ServiceProxy(SET_STATE_SRV_NAME, SetInt16)
get_err_srv = rospy.ServiceProxy(GET_ERR_SRV_NAME, GetErr)
clear_err_srv=rospy.ServiceProxy(CLEAR_ERR_SRV_NAME, ClearErr)
gripper_move_srv = rospy.ServiceProxy(GRIPPER_MOVE_SRV_NAME, GripperMove)
get_gripper_state_srv = rospy.ServiceProxy(GET_GRIPPER_STATE_SRV_NAME, GripperState)
set_gripper_config_srv = rospy.ServiceProxy(SET_GRIPPER_CONFIG_SRV_NAME, GripperConfig)
reset_servo_server_status_srv = rospy.ServiceProxy(RESET_SERVO_SERVER_STATUS_SRV_NAME, Empty)


class SpaceMouseManual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trigger_auto', 'trigger_moveit_manual','trigger_error','trigger_limp'])

        #self.recordedPose =
        #start node listening to joy topic to check for 

        #define the return map
        self.ret = {
            'm': 'trigger_moveit_manual',
            'l': 'trigger_limp',
            'e': 'trigger_auto'
        }

        # speed adjust
        self.servo_config_dr_client = dynamic_reconfigure.client.Client("servo_server/scale")

        self.linVelKey = {
            'g': False,
            'h': True
        }

        self.rotSpeedKey = {
            't': False,
            'y': True
        }

        # launch gpd demo with args
        '''
                self.select = {
            'q': 'topLeft',
            'w': 'topRight',
            'a': 'botLeft',
            's': 'botRight',
            'e': 'center'
        }
        '''


    def gripperMoveCB(self,data):
        rightButton = data.buttons[0]
        leftButton = data.buttons[1]

        if not rightButton == leftButton:
            #state = get_gripper_state_srv()

            curPos = get_gripper_state_srv().curr_pos
            newPos = curPos
            print(curPos)
            
            if leftButton == 1 :
                newPos = curPos - 50
            elif rightButton == 1:
                newPos = curPos + 50

            gripper_move_srv(newPos)

    def servoStatusCB(self,data):
        if data.data > 0:
            print("I know the servo status is wrong right now! Clearing!")
            reset_servo_server_status_srv()
            pass

    def changeLinVel(self,inc):
        newVel = rospy.get_param('/servo_server/scale/linear') + (.1 if inc else -.1)

        if newVel  < 2 or newVel > 0:
            self.servo_config_dr_client.update_configuration({
                "linear": newVel
                })
            print("current linear vel: %s"%rospy.get_param('/servo_server/scale/linear'))
        else:
            print("max linear velocity reached")
        

    def changeRotVel(self,inc):
        newVel = rospy.get_param('/servo_server/scale/rotational') + (.1 if inc else -.1)

        if  newVel  < 2 or newVel > 0:
            self.servo_config_dr_client.update_configuration({
                "rotational": newVel
                })
            print("current rotational vel: %s"%rospy.get_param('/servo_server/scale/rotational'))
        else:
            print("max rotational velocity reached")

    def unregisterSubs(self):
        self.joySub.unregister()
        self.servoStatusSub.unregister()


    def execute(self, userdata):
        rospy.loginfo('Executing state Manual')
        # monitor  for error
        # change to the group velocity controller and then remain in this state until a key is pressed
        set_mode_srv(4)
        set_state_srv(0)

        switch_controller_srv(SwitchControllerRequest(
                                        start_controllers=['joint_group_velocity_controller'],
                                        stop_controllers=['xarm7_traj_controller_velocity']))
        
        self.joySub = rospy.Subscriber("/spacenav/joy", Joy, self.gripperMoveCB)
        self.servoStatusSub = rospy.Subscriber("/servo_server/status",Int8, self.servoStatusCB)

        while not rospy.is_shutdown():
            #for some reason this doesnt work even though getKey is definitely being called

            c =  getKey()



            # check if intentionally state switch is requested
            if c in self.ret:
                self.unregisterSubs()
                return self.ret[c]
            elif c in self.linVelKey:
                self.changeLinVel(self.linVelKey[c])
            elif c in self.rotSpeedKey:
                self.changeRotVel(self.rotSpeedKey[c])
            '''
            elif c in self.select:
                

                return "trigger_auto"
            '''

            # call services for hardware errors and collision errors
            if get_err_srv().err:
                self.unregisterSubs()
                return 'trigger_error'
            
            #else if 
            
            

class MoveitManual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trigger_auto','trigger_spacemouse_manual','trigger_limp'])
        self.ret = {
            's': 'trigger_spacemouse_manual',
            'l': 'trigger_limp'
        }

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveitManual')
        # switch to trajectory controller

        set_mode_srv(4)
        set_state_srv(0)

        switch_controller_srv(SwitchControllerRequest(
                                start_controllers=['xarm7_traj_controller_velocity'],
                                stop_controllers=['joint_group_velocity_controller']))


        while not rospy.is_shutdown():
            c = getKey()
            if c in self.ret:
                return self.ret[c]

class Limp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trigger_spacemouse_manual','trigger_moveit_manual',])
        self.ret = {
            'm': 'trigger_moveit_manual',
            's': 'trigger_spacemouse_manual'
        }

    def execute(self, userdata):
        switch_controller_srv(SwitchControllerRequest(
                        start_controllers=[''],
                        stop_controllers=['joint_group_velocity_controller','xarm7_traj_controller_velocity']))
        
        set_mode_srv(2)
        set_state_srv(0)

        set_gripper_config_srv(1500)

        while not rospy.is_shutdown():
            c = getKey()
            if c in self.ret:
                return self.ret[c]

class Error(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['manual_error_cleared'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Error, clearing error now')
        rospy.sleep(1)

        try:
            pass
            resp1=clear_err_srv()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return 'manual_error_cleared'

class Automatic(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trigger_spacemouse_manual'])


    def execute(self, userdata):
        rospy.loginfo('Executing state Automatic')

        set_mode_srv(4)
        set_state_srv(0)

        # check 

        switch_controller_srv(SwitchControllerRequest(
                                start_controllers=['xarm7_traj_controller_velocity'],
                                stop_controllers=['joint_group_velocity_controller']))
        
        #os.system("roslaunch permobil_arm gpd_demo.launch")
        #os.system()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/xueyelin/test_ws/src/permobil_arm/launch/gpd_demo.launch"])
        launch.start()
        rospy.loginfo("started")
        rospy.sleep(1)
        # 3 seconds later
        launch.shutdown()

        while not rospy.is_shutdown():

            c = getKey()
            if(c == 's'):
            # switch to auto
                return 'trigger_spacemouse_manual'


def main():
    rospy.init_node('xarm_state_machine')



    # Create a SMACH state machine
    sm=smach.StateMachine(outcomes=['outcome5'])
    sm.userdata.nonCollisionJoints = 0
    # Open the container
    sis=smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    with sm:
        # Add states to the container
        smach.StateMachine.add('SpaceMouseManual', SpaceMouseManual(),
                            transitions={'trigger_auto': 'Automatic',
                                        'trigger_moveit_manual': 'MoveitManual',
                                        'trigger_error': 'Error',
                                        'trigger_limp':'Limp'
                                        })

        smach.StateMachine.add('MoveitManual', MoveitManual(),
                    transitions={'trigger_auto': 'Automatic',
                                'trigger_spacemouse_manual': 'SpaceMouseManual',
                                'trigger_limp':'Limp'})
        smach.StateMachine.add('Limp', Limp(),
                    transitions={'trigger_spacemouse_manual': 'SpaceMouseManual',
                                    'trigger_moveit_manual': 'MoveitManual'})

        smach.StateMachine.add('Automatic', Automatic(),
                            transitions={'trigger_spacemouse_manual': 'SpaceMouseManual'})
        
        '''
                smach.StateMachine.add('MoveGroupAction',
                      simple_action_state('move_group',
                                        GripperAction,
                                        goal_slots=['max_effort', 
                                                    'position']),
                      transitions={'succeeded':'APPROACH_PLUG'},
                      remapping={'max_effort':'user_data_max',
                                 'position':'user_data_position'})
        
        '''


        smach.StateMachine.add('Error', Error(),
                            transitions={'manual_error_cleared': 'Limp'})


    # Execute SMACH plan
    outcome=sm.execute()



    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
