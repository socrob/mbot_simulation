#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers

class PositionCtrlSimFIlter(object):
    '''
    This node is to match mbot sim behavior with real robot
    In real robot both arm and position ctrl are simultaneously running
    In simulation only one at a time can be running

    This node subscribes to the position ctrl command, queries if position ctrl is running
    and if it is then pipes the command to the sim driver (ros control), if not then
    first switches controller and then pipes the command
    '''
    def __init__(self):
        rospy.loginfo("position ctrl sim filter node started")
        # subscriptions

        self.pub_joint_array = []
        # callback array
        cb_array = [self.posCtrlCB0, self.posCtrlCB1, self.posCtrlCB2, self.posCtrlCB3, self.posCtrlCB4, self.posCtrlCB5, self.posCtrlCB6]
        for i in range(0, 6):
            rospy.Subscriber('/left_arm_joint' + str(i) + '_position_controller/command', Float64, cb_array[i], queue_size=1)
        # prepare srv to list controllers (will also inform their status : running or stopped)
        self.list_ctrler_srv = rospy.ServiceProxy ('/controller_manager/list_controllers', ListControllers)



        ## class variables
        #self.emotion = Neutral
        #self.emotion_received = False
        #self.blink_required = rospy.get_param('~blink_required', True)
        #if self.blink_required == True:
            #self.number_of_blinkings_cycles = rospy.get_param('~number_of_blinkings_cycles', 2)
        #else:
            #self.number_of_blinkings_cycles = 1
        #self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        ## inform the user about the parameters that will be used
        #rospy.logdebug("The node will run with the following parameters :")
        #if self.blink_required:
            #rospy.logdebug("blink_required is set to true")
            #rospy.logdebug("Number of blinkings : " + str(self.number_of_blinkings_cycles))
        #else:
            #rospy.logdebug("blink_required is set to false")
        ## publications
        #self.pub_leds = rospy.Publisher('/cmd_leds', UInt8MultiArray, queue_size=10)
        #self.pub_mouth = rospy.Publisher('/cmd_mouth', UInt8MultiArray, queue_size=10)
        ## give some time for the node to subscribe to the topic
        #rospy.sleep(0.2)
        #rospy.logdebug("Ready to receive emotion requests")


    def check_enabled_ctrler(self, controller):
        '''
        query ctrl status (enabled/disabled)
        controller : admissible values 'position', 'velocity' or 'trajectory'
        '''
        if controller not in ['position', 'velocity', 'trajectory']:
            rospy.logerr('ctrler does not exist, admissible values are : position, velocity, trajectory')
            return False
        if controller == 'position':
            # call srv to list ctrlers (will return ctrler and their status)
            print self.list_ctrler_srv()



    def posCtrlCB0(self, msg):
        '''
        callback for position ctrl command on left arm joint 0
        '''
        if self.check_enabled_ctrler('position') == 'position':
            self.arm_pos_pub_array[0].publish(msg)
        else:
            # switch joint to position ctrl
            self.switch_ctrl('position')
            # pipe msg to ros control and move the arm, but now being sure the ctrl is runing
            self.arm_pos_pub_array[0].publish(msg)


    def posCtrlCB1(self, msg):
        '''
        callback for position ctrl command on left arm joint 1
        '''


    def posCtrlCB2(self, msg):
        '''
        callback for position ctrl command on left arm joint 2
        '''


    def posCtrlCB3(self, msg):
        '''
        callback for position ctrl command on left arm joint 3
        '''


    def posCtrlCB4(self, msg):
        '''
        callback for position ctrl command on left arm joint 4
        '''


    def posCtrlCB5(self, msg):
        '''
        callback for position ctrl command on left arm joint 5
        '''


    def posCtrlCB6(self, msg):
        '''
        callback for position ctrl command on left arm joint 6
        '''


    #def start_led_emotions(self):
        ## Initial state
        ## display a neutral mouth expression
        #self.mouth_display(Neutral)
        ## iluminate eyes fully in blue colour
        #self.set_eye_leds(0, 0, 50, 50, 0)
        #self.set_eye_leds(0, 0, 50, 50, 0)
        #self.set_eye_leds(1, 0, 50, 50, 0)
        #self.set_eye_leds(2, 0, 0, 0, 0)
        #self.set_eye_leds(3, 0, 0, 0, 0)
        #self.set_eye_leds(4, 0, 0, 0, 0)
        #self.set_eye_leds(5, 0, 0, 0, 0)

        #while not rospy.is_shutdown():
            #if self.emotion_received == True:
                ## lower flag
                #self.emotion_received = False
                ## update mouth emotion
                #self.mouth_emotion_update()
                ## update eyes emotion
                #self.eyes_emotion_update()
            #self.loop_rate.sleep()

def main():
    rospy.init_node('mbot_switch_ctrl_node', anonymous=False)
    mbot_switch_ctrl_node = PositionCtrlSimFIlter()
    mbot_switch_ctrl_node.start_led_emotions()
