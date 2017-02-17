#! /usr/bin/env python

import sys
import math
import random

import roslib; roslib.load_manifest('demos')
import rospy
import smach
import smach_ros

from std_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from scout_msgs.msg import *
from scout_msgs.srv import *


# Bedroom Coords
BEDROOM_X = -3
BEDROOM_Y = 2.75
BEDROOM_T = 0

# Living Room Coords
LR_X = 0
LR_Y = 0
LR_T = 0

# Bathroom Coords
BATHROOM_X = 1
BATHROOM_Y = 3
BATHROOM_T = 0

# Inside Hallway Coords
IH_X = 2
IH_Y = 1.5
IH_T = 0



def pose2alib ( x, y, t ):
    goal = PoseStamped ( header         = Header ( frame_id = "/map" ),
                         pose           = Pose ( position = Point ( x, y, 0 ),
                         orientation    = Quaternion ( 0, 0, math.sin ( t / 2.0 ), math.cos ( t / 2.0 ) ) ) )

    return MoveBaseGoal ( target_pose = goal )



def task ():
    sm = smach.StateMachine ( ['succeeded', 'preempted', 'aborted'] )
    
    with sm:
      sm.add ( 'MOVE_2_BEDROOM',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( BEDROOM_X, BEDROOM_Y, BEDROOM_T ) ),
               transitions = { 'succeeded': 'MOVE_2_LIVING_ROOM', 'aborted': 'FAILURE' } )
      
      sm.add ( 'MOVE_2_LIVING_ROOM',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( LR_X, LR_Y, LR_T ) ),
               transitions = { 'succeeded': 'MOVE_2_BATHROOM', 'aborted': 'FAILURE' } )
        
      sm.add ( 'MOVE_2_BATHROOM',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( BATHROOM_X, BATHROOM_Y, BATHROOM_T ) ),
               transitions = { 'succeeded': 'MOVE_2_INSIDE_HALLWAY', 'aborted': 'FAILURE' } )
      
      sm.add ( 'MOVE_2_INSIDE_HALLWAY',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( IH_X, IH_Y, IH_T ) ),
               transitions = { 'succeeded': 'MOVE_2_LIVING_ROOM_2', 'aborted': 'FAILURE' } )
      
      sm.add ( 'MOVE_2_LIVING_ROOM_2',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( LR_X, LR_Y, LR_T ) ) )
      
      sm.add ( 'FAILURE',
               smach_ros.SimpleActionState ( 'move_base', MoveBaseAction, goal = pose2alib ( LR_X, LR_Y, LR_T ) ) )
    
    return sm



def main ( argv ):
    # Init ROS
    rospy.init_node( "robotica2014_task" )
    
    # Define the task
    sm = task()
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer ( 'behavior_test', sm, '/SM_ROOT' )
    sis.start()

    # Execute state machine
    rospy.loginfo ( "Executing behavior state machine" )
    sm.execute()

    sis.stop()



if __name__ == '__main__':
    main ( sys.argv )
