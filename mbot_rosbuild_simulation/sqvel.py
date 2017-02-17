#!/usr/bin/env python

import functools
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

JOINTS = ['back_wheel', 'left_wheel', 'right_wheel']

DB = {}

def cmd_cb(joint, msg):
    DB['%s_cmd'%(joint)] = msg.data

def jst_cb(msg):
    for name in ['%s_joint'%(j) for j in JOINTS]:
        idx = msg.name.index(name)
        pos = msg.position[idx]
        vel = msg.velocity[idx]
        eff = msg.effort[idx]
        DB[name] = (pos, vel, eff)

def main():
    rospy.init_node('square_cmd_vel', anonymous=True)
    argv = rospy.myargv()
    cmd = argv[1] if len(argv)>1 else 'help'
    if cmd=='help':
        print """
Commands:
  help -- this message
  log  -- logging
  cmd  -- commanding
  plot -- plotting
"""
    elif cmd=='log':
        sub1 = [rospy.Subscriber('mbot_gazebo/%s_controller/command'%(j), Float64,
                                 functools.partial(cmd_cb, j))
                for j in JOINTS]
        sub2 = rospy.Subscriber('mbot_gazebo/joint_states', JointState, jst_cb)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                print rospy.get_time(),
                for j in JOINTS:
                    cmd = DB['%s_cmd'%(j)]
                    (pos, vel, eff) = DB['%s_joint'%(j)]
                    print cmd, pos, vel, eff,
                print
            except KeyError:
                pass
            rate.sleep()
    elif cmd=='cmd':
        pub = rospy.Publisher('cmd_vel', Twist)
        msg = Twist()
        while not rospy.is_shutdown():
            # ON
            print "ON"
            msg.angular.z = 1
            pub.publish(msg)
            rospy.sleep(2)
            # OFF
            print "OFF"
            msg.angular.z = 0
            pub.publish(msg)
            rospy.sleep(2)
    elif cmd=='plot':
        import numpy as np
        import matplotlib.pyplot as plt
        data = np.loadtxt("log")
        for offset in [1, 5, 9]:
            plt.figure()
            plt.plot(data[:,0], data[:,offset+0], label='cmd')
            plt.plot(data[:,0], data[:,offset+1], label='pos')
            plt.plot(data[:,0], data[:,offset+2], label='vel')
            plt.plot(data[:,0], data[:,offset+3], label='eff')
            plt.legend()
        plt.show()
    else:
        print "*** Invalid command"


if __name__=='__main__':
    main()

# EOF
