#!/usr/bin/env python

import time
import math

import numpy as np

import rospy
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

NODE_NAME = "virtual_omni_driver"

# wheel command topics (rad/sec)
BACK_WHEEL_TOPIC  = 'mbot_gazebo/back_wheel_controller/command'
LEFT_WHEEL_TOPIC  = 'mbot_gazebo/left_wheel_controller/command'
RIGHT_WHEEL_TOPIC = 'mbot_gazebo/right_wheel_controller/command'
CMD_VEL_TOPIC     = 'cmd_vel'

# wheel joints
JOINT_STATES_TOPIC = 'mbot_gazebo/joint_states'
BACK_WHEEL_JOINT   = 'back_wheel_joint'
LEFT_WHEEL_JOINT   = 'left_wheel_joint'
RIGHT_WHEEL_JOINT  = 'right_wheel_joint'


# wheel_distance from URDF (distance from kinematic center and wheel)
WD = 0.2

# wheel_radius from URDF
WR = 0.1

A = np.array([[  0,               1.0, -WD ],
              [  math.sqrt(3)/2, -0.5, -WD ],
              [ -math.sqrt(3)/2, -0.5, -WD ]])
Ainv = np.linalg.inv(A)


class Driver:
    prev_bw_p = prev_lw_p = prev_rw_p = None
    odom = np.zeros(3)

    v_xyt = np.empty(3)
    v_blr = np.empty(3)
    f_blr = [Float64() for i in xrange(3)]
    def cmd_cb(self, msg):
        self.v_xyt[0:3] = (msg.linear.x, msg.linear.y, msg.angular.z)
        np.dot(A, self.v_xyt, out=self.v_blr)
        for i in xrange(3):
            self.f_blr[i].data = self.v_blr[i] / WR
        self.bw_pub.publish(self.f_blr[0])
        self.lw_pub.publish(self.f_blr[1])
        self.rw_pub.publish(self.f_blr[2])

    d_blr = np.empty(3)
    dp_body = np.empty(3)
    def js_cb(self, msg):
        try:
            # get wheels angular position
            bw_i = msg.name.index(BACK_WHEEL_JOINT)
            bw_p = msg.position[bw_i]
            lw_i = msg.name.index(LEFT_WHEEL_JOINT)
            lw_p = msg.position[lw_i]
            rw_i = msg.name.index(RIGHT_WHEEL_JOINT)
            rw_p = msg.position[rw_i]
            if self.prev_bw_p is not None:
                # get wheels displacement
                self.d_blr[0:3] = (WR*(bw_p - self.prev_bw_p),
                                   WR*(lw_p - self.prev_lw_p),
                                   WR*(rw_p - self.prev_rw_p))
                # get body displacement in body frame
                np.dot(Ainv, self.d_blr, out=self.dp_body)
                # integrate in world frame
                ct = math.cos(self.odom[2] + self.dp_body[2]/2)
                st = math.sin(self.odom[2] + self.dp_body[2]/2)
                R  = np.array([[ct, -st],
                               [st,  ct]])
                self.odom[0] += ct*self.dp_body[0] - st*self.dp_body[1]
                self.odom[1] += st*self.dp_body[0] + ct*self.dp_body[1]
                self.odom[2] += self.dp_body[2]
                # publish to TF
                self.tfb.sendTransform((self.odom[0], self.odom[1], 0),
                                       tf.transformations.quaternion_about_axis(self.odom[2], (0,0,1)),
                                       msg.header.stamp, 'base_link', 'odom')
            (self.prev_bw_p, self.prev_lw_p, self.prev_rw_p) = (bw_p, lw_p, rw_p)
        except ValueError:
            rospy.logwarn("Wheel joint not being published")

    def main(self):
        rospy.init_node(NODE_NAME)
        # controllers
        self.bw_pub  = rospy.Publisher(BACK_WHEEL_TOPIC,  Float64)
        self.lw_pub  = rospy.Publisher(LEFT_WHEEL_TOPIC,  Float64)
        self.rw_pub  = rospy.Publisher(RIGHT_WHEEL_TOPIC, Float64)
        self.cmd_sub = rospy.Subscriber(CMD_VEL_TOPIC, Twist, self.cmd_cb)
        # odometry
        self.tfb     = tf.TransformBroadcaster()
        self.js_sub  = rospy.Subscriber(JOINT_STATES_TOPIC, JointState, self.js_cb)
        rospy.loginfo("Node %s ready."%(NODE_NAME))
        rospy.spin()

if __name__=='__main__':
    Driver().main()

# EOF
