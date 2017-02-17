
#include <iostream>

#define __STDC_LIMIT_MACROS
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mbot_catkin_msgs/OmniMotorsMsg.h>

const double pi = 3.1415926535897;

#define NA 9
double A[NA];

// from SousaMotors.cpp: BASE_TIME/1000, where BASE_TIME=6.5
#define VELOCITY_SCALE 0.0065

ros::Publisher motors_pub;

void motion_handler(const geometry_msgs::Twist::ConstPtr& msg) {
  double vb[3] = { msg->linear.x,
                   msg->linear.y,
                   msg->angular.z };
  double vw[3];
  // matrix multiplication A.vb
  for (int i=0 ; i<3 ; i++) {
    vw[i] = 0;
    for (int j=0 ; j<3 ; j++)
      vw[i] += A[3*i+j] * vb[j];
  }
  // prepare message
  static mbot_catkin_msgs::OmniMotorsMsg out;
  out.header.stamp = ros::Time::now();
  out.enable = true;
  for (int i=0 ; i<3 ; i++)
    out.velocity[i] = VELOCITY_SCALE * vw[i];
  motors_pub.publish(out);

  // printf("vx=%g vy=%g vt=%g  -->  v1=%g v2=%g v3=%g  -->  r1=%g r2=%g r3=%g\n",
  // 	 vb[0], vb[1], vb[2],
  // 	 vw[0], vw[1], vw[2],
  // 	 out.velocity[0], out.velocity[1], out.velocity[2]);
}


bool read_params(ros::NodeHandle &n) {
  std::vector<double> buf;
  if (n.getParam("omni/calibration_direct", buf)) {
    if (buf.size()==NA) {
      for (int i=0 ; i<NA ; i++)
	A[i] = buf[i];
      ROS_INFO("Loaded %d calibration parameters.", NA);
      return true;
    } else ROS_ERROR("Direct kinematics parameters must be a flat vector of %d elements", NA);
  } else ROS_ERROR("Direct kinematics parameters missing");
  return false;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "omni_motion_node");
  ros::NodeHandle n;

  if (read_params(n)) {
    ros::Subscriber sub = n.subscribe("cmd_vel", 10, motion_handler);
    motors_pub = n.advertise<mbot_catkin_msgs::OmniMotorsMsg>("omni/motors", 10);
    ROS_INFO("Motion node ready.");
    ros::spin();
  }

  return 0;
}
