#include <iostream>

#define __STDC_LIMIT_MACROS
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <ros/ros.h>
#include <scout_msgs/OmniMotorsMsg.h>
#include <std_msgs/Float64.h>


class ControlTranslator
{

public:
    ControlTranslator ()
    {
        sub_ = n_.subscribe ( "/omni/motors", 100, &ControlTranslator::callback, this );
  
        pub_motor_back_ = n_.advertise<std_msgs::Float64> ( "/omni_cobot/scout_body_to_back_wheel_joint_controller/command", 1000 );
        
        pub_motor_left_ = n_.advertise<std_msgs::Float64> ( "/omni_cobot/scout_body_to_left_wheel_joint_controller/command", 1000 );
        
        pub_motor_right_ = n_.advertise<std_msgs::Float64> ( "/omni_cobot/scout_body_to_right_wheel_joint_controller/command", 1000 );
    }
    
    void callback ( const scout_msgs::OmniMotorsMsg::ConstPtr& msg )
    {
        int32_t vels[3] = { 0, 0, 0 };
        
        for ( int i = 0; i < 3; i++)
            vels[i] = msg -> velocity[i];

	std_msgs::Float64Ptr motor_back_msg ( new std_msgs::Float64 );
	std_msgs::Float64Ptr motor_left_msg ( new std_msgs::Float64 );
	std_msgs::Float64Ptr motor_right_msg ( new std_msgs::Float64 );

	motor_back_msg -> data = ( double ) ( - vels[0] / 10 );
	motor_left_msg -> data = ( double ) ( - vels[1] / 10 );
	motor_right_msg -> data = ( double ) ( - vels[2] / 10 );
        
	pub_motor_back_.publish ( motor_back_msg );
	pub_motor_left_.publish ( motor_left_msg );
	pub_motor_right_.publish ( motor_right_msg );

        //pub_motor_back_.publish ( ( double ) ( - vels[0] / 10 ) );
        //pub_motor_left_.publish ( ( double ) ( - vels[1] / 10 ) );
        //pub_motor_right_.publish ( ( double ) ( - vels[2] / 10 ) );
    }
    

private:
    ros::NodeHandle n_;
    
    ros::Subscriber sub_;
    
    ros::Publisher pub_motor_back_;
    ros::Publisher pub_motor_left_;
    ros::Publisher pub_motor_right_;
};



int main ( int argc, char *argv[] )
{
  ros::init ( argc, argv, "omni_control_translator" );
  
  ControlTranslator CT;

  ros::spin();

  return 0;
}
