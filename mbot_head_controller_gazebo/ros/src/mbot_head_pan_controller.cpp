#include "mbot_head_pan.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
namespace mbot_head_pan_controller
{
MbotHeadPanControllerServer::MbotHeadPanControllerServer(): np_("~")
    {
        gazebo_pub_  = np_.advertise<std_msgs::Float64>("/mbot/mbot_head_position_controller/command", 1);

        cmd_head_sub_ = np_.subscribe("/cmd_head", 1, &MbotHeadPanControllerServer::headPanCallback,this);

    }
void MbotHeadPanControllerServer::headPanCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
    {

        //ROS_INFO("Head Pan Direction: [%d], Head Pan Speed: [%d]", msg->data[1],msg->data[0]);//direction & speed in Array elements

        double panAngle;              //radians
  
        int offSetAngle = msg->data[0] - 85;
        float panAngleDegrees = offSetAngle;
        panAngle = (3.14159*panAngleDegrees)/180.0;
  
        std_msgs::Float64 _panAngle;
        
        _panAngle.data = panAngle;
        
        gazebo_pub_.publish(_panAngle);
        
        ROS_INFO("Head Pan Direction (radians): %.2f",_panAngle.data);
        //ROS_INFO("Angle degrees : %d",msg->data[0]);
    }
    
}

int main(int argc, char **argv)
{
  //Note that order of ros::init, node initialization and further functions for node operation is important.
  ros::init(argc, argv, "mbot_head_pan_node");
 
  ros::NodeHandle np_;
  
  ros::Rate rrate(10);
  
  mbot_head_pan_controller::MbotHeadPanControllerServer server_node_pan;

  ROS_INFO("Node initialized.");

  while (ros::ok())
  {
    ros::spinOnce();
    rrate.sleep();
  }
  return EXIT_SUCCESS;
}

