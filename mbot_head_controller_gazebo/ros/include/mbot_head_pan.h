#ifndef MBOT_HEAD_PAN_H
#define MBOT_HEAD_PAN_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdlib.h"
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float64.h>

namespace mbot_head_pan_controller
{
#define GAIN_SLOW 20.0
#define GAIN_NORMAL 35.0
#define GAIN_FAST 60.0

#define DEG_0 0.0
#define DEG_45 0.7854 
#define DEG_90 1.5708
#define DEG_135 2.356
#define DEG_180 3.1416



class MbotHeadPanControllerServer
{
private:
  ros::NodeHandle np_;
  ros::Subscriber cmd_head_sub_;
  ros::Publisher gazebo_pub_;

  protected:
  
  public:
  /**
    * @brief MbotHeadPanControllerServer - class constructor
    */
  MbotHeadPanControllerServer();

  /**
    * @brief headPanCallback - callback for semantic message requests
    * @param msg
    */
  void
  headPanCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);

};
// End of namespace mbot_head_pan_controller
}

#endif

