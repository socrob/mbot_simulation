/*
 * Copyright [2017] <Instituto Superior Tecnico>  
 *
 * Author: Parth Chopra (parthc@umich.edu)
 * Heavily refactored by: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * This node is a republisher used to make topic even between the real mbot
 * and the simulated one.
 *
 * Receives a low level cmd_head command to move the robot's neck and
 * publishes the correspoding gazebo command.
 *
 *
 * More in detail:
 *
 * Listens to std_msgs UInt8MultiArray topic and extracts from that vector
 * the first two elements, assuming that the first is desired position and
 * the second being desired speed, then republishes only the position to a
 * controller in the form of std_msgs Float64.
 *
 */

#ifndef MBOT_HEAD_PAN_H
#define MBOT_HEAD_PAN_H

#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float64.h>

class MbotHeadPanControllerServer
{
    public:
        MbotHeadPanControllerServer();

        void headPanCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
        
    private:
        ros::NodeHandle np_;
        ros::Subscriber cmd_head_sub_;
        ros::Publisher gazebo_pub_;
};

#endif
