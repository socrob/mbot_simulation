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
#include <std_msgs/UInt16.h>
#include <math.h>

class MbotHeadGazeboRepublisher
{
    public:
        MbotHeadGazeboRepublisher();

        // callback to receive head pan requests in the real robot format
        void headPanCallback (const std_msgs::UInt8MultiArray::ConstPtr& msg);

        // callback to receive head cam tilt requests in the real robot format
        void headCamTiltCallback (const std_msgs::UInt16::ConstPtr& msg);

        // transforms the data into the proper gazebo format and republishes to gazebo simulator
        void republishNeckData ();

        // transforms the data into the proper gazebo format and republishes to gazebo simulator
        void republishHeadCamTiltData ();
        
        // check if any data has been received in the callbacks and if so then republish the content
        void update ();

    private:
        ros::NodeHandle nh_;

        // subscribe to the command to moving the mbot neck (head pan movement)
        ros::Subscriber cmd_head_pan_sub_;

        // subscribe to the command for tilting camera located in the mbot head
        ros::Subscriber cmd_head_cam_tilt_sub_;

        // to republish the data in a suitable way to the gazebo controller
        ros::Publisher mbot_neck_pub_;

        // to republish the data in a suitable way to the gazebo controller
        ros::Publisher mbot_head_cam_tilt_pub_;

        // to store the value recevied via callback
        std_msgs::UInt8MultiArray pan_angle_;

        // to store the value received in the callback
        std_msgs::UInt16 head_cam_tilt_angle_;

        // flag that indicates that a neck pan command was received
        bool neck_motion_request_received_;

        // flag that indicates that the camera on top of the mbot needs to be tilted
        bool head_cam_tilt_request_received_;
};

#endif
