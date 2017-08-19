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

#include <mbot_gazebo_control/mbot_head_gazebo_republisher.h>

MbotHeadGazeboRepublisher::MbotHeadGazeboRepublisher (): nh_ (""), neck_motion_request_received_ (false), head_cam_tilt_request_received_ (false)
{
    // subscriptions
    cmd_head_pan_sub_ = nh_.subscribe ("cmd_head", 1, &MbotHeadGazeboRepublisher::headPanCallback,this);
    cmd_head_cam_tilt_sub_ = nh_.subscribe ("cmd_head_camera_motor", 1, &MbotHeadGazeboRepublisher::headCamTiltCallback,this);

    // publications
    mbot_neck_pub_  = nh_.advertise <std_msgs::Float64> ("mbot_head_position_controller/command", 1);
    mbot_head_cam_tilt_pub_  = nh_.advertise <std_msgs::Float64> ("mbot_head_camera_position_controller/command", 1);
}

void MbotHeadGazeboRepublisher::headPanCallback (const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    pan_angle_ = *msg;
    neck_motion_request_received_ = true;
}

void MbotHeadGazeboRepublisher::republishNeckData ()
{
    ROS_DEBUG ("Head Pan Direction: [%d], Head Pan Speed: [%d]", pan_angle_.data[1], pan_angle_.data[0]);

    std_msgs::Float64 pan_angle;

    pan_angle.data = (3.14159 * (float)(pan_angle_.data[0] - 85)) / 180.0;

    mbot_neck_pub_.publish (pan_angle);

    ROS_INFO ("Head Pan Direction (radians): %.2f", pan_angle.data);
    ROS_DEBUG ("Angle degrees : %d", pan_angle_.data[0]);
}

void MbotHeadGazeboRepublisher::headCamTiltCallback (const std_msgs::UInt16::ConstPtr& msg)
{
    head_cam_tilt_angle_ = *msg;
    head_cam_tilt_request_received_ = true;
}

void MbotHeadGazeboRepublisher::republishHeadCamTiltData ()
{
    ROS_DEBUG ("Head cam tilt angle requested : [%d]", head_cam_tilt_angle_.data);

    std_msgs::Float64 tilt_angle;

    // 1.0 corresponds to 75
    // 1.4 corresponds roughly to 95
    // 1. transform from deg to rad (deg * pi / 180) and substract offset
    tilt_angle.data = ((float)(head_cam_tilt_angle_.data) * M_PI / 180.0) - 0.309 ;

    mbot_head_cam_tilt_pub_.publish (tilt_angle);

    ROS_INFO ("Tilting head cam at an angle of (radians): %.2f", tilt_angle.data);
}

void MbotHeadGazeboRepublisher::update ()
{
    // check if a neck pan request has been received
    if (neck_motion_request_received_)
    {
        // lower flag
        neck_motion_request_received_ = false;

        // process request
        republishNeckData();
    }

    if (head_cam_tilt_request_received_)
    {
        // lower flag
        head_cam_tilt_request_received_ = false;

        // process request
        republishHeadCamTiltData();
    }
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "mbot_head_pan_node");
    ROS_INFO ("Node is going to initialize...");

    // create object of the node class (MbotHeadGazeboRepublisher)
    MbotHeadGazeboRepublisher republisher_instance;

    // setup node frequency
    double node_frequency = 10.0;
    ros::NodeHandle nh ("~");
    nh.param ("node_frequency", node_frequency, 10.0);
    ROS_INFO ("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");

    while (ros::ok ())
    {
        // listen to callbacks
        ros::spinOnce ();

        // update
        republisher_instance.update ();

        // sleep to control the node frequency
        loop_rate.sleep ();
    }

    return 0;
}
