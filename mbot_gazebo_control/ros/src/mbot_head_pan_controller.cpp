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

#include <mbot_gazebo_control/mbot_head_pan.h>

MbotHeadPanControllerServer::MbotHeadPanControllerServer(): np_("")
{
    gazebo_pub_  = np_.advertise<std_msgs::Float64>("mbot_head_position_controller/command", 1);
    cmd_head_sub_ = np_.subscribe("cmd_head", 1, &MbotHeadPanControllerServer::headPanCallback,this);
}

void MbotHeadPanControllerServer::headPanCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    ROS_DEBUG("Head Pan Direction: [%d], Head Pan Speed: [%d]", msg->data[1],msg->data[0]); // direction & speed in Array elements

    std_msgs::Float64 pan_angle;

    pan_angle.data = (3.14159 * (float)(msg->data[0] - 85)) / 180.0;

    gazebo_pub_.publish(pan_angle);

    ROS_INFO("Head Pan Direction (radians): %.2f", pan_angle.data);
    ROS_DEBUG("Angle degrees : %d", msg->data[0]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mbot_head_pan_node");
    ROS_INFO("Node is going to initialize...");

    // create object of the node class (MbotHeadPanControllerServer)
    MbotHeadPanControllerServer server_node_pan;

    // setup node frequency
    double node_frequency = 10.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 10.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
