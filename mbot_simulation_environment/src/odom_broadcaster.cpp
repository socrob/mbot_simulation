#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class OdomBroadcaster
{
public:
    OdomBroadcaster ()
    {
        sub_ = n_.subscribe ( "/odom", 100, &OdomBroadcaster::callback, this );
    }
    
    
    
    void callback ( const nav_msgs::Odometry::ConstPtr& msg )
    {
        odom_quat_ = msg -> pose.pose.orientation;
        
        odom_trans_.header.stamp = msg -> header.stamp;
        odom_trans_.header.frame_id = "odom";
        odom_trans_.child_frame_id = "base_footprint"; //base_link
        
        odom_trans_.transform.translation.x = msg -> pose.pose.position.x;
        odom_trans_.transform.translation.y = msg -> pose.pose.position.y;
        odom_trans_.transform.translation.z = msg -> pose.pose.position.z;
        odom_trans_.transform.rotation = odom_quat_;
        
        odom_broadcaster_.sendTransform ( odom_trans_ );
    }
    
    
    
private:
    ros::NodeHandle n_;
    
    ros::Subscriber sub_;
    
    tf::TransformBroadcaster odom_broadcaster_;
    geometry_msgs::Quaternion odom_quat_;
    geometry_msgs::TransformStamped odom_trans_;
    
};



int main ( int argc, char *argv[] )
{
  ros::init ( argc, argv, "odom_broadcaster" );
  
  OdomBroadcaster OB;

  ros::spin ();

  return 0;
}
