#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>

void counterCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("%d", msg->data);
}

void counterCallbackOdom(const nav_msgs::Odometry::ConstPtr& odom)
{
    ROS_INFO("Id : %s", odom->child_frame_id.c_str());
    ROS_INFO("X : %f", odom->pose.pose.position.x);
    ROS_INFO("Y : %f", odom->pose.pose.position.y);
    ROS_INFO("Z: %f", odom->pose.pose.position.z);
    
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Topic Subscriber");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("odom", 
        1000, counterCallbackOdom);
    
    ros::spin();
    
    return 0;
}