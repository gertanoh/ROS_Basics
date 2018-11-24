#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Topic publisher");
    ros::NodeHandle nh;
    
    //ros::Publisher pub = nh.advertise<std_msgs::Int32>("cmd_vel", 1000);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(2);
    
    std_msgs::Int32 count;
    geometry_msgs::Twist var;
    var.linear.x = 0.6;
    var.angular.z = 0.6;
    count.data = 0;
    
    while(ros::ok())
    {
        pub.publish(var);
        ros::spinOnce();
        loop_rate.sleep();
        
        ++count.data;
    }
    
    return 0;
}