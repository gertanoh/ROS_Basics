#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>



// robot can only move linearly in X and angularly in Z
// values are btw 0 and 1

float linear_x = 0;
float angular_z = 0;

void ReadLaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // set the readings values
    // robot scans at 180°
    // values range from right (index 0) to left (index len(ranges) - 1))
    int middle = msg->ranges.size() / 2;
    //ROS_INFO("Middle : %lu", middle);
    
    // Wall is 1 meter ahead
    if (msg->ranges[360] > 1)
    {
        // Go straight
        linear_x = 0.1;
        angular_z = 0.0;
    }
    else 
    {
        linear_x = 0.0;
        angular_z = 0.2;
    }
    if(msg->ranges[0] < 0.5)
    {
        // Go Left
        linear_x = 0.0;
        angular_z = 0.3;
    }   
        
    if(msg->ranges[719] < 0.5)
    {
        // Go Right
        linear_x = 0.0;
        angular_z = -0.3;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "project_node");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber sub = nh.subscribe("/kobuki/laser/scan", 1000, ReadLaserScanCallback);
    
    ros::Rate loop_rate(2);
    
    // create msg for pub 
    geometry_msgs::Twist pose;
    pose.linear.x = 0;
    pose.linear.z = 0;
    
    while(ros::ok())
    {
        pub.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
        
        // modify msg value according to reading from lase scan
        pose.linear.x = linear_x ;
        pose.angular.z = angular_z ;
        
    }
    
    return 0;
}