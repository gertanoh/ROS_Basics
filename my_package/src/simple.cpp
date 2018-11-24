#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FNode"); // Initiate a node called ObiWan
    ros::NodeHandle nh; // handle for ObiWan node, this handle will 
    //initialize the node
    ROS_INFO("I am ready to learn ROS"); // like printf
    ros::Rate loop_rate(2); // loop at 2Hz, rate object
    
    while(ros::ok())
    {
        ROS_INFO("I am ready to laern ");
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}