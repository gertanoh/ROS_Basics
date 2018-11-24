#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>


int main(int argc, char** argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
        ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    ros::init(argc, argv, "log_demo");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.5);
    
    while(ros::ok())
    {
        ROS_DEBUG("There is a missing droid");
        ROS_INFO("Come on");
        ROS_WARN("Help me please ");
        
        int exhaust_number = rand() % 100;
        int port_number = rand() % 100 + 1;
        
        ROS_ERROR("the thermal exhaust %d, right below the main port %d", 
            exhaust_number, port_number);
        ROS_FATAL("Exploding");
        
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}