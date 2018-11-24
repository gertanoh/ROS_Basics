#include <ros/ros.h>
// Service message header
#include "std_srvs/Empty.h"


// call back function of service

bool my_callback(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res)
{
    // print status data
    ROS_INFO("My callback has been called");   
    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_server");
    ros::NodeHandle nh;
    
    // register service
    ros::ServiceServer my_service = 
        nh.advertiseService("/my_service", my_callback);
    
    ros::spin(); // maintain service open
    
    return 0;
}
