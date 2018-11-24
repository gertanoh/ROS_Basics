#include <ros/ros.h>
#include "std_srvs/Empty.h"


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "service square client");
    ros::NodeHandle nh;
    
    // Create the connection to the service client of gazebo model
    ros::ServiceClient traj_service = 
        nh.serviceClient<std_srvs::Empty>
        ("/perform_square");
    
    std_srvs::Empty srv;
    
    if (traj_service.call(srv))
    {
        ROS_INFO("Trajectory Executed");
    }
    else 
    {
        ROS_ERROR("Failed to call service");
        return -1;
    }
    
    
    return 0;
    
}