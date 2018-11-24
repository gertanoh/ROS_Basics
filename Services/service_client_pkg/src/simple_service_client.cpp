#include <ros/ros.h>
//#include "gazebo_msgs/DeleteModel.h"
#include <ros/package.h>
#include <iri_wam_reproduce_trajectory/ExecTraj.h>


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "service client");
    ros::NodeHandle nh;
    
    // Create the connection to the service client of gazebo model
    ros::ServiceClient traj_service = 
        nh.serviceClient<iri_wam_reproduce_trajectory::ExecTraj>
        ("execute_trajectory");
    
    iri_wam_reproduce_trajectory::ExecTraj srv; // Object of type deletemodel
    
    srv.request.file = ros::package::getPath("iri_wam_reproduce_trajectory")
        +"/config/get_food.txt";
    
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