#include <ros/ros.h>
#include "custom_srv_msg_pkg/MyCustomServiceMessage.h"


bool my_callback(custom_srv_msg_pkg::MyCustomServiceMessage::Request& req,
                 custom_srv_msg_pkg::MyCustomServiceMessage::Response& res)
{
  ROS_INFO("Request Data==> radius=%f, repetitions=%d", 
    req.radius, req.repetitions); 
  if (req.radius > 5.0)
  {
    res.success = true;
    ROS_INFO("sending back response:true");
  }
  else
  {
    res.success = false;
    ROS_INFO("sending back response:false");
  }
  
  return true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Service_server_Custom_MSG");
    
    ros::NodeHandle nh;
    
    ros::ServiceServer my_service = nh.advertiseService("/my_service",
        my_callback);
        
    ros::spin();
    
    return 0;
}