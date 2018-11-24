#include <ros/ros.h>
#include <ardrone_as/ArdroneAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

int nImage = 0;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const ardrone_as::ArdroneResultConstPtr& result)
{
  ROS_INFO("[State Result]: %s", state.toString().c_str());
  ROS_INFO("The Action has been completed");
  //ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

void feedbackCb(const ardrone_as::ArdroneFeedbackConstPtr& feedback)
{
  ROS_INFO("[Feedback] image n.%d received", nImage);
  ++nImage;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_action_client");
  ros::NodeHandle nh;
  
  
  actionlib::SimpleActionClient<ardrone_as::ArdroneAction> 
    client("ardrone_action_server", true);
  client.waitForServer();

  ardrone_as::ArdroneGoal goal;
  goal.nseconds = 10;
  
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  //client.waitForResult();
  
  // publisher to move around the robot while it takes pictures
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  // it seems like robot must take off before being moved
  ros::Publisher takeoff = nh.advertise<std_msgs::Empty>("/drone/takeoff", 1000);
  std_msgs::Empty takeoff_var;
  geometry_msgs::Twist var;
  
  
  ros::Rate loop_rate(2);
  actionlib::SimpleClientGoalState state_result = client.getState();
  ROS_INFO("[State Result]: %s", state_result.toString().c_str());
    
    
  // Take off 
  int ind = 0;
  while( ind < 2)
  {
    takeoff.publish(takeoff_var);
    loop_rate.sleep();
    ++ind;
  }
    
  while ( state_result == actionlib::SimpleClientGoalState::ACTIVE ||
    state_result == actionlib::SimpleClientGoalState::PENDING )
  {
    ROS_INFO("Doing Stuff while waiting for the Server to give a result...");
    
    
    var.linear.x = 0.2;
    var.angular.z = 0.2;
    pub.publish(var);
    
    loop_rate.sleep();
    state_result = client.getState();
    ROS_INFO("[State Result]: %s", state_result.toString().c_str());
  }
  
  ROS_INFO("Finished ");
  
  // stop
  var.linear.x = 0;
  var.angular.z = 0;
  pub.publish(var);
  
  ros::spinOnce();
  return 0;
}