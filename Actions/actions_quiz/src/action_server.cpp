#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include "actions_quiz/CustomActionMsgAction.h"


class ActionQuiz

{
    
    ros::NodeHandle nh;
    using Server = actionlib::SimpleActionServer<actions_quiz::CustomActionMsgAction>; 
    Server as_;
    std::string action_name_;
    
    actions_quiz::CustomActionMsgFeedback feedback_;
    actions_quiz::CustomActionMsgResult result_;
    
    // odometry
    float x_meas;
    float y_meas;
    float z_meas;
    
    // ROS Variables
    ros::Rate loop_rate;
    ros::Publisher pub_move;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    
    
    bool running_;
    
    
    
public:
    ActionQuiz(std::string name, int rate_in): as_(nh, name, 
        boost::bind(&ActionQuiz::executeCB, this, _1), false), action_name_(name),
        loop_rate(rate_in)
    {
        as_.start();
        
        running_ = false;
        
        pub_move = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        pub_takeoff = nh.advertise<std_msgs::Empty>("/drone/takeoff", 1000);
        pub_land = nh.advertise<std_msgs::Empty>("/drone/land", 1000);
        
    }
    
    
    void executeCB(const actions_quiz::CustomActionMsgGoalConstPtr& goal)
    {
        bool success = true;
        
        
        if(!running_)
        {
            takeoff();
            running_= true;
        }
        
        // check if preempted
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            as_.setPreempted();
            success = false;
            return;
        }
        if (goal->goal_action == "UP")
        {
            moveUp();
        }
        else if (goal->goal_action == "Down")
        {
            moveDown();
        }
        feedback_.feedback_action = goal->goal_action;
        as_.publishFeedback(feedback_);
        loop_rate.sleep();
        
        if (success)
        {
            ROS_INFO("%s: succeeded", action_name_.c_str());
            as_.setSucceeded();
            stop();
        }
    }
    
    
    void moveUp()
    {
        // by travelling at full speed with rate 1 Hz
        // move up 1 meter
        
        geometry_msgs::Twist var;
        
        var.linear.x = 0;
        var.linear.y = 0;
        var.linear.z = 1;
        
        var.angular.x = 0;
        var.angular.y = 0;
        var.angular.z = 0;
        pub_move.publish(var);
        
        loop_rate.sleep();
    }
    
    
    void moveDown()
    {
        // by travelling at full speed with rate 1 Hz
        // move down 1 meter
        bool res = false;
        
        
        geometry_msgs::Twist var;
        
        var.linear.x = 0;
        var.linear.y = 0;
        var.linear.z = -1;
        
        var.angular.x = 0;
        var.angular.y = 0;
        var.angular.z = 0;
        pub_move.publish(var);
        res = true;
        
        loop_rate.sleep();
        
    }
    void takeoff()
    {
        // by travelling at full speed with rate 1 Hz
        int index = 0;
        std_msgs::Empty var;
        
        while(index < 4)
        {
            pub_takeoff.publish(var);
            index++;
            loop_rate.sleep();
        }
    }
    
    void stop()
    {
        // by travelling at full speed with rate 1 Hz
        int index = 0;
        geometry_msgs::Twist var;
        
        while(index < 4)
        {
            var.linear.x = 0;
            var.linear.y = 0;
            var.linear.z = 0;
            
            var.angular.x = 0;
            var.angular.y = 0;
            var.angular.z = 0;
            pub_move.publish(var);
            index++;
            loop_rate.sleep();
        }
    }
};


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "ActionServer");
  ActionQuiz quiz("action_custom_msg_as", 1);
  ros::spin();
}

