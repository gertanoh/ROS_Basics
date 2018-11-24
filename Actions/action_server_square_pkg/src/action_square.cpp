#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/TestAction.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>


class MoveSquareAction

{
    
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise 
    // strange error occurs.
    actionlib::SimpleActionServer<actionlib::TestAction> as_; 
    std::string action_name_;
    // create messages that are used to publish feedback and result
    actionlib::TestFeedback feedback_;
    actionlib::TestResult result_;
    
    
    // ROS Variables
    ros::Rate loop_rate;
    ros::Publisher pub_move;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    
    
    
public:
    MoveSquareAction(std::string name, int rate_in) :
        as_(nh_, name, boost::bind(&MoveSquareAction::executeCB, this, _1), false),
        action_name_(name), loop_rate(rate_in)
    {
        as_.start();
        
        pub_move = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        pub_takeoff = nh_.advertise<std_msgs::Empty>("/drone/takeoff", 1000);
        pub_land = nh_.advertise<std_msgs::Empty>("/drone/land", 1000);
    }
    
    void executeCB(const actionlib::TestGoalConstPtr& goal)
    {
        bool success = true;
        
        ROS_INFO("Perform square move");
        
        takeoff();
        
        for(int index = 1 ; index <= 4; ++index)
        {
            // check if preempted
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }
            
            // perform on move of square
            move(goal->goal);
            turn();
            // feedback the side of the square
            feedback_.feedback = index;
            as_.publishFeedback(feedback_);
            loop_rate.sleep();
        }
        
        if (success)
        {
            result_.result = feedback_.feedback;
            ROS_INFO("%s: succeeded", action_name_.c_str());
            as_.setSucceeded(result_);
            stop();
            land();
        }
    }
    
    void move(int side)
    {
        // by travelling at full speed with rate 1 Hz
        int index = 0;
        geometry_msgs::Twist var;
        
        while(index < side)
        {
            var.linear.x = 1;
            var.angular.z = 0;
            pub_move.publish(var);
            index++;
            loop_rate.sleep();
        }
    }
    
    void turn()
    {
        // by travelling at full speed with rate 1 Hz
        int index = 0;
        geometry_msgs::Twist var;
        
        // this value was a guess
        int turn_sec = 2;
        
        while(index < turn_sec)
        {
            var.linear.x = 0;
            var.angular.z = 0.5;
            pub_move.publish(var);
            index++;
            loop_rate.sleep();
        }
    }
    
    void stop()
    {
        // by travelling at full speed with rate 1 Hz
        int index = 0;
        geometry_msgs::Twist var;
        
        while(index < 2)
        {
            var.linear.x = 0;
            var.angular.z = 0;
            pub_move.publish(var);
            index++;
            loop_rate.sleep();
        }
    }
    
    void land()
    {
        // by travelling at full speed with rate 1 Hz
        int index = 0;
        std_msgs::Empty var;
        
        while(index < 4)
        {
            pub_land.publish(var);
            index++;
            loop_rate.sleep();
        }
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MoveSquareAction");
    MoveSquareAction move_square_action("MoveSquare", 1);
    ros::spin();
    
    return 0;
}