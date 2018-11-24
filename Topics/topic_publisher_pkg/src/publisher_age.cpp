#include <ros/ros.h>
#include <topic_publisher_pkg/Age.h>
// an header is created for the message

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher_age_node");
    ros::NodeHandle nh;
    
    
    ros::Publisher pub = nh.advertise<topic_publisher_pkg::Age>
        ("age_info", 1000);
    ros::Rate loop_rate(2);
    
    topic_publisher_pkg::Age age;
    age.years = 2.0 ;
    age.months = 11 ;
    age.days = 22 ;
    
    while(ros::ok())
    {
        pub.publish(age);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}