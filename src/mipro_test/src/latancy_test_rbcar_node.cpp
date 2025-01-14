#include <ros/ros.h>
#include <std_msgs/Header.h>

// Publisher and Subscriber handles
ros::Publisher publisher;
ros::Subscriber subscriber;

// Callback function to forward messages from /latancy_halfway to /latancy_fullway
void forwardCallback(const std_msgs::Header::ConstPtr& msg)
{
    publisher.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "latency_rbcar_node");
    ros::NodeHandle nh;

    subscriber = nh.subscribe("/latancy_halfway", 10, forwardCallback);
    publisher = nh.advertise<std_msgs::Header>("/latancy_fullway", 10);
    
    ros::spin();
    return 0;
}
