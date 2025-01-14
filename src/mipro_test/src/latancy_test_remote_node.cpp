#include <ros/ros.h>
#include <std_msgs/Header.h>

// Publisher and Subscriber handles
ros::Publisher publisher;
ros::Subscriber subscriber;

// Timer callback to publish timestamp messages
void publishTimestamp(const ros::TimerEvent&)
{
    std_msgs::Header msg;
    msg.stamp = ros::Time::now();
    publisher.publish(msg);
    ROS_INFO("Published timestamp to /latancy_halfway: %.3f ms", msg.stamp.toSec() * 1000.0);
}

// Callback for latency calculation
void latencyCallback(const std_msgs::Header::ConstPtr& msg)
{
    ros::Time now = ros::Time::now();
    double latency_ms = (now - msg->stamp).toSec() * 1000.0;
    ROS_INFO("Latency calculated from /latancy_fullway: %.3f ms", latency_ms);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "latency_remote_node");
    ros::NodeHandle nh;

    publisher = nh.advertise<std_msgs::Header>("/latancy_halfway", 10);
    subscriber = nh.subscribe("/latancy_fullway", 10, latencyCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), publishTimestamp);
    ros::spin();

    return 0;
}
