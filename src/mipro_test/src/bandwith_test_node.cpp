
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv){

  ros::init(argc, argv, "bandwith_test_node");
  ros::NodeHandle nh;
  ros::Publisher topic_pub = nh.advertise<std_msgs::String>("bandwith_test", 1000);
  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world ";
    topic_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}