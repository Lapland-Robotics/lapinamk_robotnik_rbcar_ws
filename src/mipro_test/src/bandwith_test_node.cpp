#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char **argv){

  ros::init(argc, argv, "bandwith_test_node");
  ros::NodeHandle nh;

  // Create a publisher for sensor_msgs/Image
  ros::Publisher image_pub1 = nh.advertise<sensor_msgs::Image>("bandwidth_test1", 20, ros::TransportHints().tcpNoDelay());
  ros::Publisher image_pub2 = nh.advertise<sensor_msgs::Image>("bandwidth_test2", 20, ros::TransportHints().tcpNoDelay());
  ros::Publisher image_pub3 = nh.advertise<sensor_msgs::Image>("bandwidth_test3", 20, ros::TransportHints().tcpNoDelay());
  ros::Publisher image_pub4 = nh.advertise<sensor_msgs::Image>("bandwidth_test4", 20, ros::TransportHints().tcpNoDelay());
  ros::Publisher image_pub5 = nh.advertise<sensor_msgs::Image>("bandwidth_test5", 20, ros::TransportHints().tcpNoDelay());

  // Set loop rate (publishing frequency)
  ros::Rate loop_rate(10); // 4 Hz => 0,5 x 8 x 10 => 40 Mbps

  // Load the image using OpenCV (replace with your image file path)
  std::string image_path = "src/mipro_test/resources/test_image1.jpg";
  cv::Mat cv_image = cv::imread(image_path, cv::IMREAD_COLOR);

  if (cv_image.empty()){
    ROS_ERROR("Could not open or find the image at path: %s", image_path.c_str());
    return -1;
  }

  // Convert the OpenCV image to a ROS Image message
  cv_bridge::CvImage cv_bridge_image;
  cv_bridge_image.header = std_msgs::Header();
  cv_bridge_image.header.frame_id = "test_image";
  cv_bridge_image.encoding = sensor_msgs::image_encodings::BGR8; 
  cv_bridge_image.image = cv_image;

  // Main loop to publish image
  while (ros::ok()){
    cv_bridge_image.header.stamp = ros::Time::now();
    image_pub1.publish(cv_bridge_image.toImageMsg());
    image_pub2.publish(cv_bridge_image.toImageMsg());
    image_pub3.publish(cv_bridge_image.toImageMsg());
    image_pub4.publish(cv_bridge_image.toImageMsg());
    image_pub5.publish(cv_bridge_image.toImageMsg());
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}