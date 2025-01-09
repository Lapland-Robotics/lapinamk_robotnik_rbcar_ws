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
  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("bandwidth_test", 1000);

  // Set loop rate (publishing frequency)
  ros::Rate loop_rate(4); // 4 Hz => 4.1 x 8 x 4 => 131,2 Mbps

  // Load the image using OpenCV (replace with your image file path)
  std::string image_path = "src/mipro_test/resources/test_image.jpg";
  cv::Mat cv_image = cv::imread(image_path, cv::IMREAD_COLOR); // Read the image as a color image

  if (cv_image.empty()){
    ROS_ERROR("Could not open or find the image at path: %s", image_path.c_str());
    return -1;
  }

  // Convert the OpenCV image to a ROS Image message
  cv_bridge::CvImage cv_bridge_image;
  cv_bridge_image.header = std_msgs::Header();                   // Add header
  cv_bridge_image.header.stamp = ros::Time::now();               // Timestamp
  cv_bridge_image.header.frame_id = "camera";                    // Set the frame id (e.g., "camera")
  cv_bridge_image.encoding = sensor_msgs::image_encodings::BGR8; // Encoding type for color image
  cv_bridge_image.image = cv_image;

  // Main loop to publish image
  while (ros::ok()){
    image_pub.publish(cv_bridge_image.toImageMsg()); // Publish the image as a ROS message
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}