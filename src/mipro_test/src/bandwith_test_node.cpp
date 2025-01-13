#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <iostream>

cv::Mat cv_image1;
cv::Mat cv_image2;

void publish(cv_bridge::CvImage& br_image, ros::Publisher& publisher);

int main(int argc, char **argv){

  ros::init(argc, argv, "bandwith_test_node");
  ros::NodeHandle nh;

  // Create a publisher for sensor_msgs/Image
  ros::Publisher image_pub1 = nh.advertise<sensor_msgs::Image>("bandwidth_test1", 10);
  ros::Publisher image_pub2 = nh.advertise<sensor_msgs::Image>("bandwidth_test2", 10);
  ros::Publisher image_pub3 = nh.advertise<sensor_msgs::Image>("bandwidth_test3", 10);
  ros::Publisher image_pub4 = nh.advertise<sensor_msgs::Image>("bandwidth_test4", 10);
  ros::Publisher image_pub5 = nh.advertise<sensor_msgs::Image>("bandwidth_test5", 10); 
  ros::Publisher image_pub6 = nh.advertise<sensor_msgs::Image>("bandwidth_test6", 10);
  ros::Publisher image_pub7 = nh.advertise<sensor_msgs::Image>("bandwidth_test7", 10);
  ros::Publisher image_pub8 = nh.advertise<sensor_msgs::Image>("bandwidth_test8", 10);
  ros::Publisher image_pub9 = nh.advertise<sensor_msgs::Image>("bandwidth_test9", 10);
  ros::Publisher image_pub10 = nh.advertise<sensor_msgs::Image>("bandwidth_test10", 10);

  // Set loop rate (publishing frequency)
  ros::Rate loop_rate(10); // 4 Hz => 0,5 x 8 x 10 => 40 Mbps

  // Load the image using OpenCV (replace with your image file path)
  std::string image_path1 = "src/mipro_test/resources/test_image1.jpg";
  std::string image_path2 = "src/mipro_test/resources/test_image2.jpg";
  cv_image1 = cv::imread(image_path1, cv::IMREAD_COLOR);
  cv_image2 = cv::imread(image_path2, cv::IMREAD_COLOR);

  if (cv_image1.empty()){
    ROS_ERROR("Could not open or find the image at path: %s", image_path1.c_str());
    return -1;
  }

  if (cv_image2.empty()){
    ROS_ERROR("Could not open or find the image at path: %s", image_path2.c_str());
    return -1;
  }

  // Convert the OpenCV image to a ROS Image message
  cv_bridge::CvImage cv_bridge_image;
  cv_bridge_image.header = std_msgs::Header();
  cv_bridge_image.header.frame_id = "test_image";
  cv_bridge_image.encoding = sensor_msgs::image_encodings::BGR8; 
  cv_bridge_image.image = cv_image1;

  // Main loop to publish image
  while (ros::ok()){
    publish(cv_bridge_image, image_pub1);
    publish(cv_bridge_image, image_pub2);
    publish(cv_bridge_image, image_pub3);
    publish(cv_bridge_image, image_pub4);
    publish(cv_bridge_image, image_pub5);
    publish(cv_bridge_image, image_pub6);
    publish(cv_bridge_image, image_pub7);
    publish(cv_bridge_image, image_pub8);
    publish(cv_bridge_image, image_pub9);
    publish(cv_bridge_image, image_pub10);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


void publish(cv_bridge::CvImage& br_image, ros::Publisher& publisher) {
    int random_number = rand() % 100 + 1;
    if (random_number % 2 == 0) {
        br_image.image = cv_image1;
    } else {
        br_image.image = cv_image2;
    }
    br_image.header.stamp = ros::Time::now();
    publisher.publish(br_image.toImageMsg());
}