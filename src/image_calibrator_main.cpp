#include <ros/ros.h> 
#include "ImageCalibrator.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "image_calibrator");
  ROS_INFO("Starting image_calibrator-node with name %s", ros::this_node::getName().c_str());

  // Create ImageCalibrator instance
  ImageCalibrator imageCalibrator;

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
