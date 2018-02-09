#include <ros/ros.h>
#include <iostream>
#include "MonocularVisualOdometer.hpp"


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "mono_vo_ros");
  ROS_INFO("Starting mono_vo_ros-node with name %s", ros::this_node::getName().c_str());
  ros::NodeHandle nodeHandle;
  ros::NodeHandle localNodeHandle("~");

  // Create a VisualOdometer instance
  mvo::MonocularVisualOdometer monocularVisualOdometer(&nodeHandle, &localNodeHandle);
  
  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
