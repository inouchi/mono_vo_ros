/*

   The MIT License

   Copyright (c) 2015 Avi Singh

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.

*/

#pragma once

// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// C++
#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <cmath>
#include <sstream>
#include <fstream>
#include <string>

#include "RotationConverter.hpp"

namespace mvo
{
  enum FeatureDetectionMethod
  {
    FAST,
    SIFT,
    SURF,
    ORB,
    AKAZE
  };

  // Set parameters
  const int MIN_NUM_FEAT = 2000;
  const double FOCAL = 718.8560;  // Focal length of the camera
  const cv::Point2d PP(607.1928, 185.2157);  // Principle point of the camera
  const double OFFSET_ROLL   = -1.0 * M_PI / 2.0;
  const double OFFSET_PITCH  = -1.0 * M_PI / 2.0;
  const double OFFSET_YAW    = 0.0;
  const std::string FILE_PATH   = ros::package::getPath("mono_vo_ros"); 
  const std::string KITTI_FILE  = FILE_PATH + "/00.txt";  // Uses to compute scales with ground truth of KITTI dataset 
  const std::string RESULT_FILE = FILE_PATH + "/result.csv"; 


  class MonocularVisualOdometer
  {

    public:
      MonocularVisualOdometer(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr);
      ~MonocularVisualOdometer();

      // TODO: Finds some algorithms that can campute scales without ground truth
      // Computes scales with ground truth of KITTI dataset
      double getAbsoluteScale(std::string filePath, int frameId);

      void getQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion& quaternionMsg);

      // Uses KLT algorithm assume that a point in the nearby space
      // This function automatically gets rid of points for which tracking fails
      void featureTracking(cv::Mat prevImage, cv::Mat currImage, std::vector<cv::Point2f>& prevFeatures, 
	  std::vector<cv::Point2f>& currFeatures, std::vector<unsigned char>& status);

      // Detects a feature from a image using a selected feature detection method 
      void featureDetection(cv::Mat image, std::vector<cv::Point2f>& features, 
	                    FeatureDetectionMethod method = FeatureDetectionMethod::FAST);

      void imageCb(const sensor_msgs::ImageConstPtr& imageMsg);


    private:
      ros::NodeHandle* nodeHandlePtr_;
      ros::NodeHandle* localNodeHandlePtr_;

      image_transport::ImageTransport it_;
      image_transport::Subscriber imageSub_;
      geometry_msgs::PoseArray results_;
      ros::Publisher visualOdometryPub_;

      cv::Mat prevImage_, currImage_;
      std::vector<cv::Point2f> prevFeatures_, currFeatures_;  // Vectors to store the coordinates of the feature points
      bool isReady_;    // Whether it is ready for calclating visual odometry
      cv::Mat Rf_, tf_; // The final rotation and tranlation vectors containing
      size_t numReceiveTopic_;

  };

}  // namespace mvo
