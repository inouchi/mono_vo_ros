#pragma once

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class ImageCalibrator
{

  public:
    ImageCalibrator();
    ~ImageCalibrator();

    cv::Mat calibrateImage(const cv::Mat& inputImage);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);


  private:
    ros::NodeHandle nodeHandle_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber imageSub_;
    image_transport::Publisher imagePub_;

};
