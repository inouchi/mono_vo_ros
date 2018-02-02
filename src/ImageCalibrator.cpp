#include "ImageCalibrator.hpp"


ImageCalibrator::ImageCalibrator() : it_(nodeHandle_)
{
  // Subscrive to input video feed and publish output video feed
  imageSub_ = it_.subscribe("/camera/image_raw", 1, &ImageCalibrator::imageCb, this);
  imagePub_ = it_.advertise("/image_callibrator/output", 1);
}


ImageCalibrator::~ImageCalibrator()
{

}


cv::Mat ImageCalibrator::calibrateImage(const cv::Mat& inputImage)
{
  cv::Mat outputImage; // Result of a Calibration image
  cv::Mat intrinsic;   // Internal parameters
  cv::Mat distortion;  // Distortion coeffocoent

  // Read parameter file to do distortion correction
  cv::FileStorage fs(ros::package::getPath("image_callibrator") + "/callibration_data.xml", cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    ROS_INFO("callibration_data.xml not fonud.\n");
    exit(1);
  }

  // Set parameter to do distortion correction
  cv::Mat cameraMat, distMat;
  fs["intrinsic"] >> cameraMat;
  fs["distortion"] >> distMat;
  intrinsic = cameraMat;
  distortion = distMat;
  fs.release(); 

  // Distortion correction
  cv::undistort(inputImage, outputImage, intrinsic, distortion);

  return outputImage;
}



void ImageCalibrator::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cvPtr;

  try
  {
    cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Obtain camera image
  cv::Mat cameraImage = cvPtr->image;

  // Calibrate
  cv::Mat imageCalibrated = cameraImage(cameraImage);

  // Publish
  sensor_msgs::ImagePtr foo = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageCalibrated).toImageMsg();
  imagePub_.publish(foo);
}
