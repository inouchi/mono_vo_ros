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

#include "VisualOdometer.hpp"

VisualOdometer::VisualOdometer(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr)
  : it_(*nodeHandlePtr)
{
  // Set node handle ptrs
  nodeHandlePtr_      = nodeHandlePtr;
  localNodeHandlePtr_ = localNodeHandlePtr;

  // Initialize Sibscriber and Publishers
  imageSub_ = it_.subscribe("camera/image_raw", 1000, &VisualOdometer::imageCb, this);
  //visualOdometryPub_ = nodeHandlePtr_->advertise<nav_msgs::Odometry>("mono_vo_ros/visual_odometry", 1);
  visualOdometryPub_ = nodeHandlePtr_->advertise<geometry_msgs::PoseArray>("mono_vo_ros/visual_odometry", 1);

  // Initialize values
  isReady_ = false;
  numReceiveTopic_ = 0;
}


VisualOdometer::~VisualOdometer()
{

}


double VisualOdometer::getAbsoluteScale(std::string filePath, int frameId)
{
  std::string line;
  int i = 0;
  std::ifstream file(filePath);
  double x = 0, y = 0, z = 0;
  double x_prev, y_prev, z_prev;

  if (file.is_open())
  {

    while ((getline(file, line)) && (i <= frameId))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;

      std::istringstream in(line);

      for (int j = 0; j < 12; ++j)
      {
        in >> z ;
        if (j == 7) y = z;
        if (j == 3)  x = z;
      }
      
      i++;
    }

    file.close();
  }
  else
  {
    ROS_ERROR("Unable to open %s", filePath.c_str());
    return 0;
  }

  return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));
}


void VisualOdometer::featureTracking(cv::Mat prevImage, cv::Mat currImage, std::vector<cv::Point2f>& prevFeatures,
                                     std::vector<cv::Point2f>& currFeatures, std::vector<unsigned char>& status)
{ 
  std::vector<float> err;					
  cv::Size winSize = cv::Size(21, 21);																								
  cv::TermCriteria termcrit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
  cv::calcOpticalFlowPyrLK(prevImage, currImage, prevFeatures, currFeatures, status, err, winSize, 3, termcrit, 0, 0.001);

  // Getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for(int i = 0; i < status.size(); ++i)
  {  
    cv::Point2f pt = currFeatures.at(i- indexCorrection);

    if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0))
    {

      if((pt.x < 0) || (pt.y < 0))
      {
	status.at(i) = 0;
      }

      prevFeatures.erase(prevFeatures.begin() + (i - indexCorrection));
      currFeatures.erase(currFeatures.begin() + (i - indexCorrection));
      indexCorrection++;
    }
  }

}


void VisualOdometer::featureDetection(cv::Mat image, std::vector<cv::Point2f>& features)
{
  std::vector<cv::KeyPoint> keypoints;
  int fastThreshold = 20;
  bool nonmaxSuppression = true;
  FAST(image, keypoints, fastThreshold, nonmaxSuppression);
  cv::KeyPoint::convert(keypoints, features, std::vector<int>());
}


void VisualOdometer::imageCb(const sensor_msgs::ImageConstPtr& imageMsg)
{
  cv_bridge::CvImagePtr inputImagePtr;
  numReceiveTopic_++;

  try
  {
    inputImagePtr = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Convert RGB image into gray scale image
  cv::Mat grayScaleImage;
  cv::cvtColor(inputImagePtr->image, grayScaleImage, cv::COLOR_BGR2GRAY);

  // Ready for calculating visual odometry
  if (isReady_ == false)
  {
    prevImage_ = grayScaleImage.clone();
    isReady_ = true;
    
    return;
  }

  // Set a current Image
  currImage_ = grayScaleImage.clone();

  // Feature detection, tracking
  featureDetection(prevImage_, prevFeatures_);  // Detect features in prevImage_
  std::vector<unsigned char> status;
  featureTracking(prevImage_, currImage_, prevFeatures_, currFeatures_, status);  // Track those features to currImage_

  // TODO: Add a fucntion to load these values directly from KITTI's calib files
  // WARNING: Different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
  // Recovering the pose and the essential matrix
  cv::Mat E, R, t, mask;
  E = cv::findEssentialMat(currFeatures_, prevFeatures_, visualOdometer::FOCAL, visualOdometer::PP, cv::RANSAC, 0.999, 1.0, mask);
  cv::recoverPose(E, currFeatures_, prevFeatures_, R, t, visualOdometer::FOCAL, visualOdometer::PP, mask);


  // Compute scale from ground truth of KITII dataset
  double scale = getAbsoluteScale(visualOdometer::FILE_PATH, numReceiveTopic_);

  if (Rf_.empty() && tf_.empty())
  {
    Rf_ = R.clone();
    tf_ = t.clone();
  }
  else
  {
    if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)))
    {
      tf_ = tf_ + scale * (Rf_ * t);
      Rf_ = R * Rf_;
    }
    else
    {

    }
    //tf_ = tf_ + scale * (Rf_ * t);
    //Rf_ = R * Rf_;
  }

  // Set result of visual odometry
  geometry_msgs::Pose tmp;
  tmp.position.x = tf_.at<double>(0);
  tmp.position.y = tf_.at<double>(2);
  tmp.position.z = 0.0;//tf_.at<double>(1);
  if (results_.poses.empty())
  {
    results_.poses.push_back(tmp);
    results_.header.frame_id = "camera";
  }
  else
  {
    results_.poses.push_back(tmp);
  }

  // Publish visual odometry
  visualOdometryPub_.publish(results_);

  // A redetection is triggered in case the number of feautres being trakced go below a particular threshold
  if (prevFeatures_.size() < visualOdometer::MIN_NUM_FEAT)
  {
    //std::cout << "Number of tracked features reduced to " << prevFeatures.size() << std::endl;
    //std::cout << "Trigerring redection" << std::endl;
    featureDetection(prevImage_, prevFeatures_);
    featureTracking(prevImage_, currImage_, prevFeatures_, currFeatures_, status);
  }

  prevImage_ = currImage_.clone();
  prevFeatures_ = currFeatures_;
}
