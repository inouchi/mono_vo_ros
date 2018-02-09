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

#include "MonocularVisualOdometer.hpp"

mvo::MonocularVisualOdometer::MonocularVisualOdometer(ros::NodeHandle* nodeHandlePtr, ros::NodeHandle* localNodeHandlePtr)
  : it_(*nodeHandlePtr)
{
  // Set node handle ptrs
  nodeHandlePtr_      = nodeHandlePtr;
  localNodeHandlePtr_ = localNodeHandlePtr;

  // Initialize Sibscriber and Publishers
  imageSub_ = it_.subscribe("camera/image_raw", 10000, &MonocularVisualOdometer::imageCb, this);
  //visualOdometryPub_ = nodeHandlePtr_->advertise<nav_msgs::Odometry>("mono_vo_ros/visual_odometry", 1);
  visualOdometryPub_ = nodeHandlePtr_->advertise<geometry_msgs::PoseArray>("mono_vo_ros/visual_odometry", 1);

  // Initialize values
  isReady_ = false;
  numReceiveTopic_ = 0;
}


mvo::MonocularVisualOdometer::~MonocularVisualOdometer()
{

}


double mvo::MonocularVisualOdometer::getAbsoluteScale(std::string filePath, int frameId)
{
  std::string line;
  int i = 0;
  std::ifstream file(filePath);
  double currX = 0, currY = 0, currZ = 0;
  double prevX, prevY, prevZ;

  if (file.is_open())
  {
    while ((getline(file, line)) && (i <= frameId))
    {
      prevZ = currZ;
      prevX = currX;
      prevY = currY;
      std::istringstream in(line);

      for (int j = 0; j < 12; ++j)
      {
        in >> currZ ;
        if (j == 7)  currY = currZ;
        if (j == 3)  currX = currZ;
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

  return sqrt(pow((currX - prevX), 2) + pow((currY - prevY), 2) + pow((currZ - prevZ), 2));
}


void mvo::MonocularVisualOdometer::getQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion &quaternionMsg)
{
  tf::Quaternion quaternion =tf::createQuaternionFromRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(quaternion, quaternionMsg);
}


void mvo::MonocularVisualOdometer::featureTracking(cv::Mat prevImage, cv::Mat currImage, std::vector<cv::Point2f>& prevFeatures,
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


void mvo::MonocularVisualOdometer::featureDetection(cv::Mat image, std::vector<cv::Point2f>& features, FeatureDetectionMethod method)
{
  // Generate a feature detection
  cv::Ptr<cv::FeatureDetector> detector;

  switch (method)
  {
    case FeatureDetectionMethod::FAST:
      detector = cv::FastFeatureDetector::create();
      break;
    
    case FeatureDetectionMethod::SIFT:
      detector = cv::xfeatures2d::SIFT::create();
      break;

    case FeatureDetectionMethod::SURF:
      detector = cv::xfeatures2d::SURF::create();
      break;

    case FeatureDetectionMethod::ORB:
      detector = cv::ORB::create();
      break;

    case FeatureDetectionMethod::AKAZE:
      detector = cv::AKAZE::create();
      break;

    default:
      break;
  }

  std::vector<cv::KeyPoint> keypoints;
  detector->detect(image, keypoints);
  cv::KeyPoint::convert(keypoints, features);
}


void mvo::MonocularVisualOdometer::imageCb(const sensor_msgs::ImageConstPtr& imageMsg)
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

  // Set a current Image
  currImage_ = grayScaleImage.clone();

  // Ready for calculating visual odometry
  if (isReady_ == false)
  {
    if (numReceiveTopic_ == 1)
    {
      prevImage_ = currImage_.clone();
      std::ofstream(RESULT_FILE);
    
      return;
    }
    else if (numReceiveTopic_ == 2)
    {
      // Feature detection, tracking
      featureDetection(prevImage_, prevFeatures_);  // Detect features in prevImage_
      std::vector<unsigned char> status;
      featureTracking(prevImage_, currImage_, prevFeatures_, currFeatures_, status);  // Track those features to currImage_
   
      // TODO: Add a fucntion to load these values directly from KITTI's calib files
      // WARNING: Different sequences in the KITTI VO dataset have different intrinsic/extyyrinsic parameters
      // Recovering the pose and the essential matrix
      cv::Mat E, R, t, mask;
      E = cv::findEssentialMat(currFeatures_, prevFeatures_, FOCAL, PP, cv::RANSAC, 0.999, 1.0, mask);
      cv::recoverPose(E, currFeatures_, prevFeatures_, R, t, FOCAL, PP, mask);

      Rf_ = R.clone();
      tf_ = t.clone();

      prevImage_ = currImage_.clone();
      prevFeatures_ = currFeatures_;

      isReady_ = true;

      return;
    }

  }

  // Feature tracking
  std::vector<unsigned char> status;
  featureTracking(prevImage_, currImage_, prevFeatures_, currFeatures_, status);  // Track those features to currImage_

  // Recovering the pose and the essential matrix
  cv::Mat E, R, t, mask;
  E = cv::findEssentialMat(currFeatures_, prevFeatures_, FOCAL, PP, cv::RANSAC, 0.999, 1.0, mask);
  cv::recoverPose(E, currFeatures_, prevFeatures_, R, t, FOCAL, PP, mask);

  // Compute scale from ground truth of KITII dataset
  double scale = getAbsoluteScale(KITTI_FILE, numReceiveTopic_ - 1);

  if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)))
  {
    tf_ = tf_ + scale * (Rf_ * t);
    Rf_ = R * Rf_;
  }

  // Set results of a estimated tranlation vector
  geometry_msgs::Pose tmp;
  tmp.position.x = tf_.at<double>(0);
  tmp.position.y = 0.0;//tf_.at<double>(1);
  tmp.position.z = tf_.at<double>(2);

  // Set result of a estimated rotation vector
  cv::Mat direction;
  geometry_msgs::Quaternion q;
  calib::euler(Rf_, direction);
  getQuaternionMsg(OFFSET_ROLL, direction.at<double>(1, 0) + OFFSET_PITCH, OFFSET_YAW, q);
  tmp.orientation = q;

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

  // Save visual odometry on csv file
  std::ofstream resultFile;
  resultFile.open(RESULT_FILE, std::ios::app);
  resultFile << tf_.at<double>(0) << "," << tf_.at<double>(1) << "," << tf_.at<double>(2) << std::endl;
  resultFile.close();

  // A redetection is triggered in case the number of feautres being trakced go below a particular threshold
  if (prevFeatures_.size() < MIN_NUM_FEAT)
  {
    featureDetection(prevImage_, prevFeatures_);
    featureTracking(prevImage_, currImage_, prevFeatures_, currFeatures_, status);
  }

  prevImage_ = currImage_.clone();
  prevFeatures_ = currFeatures_;
}
