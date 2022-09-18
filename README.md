# mono_vo_ros
This is an OpenCV 3.0 based implementation of a monocular visual odometry algorithm.  
Note that this project is not yet capable of doing reliable relative scale estimation, 
so the scale informaion is extracted from the KITTI dataset ground truth files.  
  
This program is originally from GitHub.  
Please visit the following URL for more information.  
https://github.com/avisingh599/mono-vo

## Demo Video
![Demo_Video](https://github.com/inouchi/mono_vo_ros/blob/media/mono_vo_ros_demo.gif)

## Performance
![Performance](https://github.com/inouchi/mono_vo_ros/blob/media/mono_vo_ros_performance.png)

## Before you run
In order to run this algorithm, you need to have either your own data, 
or else the sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).  
In order to run this algorithm on your own data, you must modify the intrinsic calibration parameters in the code.
