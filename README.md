# mono_vo_ros
This is an OpenCV 3.0 based implementation of a monocular visual odometry algorithm.  
Note that this project is not yet capable of doing reliable relative scale estimation, 
so the scale informaion is extracted from the KITTI dataset ground truth files.  
  
This program is originally from GitHub.  
Please visit the following URL for more information.  
https://github.com/avisingh599/mono-vo

## Demo Video
![Demo_Video](https://bytebucket.org/MasatakaInouchi/mono_vo_ros/raw/1abf3db4d7b9d474ebe7fa41372001661ede4802/mono_vo_ros_demo.gif?token=3bd2ba5496d74bb1896501c9288ef9b96c740758)

## Performance
<div align="center">
<img src="https://bytebucket.org/MasatakaInouchi/mono_vo_ros/raw/8d4b23ddcb1922af0f0ff37138f5faf24d4160d1/mono_vo_ros_performance.png?token=94d3b43f4d91ab3194373d17174aa98f65e1d6a2" width=50%>
</div>

## Before you run
In order to run this algorithm, you need to have either your own data, 
or else the sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).  
In order to run this algorithm on your own data, you must modify the intrinsic calibration parameters in the code.
