# Hough_circle
Circle detection with ROS and C++ code. Also adding some TF publisher

# Introduction
This package is used to detect a circle which uses Hough Circle Detection Function from OpenCV. More details about the hough circle 
can be read in https://docs.opencv.org/2.4.13.7/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html and the ROS version
http://wiki.ros.org/opencv_apps.

This package is specifically used to determine camera angle from detecting a circle.

# How to launch
```bash
 roslaunch hough_circle circle_detection.launch
```
You also need to run the service for the hough circle detection to work
```bash
rosservice call /camera_calibration "data: true"
```

You can also determine parameter such as (threshold,circle to circle distance in dynamic reconfigure)
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
You need to checklist the image_stream so it will stream the image. And if the debugging is checked, then you the angle will be shown at real time, but if it's unchecked, it will take approximately 10 data and take the median as the final angle result

![alt text](https://github.com/Jphartogi/Hough_circle/blob/master/RQT_reconfigure%20for%20hough%20circle.png)

# Author 

Joshua Phartogi https://github.com/Jphartogi

