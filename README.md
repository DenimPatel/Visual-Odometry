# Visual-Odometry

This repository contains the visual odometry pipeline on which I am currently working on.

Currently it works on images sequences of kitti dataset.
1.) Feature detection
FAST features are detected from query Images

2.) Feature tracking
Optical flow based Feature tracking is performed between two consecutive frames for point correspondence.

3.) Pose estimation
Epipolar geometry based Pose estimation which includes computation of Essential matrix and Decomposition to find translation and orientation between two frames. 
![alt text](https://github.com/DenimPatel/Visual-Odometry/blob/master/vo.gif)
