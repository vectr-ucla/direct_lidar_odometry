# Nano-GICP: Fast-GICP + Nano-FLANN
+ This branch is for being used as a module in other packages
+ Nano-GICP is from [here, official repo of DLO or DLIO](https://github.com/vectr-ucla/direct_lidar_odometry)

### Dependencies
- Ubuntu 18.04 or 20.04
- ROS Melodic or Noetic (`roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `pcl_ros`)
- C++ >= 14
- OpenMP >= 4.5
- CMake >= 3.16.3
- Eigen >= 3.3.7

### Use case
+ Refer - [here](https://github.com/engcang/FAST-LIO-SAM-QN)

## License and acknowledgements
This work is licensed under the terms of the MIT license.

- [FastGICP](https://github.com/SMRT-AIST/fast_gicp) - Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno, “Voxelized GICP for Fast and Accurate 3D Point Cloud Registration,” in _IEEE International Conference on Robotics and Automation (ICRA)_, IEEE, 2021, pp. 11 054–11 059.
- [NanoFLANN](https://github.com/jlblancoc/nanoflann) - Jose Luis Blanco and Pranjal Kumar Rai, “NanoFLANN: a C++ Header-Only Fork of FLANN, A Library for Nearest Neighbor (NN) with KD-Trees,” https://github.com/jlblancoc/nanoflann, 2014.
- [DLO](https://github.com/vectr-ucla/direct_lidar_odometry) - Kenny Chen, Brett T. Lopez, Ali-akbar Agha-mohammadi, and Ankur Mehta, “Direct LiDAR Odometry: Fast Localization With Dense Point Clouds,” in _IEEE Robotics and Automation Letters_, 2022.