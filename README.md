# Nano-GICP: Fast-GICP + Nano-FLANN
+ This branch is for being used as a module in other packages
+ Nano-GICP is from [here, official repo of DLO or DLIO](https://github.com/vectr-ucla/direct_lidar_odometry)

### Dependencies
- PCL >= 1.8
- C++ >= 14
- OpenMP >= 4.5
- CMake >= 3.10.0
- Eigen >= 3.3.7

### Use case
+ Refer - [here](https://github.com/engcang/FAST-LIO-SAM-QN)
+ Or, refer to the example usage as follows:
  1. `catkin build` this repository in your workspace,
  2. In the `CMakeLists.txt` of your wanted package, import `nano_gicp` as a component of `catkin`
    ```CMake
    find_package(catkin REQUIRED COMPONENTS
      ...
      nano_gicp #Include here
      ...
    )
    include_directories(
      ...
      ${catkin_INCLUDE_DIRS} #Header files are included in catkin_INCLUDE_DIRS
      ...
    )
    add_library(some_library src/some_src.cpp)
    target_link_libraries(some_library ${catkin_LIBRARIES}) #Libraries are included in catkin_LIBRARIES
    ```
  3. Use in the source file of your wanted package as:
    ```cpp
    #include <nano_gicp/point_type_nano_gicp.hpp> //change PointType in this headerfile, currently pcl::PointXYZI
    #include <nano_gicp/nano_gicp.hpp>
    
    nano_gicp::NanoGICP<PointType, PointType> m_nano_gicp;
    
    ////// nano_gicp init
    m_nano_gicp.setMaxCorrespondenceDistance(max_corres_dist_);
    m_nano_gicp.setNumThreads(thread_number_);
    m_nano_gicp.setCorrespondenceRandomness(correspondences_number_);
    m_nano_gicp.setMaximumIterations(max_iter_);
    m_nano_gicp.setTransformationEpsilon(transformation_epsilon_);
    m_nano_gicp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
    m_nano_gicp.setRANSACIterations(ransac_max_iter_);
    m_nano_gicp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold_);
    
    ////// use
    pcl::PointCloud<pcl::PointXYZI>::Ptr src_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr dst_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointType> dummy_;
    /* watch out! */
    *src_ = src_data; //put your data here
    *dst_ = dst_data; //put your data here
    /* watch out! */
    m_nano_gicp.setInputSource(src_);
    m_nano_gicp.calculateSourceCovariances();
    m_nano_gicp.setInputTarget(dst_);
    m_nano_gicp.calculateTargetCovariances();
    m_nano_gicp.align(dummy_);
    
    double score_ = m_nano_gicp.getFitnessScore();
    // if matchness score is lower than threshold, (lower is better)
    if(m_nano_gicp.hasConverged() && score_ < icp_score_threshold)
    {
      Eigen::Matrix4f pose_betweenf_ = m_nano_gicp.getFinalTransformation(); //float
      Eigen::Matrix4d pose_betweend_ = m_nano_gicp.getFinalTransformation().cast<double>(); //double
    }
    ```

## License and acknowledgements
This work is licensed under the terms of the MIT license.

- [FastGICP](https://github.com/SMRT-AIST/fast_gicp) - Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno, “Voxelized GICP for Fast and Accurate 3D Point Cloud Registration,” in _IEEE International Conference on Robotics and Automation (ICRA)_, IEEE, 2021, pp. 11 054–11 059.
- [NanoFLANN](https://github.com/jlblancoc/nanoflann) - Jose Luis Blanco and Pranjal Kumar Rai, “NanoFLANN: a C++ Header-Only Fork of FLANN, A Library for Nearest Neighbor (NN) with KD-Trees,” https://github.com/jlblancoc/nanoflann, 2014.
- [DLO](https://github.com/vectr-ucla/direct_lidar_odometry) - Kenny Chen, Brett T. Lopez, Ali-akbar Agha-mohammadi, and Ankur Mehta, “Direct LiDAR Odometry: Fast Localization With Dense Point Clouds,” in _IEEE Robotics and Automation Letters_, 2022.
