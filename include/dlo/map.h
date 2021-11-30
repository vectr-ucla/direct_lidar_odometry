/************************************************************
 *
 * Copyright (c) 2021, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/dlo.h"

class dlo::MapNode {

public:

  MapNode(ros::NodeHandle node_handle);
  ~MapNode();

  static void abort() {
    abort_ = true;
  }

  void start();
  void stop();

private:

  void abortTimerCB(const ros::TimerEvent& e);
  void publishTimerCB(const ros::TimerEvent& e);

  void keyframeCB(const sensor_msgs::PointCloud2ConstPtr& keyframe);

  void getParams();

  ros::NodeHandle nh;
  ros::Timer abort_timer;
  ros::Timer publish_timer;

  ros::Subscriber keyframe_sub;
  ros::Publisher map_pub;

  pcl::PointCloud<PointType>::Ptr dlo_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  ros::Time map_stamp;
  std::string odom_frame;

  double publish_freq_;
  double leaf_size_;

  static std::atomic<bool> abort_;

};
