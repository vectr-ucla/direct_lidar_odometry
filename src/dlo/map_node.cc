/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/map.h"

void controlC(int sig) {

  dlo::MapNode::abort();

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "dlo_map_node");
  ros::NodeHandle nh("~");

  signal(SIGTERM, controlC);
  sleep(0.5);

  dlo::MapNode node(nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  node.start();
  ros::waitForShutdown();

  return 0;

}
