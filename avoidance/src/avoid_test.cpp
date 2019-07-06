#include "avoidance/avoid.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "avoid_test");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  avoidance::Avoidance node(nh, nh_private);
  ROS_INFO("Initialized avoidance node.");

  ros::spin();
  return 0;
}