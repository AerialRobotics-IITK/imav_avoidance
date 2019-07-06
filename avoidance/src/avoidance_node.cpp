#include "avoidance/avoidance.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "avoidance_node");

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  avoidance::Avoidance node(nh, nh_private);
  ROS_INFO("Initialized avoidance node.");

  ros::spin();
  return 0;
}