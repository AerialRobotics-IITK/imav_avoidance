#ifndef AVOIDANCE_AVOID_H_
#define AVOIDANCE_AVOID_H_

#include <mav_utils_msgs/GlobalPose.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/Trajectory.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <GeographicLib/UTMUPS.hpp>

#define PI 3.14159265

namespace GeoLib = GeographicLib;

namespace avoidance {
class Avoidance {
 public:
  Avoidance(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void mav1Callback(const mav_utils_msgs::GlobalPose& msg);
  void mav2Callback(const mav_utils_msgs::GlobalPose& msg);
  void mav3Callback(const mav_utils_msgs::GlobalPose& msg);
  void trajCallback(const mavros_msgs::Trajectory& msg);
  void trajPubCallback();
  void compProcPubCallback();

  double tToHit(mav_utils_msgs::GlobalPose mav1,
                mav_utils_msgs::GlobalPose mav2);

  void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);

  struct MAVRelPose {
    double rel_dist;
    double rel_head;
    double rel_vel_head;
    double rel_vel;
  };

  mav_utils_msgs::GlobalPose mav1_pose, mav2_pose, mav3_pose;
  mavros_msgs::Trajectory self_traj;

  bool mav1, mav2, mav3, mission_obs;
  double time_to_hit_thresh;
  int mav_id;

  ros::NodeHandle nh_, nh_private_;

  ros::Subscriber mav1_pose_sub, mav2_pose_sub, mav3_pose_sub,
      traj_des_local_sub;
  ros::Publisher mav_comp_proc_pub, traj_gen_local_pub;
};
}  // namespace avoidance

#endif