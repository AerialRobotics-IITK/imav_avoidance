#ifndef AVOIDANCE_AVOIDANCE_H_
#define AVOIDANCE_AVOIDANCE_H_

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_utils_msgs/UTMPose.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Trajectory.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.14159265

namespace avoidance {
class Avoidance {
 public:
  Avoidance(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // input data using ros callabacks
  void mav1Callback(const mav_utils_msgs::UTMPose& msg);
  void mav2Callback(const mav_utils_msgs::UTMPose& msg);
  void mav3Callback(const mav_utils_msgs::UTMPose& msg);
  void trajCallback(const mavros_msgs::Trajectory& msg);
  void mpcCallback(const mav_msgs::RollPitchYawrateThrust& msg);
  void missionSetpointCallback(const geometry_msgs::PointStamped& msg);
  void mavStateCallback(const mavros_msgs::State& msg);

  /*
  all avoidance related things are here
  ---- traj pub for mission mode
  ---- offboard setpoint modification for avoidance
   */
  void trajPubCallback();

  // contains flag for enable avoidance
  void compProcPubCallback();

  // offboard control setpoint
  void offboardControlPubCallback();

  // returns time to other mav
  double tToHit(mav_utils_msgs::UTMPose mav1, mav_utils_msgs::UTMPose mav2);

  bool collision();

  // to fill not used trajectory points
  void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);

  struct MAVRelPose {
    double rel_dist;
    double rel_head;
    double rel_vel_head;
    double rel_vel;
  };

  // state -- all mavs pose
  mav_utils_msgs::UTMPose mav1_pose, mav2_pose, mav3_pose;

  // state -- mav px4 state
  mavros_msgs::State mav_state;

  // setpoints -- from mission mode
  mavros_msgs::Trajectory self_traj;

  // setpoints -- from model predictive controller
  mav_msgs::RollPitchYawrateThrust mpc_rpyth;

  // modified setpoints -- in mission mode after avoidance modification
  geometry_msgs::PointStamped mission_setpoint;

  // check -- checking for mav3 if its available!?
  bool mav3_check;

  // check param file
  bool mission_obs, mpc_enable;
  double time_to_hit_thresh, min_dist, flight_alt, delta_z;
  int mav_id;

  ros::NodeHandle nh_, nh_private_;

  // subscriber and publisher
  ros::Subscriber mav1_pose_sub, mav2_pose_sub, mav3_pose_sub,
      traj_des_local_sub, mpc_rpyth_sub, mission_setpoint_sub, mav_state_sub;
  ros::Publisher mav_comp_proc_pub, traj_gen_local_pub, command_pose_pub,
      mpc_rpyth_pub;
};
}  // namespace avoidance

#endif