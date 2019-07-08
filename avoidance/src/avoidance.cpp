#include "avoidance/avoidance.h"

namespace avoidance {
Avoidance::Avoidance(const ros::NodeHandle &nh,
                     const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      mav1(false),
      mav2(false),
      mav3(false),
      mission_obs(true),
      time_to_hit_thresh(5.0),
      min_dist(2.0),
      mav_id(2) {
  nh_private_.param("mav_id", mav_id, mav_id);
  nh_private_.param("mission_obs", mission_obs, mission_obs);
  nh_private_.param("time_to_hit_thresh", time_to_hit_thresh,
                    time_to_hit_thresh);
  nh_private_.param("min_dist", min_dist, min_dist);

  mav1_pose_sub = nh_.subscribe("pose_mav1", 1, &Avoidance::mav1Callback, this);
  mav2_pose_sub = nh_.subscribe("pose_mav2", 1, &Avoidance::mav2Callback, this);
  mav3_pose_sub = nh_.subscribe("pose_mav3", 1, &Avoidance::mav3Callback, this);
  traj_des_local_sub = nh_.subscribe("pilot/trajectory/desired", 1,
                                     &Avoidance::trajCallback, this);

  mav_comp_proc_pub = nh_.advertise<mavros_msgs::CompanionProcessStatus>(
      "pilot/companion_process/status", 1, true);
  traj_gen_local_pub = nh_.advertise<mavros_msgs::Trajectory>(
      "pilot/trajectory/generated", 1, true);
}

void Avoidance::mav1Callback(const mav_utils_msgs::GlobalPose &msg) {
  mav1_pose = msg;
  mav1 = true;
  compProcPubCallback();
}
void Avoidance::mav2Callback(const mav_utils_msgs::GlobalPose &msg) {
  mav2_pose = msg;
  mav2 = true;
}
void Avoidance::mav3Callback(const mav_utils_msgs::GlobalPose &msg) {
  mav3_pose = msg;
  mav3 = true;
}

void Avoidance::trajCallback(const mavros_msgs::Trajectory &msg) {
  self_traj = msg;
  trajPubCallback();
}

double Avoidance::tToHit(mav_utils_msgs::GlobalPose mav1,
                         mav_utils_msgs::GlobalPose mav2) {
  int zone;
  bool northp1, northp2;
  double x1, y1, conv1, conv_rad1, scale1;
  double x2, y2, conv2, conv_rad2, scale2;
  GeoLib::UTMUPS::Forward(mav1.latitude, mav1.longitude, zone, northp1, x1, y1,
                          conv2, scale1);
  GeoLib::UTMUPS::Forward(mav2.latitude, mav2.longitude, zone, northp2, x2, y2,
                          conv2, scale2);
  struct Avoidance::MAVRelPose relPose;
  relPose.rel_dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  relPose.rel_head = (atan2(y2 - y1, x2 - x1) >= 0)
                         ? atan2(y2 - y1, x2 - x1)
                         : atan2(y2 - y1, x2 - x1) + 2 * PI;

  tf::Quaternion q1(mav1.orientation.x, mav1.orientation.y, mav1.orientation.z,
                    mav1.orientation.w);

  tf::Matrix3x3 m1(q1);
  double r, p, yaw1;
  m1.getRPY(r, p, yaw1);

  tf::Quaternion q2(mav2.orientation.x, mav2.orientation.y, mav2.orientation.z,
                    mav2.orientation.w);

  tf::Matrix3x3 m2(q2);
  double yaw2;
  m2.getRPY(r, p, yaw2);

  double vel_x =
      ((mav2.linear_twist.x) * cos(yaw2) - (mav2.linear_twist.y) * sin(yaw2)) -
      ((mav1.linear_twist.x) * cos(yaw1) - (mav1.linear_twist.y) * sin(yaw1));
  double vel_y =
      ((mav2.linear_twist.y) * cos(yaw2) + (mav2.linear_twist.x) * sin(yaw2)) -
      ((mav1.linear_twist.y) * cos(yaw1) + (mav1.linear_twist.x) * sin(yaw1));
  relPose.rel_vel_head = (atan2(vel_y, vel_x) >= 0)
                             ? atan2(vel_y, vel_x)
                             : atan2(vel_y, vel_x) + 2 * PI;
  relPose.rel_vel = fabs(sqrt(pow(vel_x, 2) + pow(vel_y, 2)) *
                         cos(relPose.rel_vel_head - relPose.rel_head));
  if (relPose.rel_vel > 0.005 && relPose.rel_dist > min_dist)
    return (relPose.rel_dist / relPose.rel_vel);
  // put this as a prarmeter
  else if (relPose.rel_dist < min_dist)
    return 0.01;
  else
    return (relPose.rel_dist / 0.005);
}

void Avoidance::fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget &point) {
  point.position.x = NAN;
  point.position.y = NAN;
  point.position.z = NAN;
  point.velocity.x = NAN;
  point.velocity.y = NAN;
  point.velocity.z = NAN;
  point.acceleration_or_force.x = NAN;
  point.acceleration_or_force.y = NAN;
  point.acceleration_or_force.z = NAN;
  point.yaw = NAN;
  point.yaw_rate = NAN;
}

void Avoidance::compProcPubCallback() {
  mavros_msgs::CompanionProcessStatus mav_comp_proc;
  mav_comp_proc.component = 196;
  mav_comp_proc.state = 4;
  mav_comp_proc.header.stamp = ros::Time::now();
  mav_comp_proc_pub.publish(mav_comp_proc);
}

void Avoidance::trajPubCallback() {
  mavros_msgs::Trajectory traj_gen_local;
  traj_gen_local.type = 0;
  traj_gen_local.header.frame_id = "local_origin";
  traj_gen_local.header.stamp = ros::Time::now();

  double t_to_hit_mav = tToHit(mav1_pose, mav2_pose);

  if (t_to_hit_mav > time_to_hit_thresh && mav3)
    t_to_hit_mav = tToHit(mav1_pose, mav3_pose);

  traj_gen_local.point_valid = {true, false, false, false, false};
  traj_gen_local.point_1 = self_traj.point_2;

  if (t_to_hit_mav < time_to_hit_thresh && mission_obs) {
    traj_gen_local.point_1.position.z =
        self_traj.point_2.position.z - ((mav_id - 2));
    traj_gen_local.point_1.velocity.z = 2 * (1.0 / t_to_hit_mav) * (2 - mav_id);
  }

  traj_gen_local.point_1.velocity.x = NAN;
  traj_gen_local.point_1.velocity.y = NAN;
  traj_gen_local.point_1.acceleration_or_force.x = NAN;
  traj_gen_local.point_1.acceleration_or_force.y = NAN;
  traj_gen_local.point_1.acceleration_or_force.z = NAN;
  traj_gen_local.point_1.yaw = 1.57;
  traj_gen_local.point_1.yaw_rate = NAN;
  traj_gen_local.time_horizon = {NAN, NAN, NAN, NAN, NAN};
  fillUnusedTrajectoryPoint(traj_gen_local.point_2);
  fillUnusedTrajectoryPoint(traj_gen_local.point_3);
  fillUnusedTrajectoryPoint(traj_gen_local.point_4);
  fillUnusedTrajectoryPoint(traj_gen_local.point_5);

  traj_gen_local_pub.publish(traj_gen_local);
}
}  // namespace avoidance