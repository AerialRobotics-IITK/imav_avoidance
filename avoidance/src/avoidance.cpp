#include "avoidance/avoidance.h"

namespace avoidance {
Avoidance::Avoidance(const ros::NodeHandle &nh,
                     const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      mav3_check(false),
      mission_obs(true),
      mpc_enable(false),
      time_to_hit_thresh(5.0),
      min_dist(2.0),
      flight_alt(3.0),
      delta_z(1),
      mav_id(2){
  // ros parameters
  nh_private_.param("mission_obs", mission_obs, mission_obs);
  nh_private_.param("time_to_hit_thresh", time_to_hit_thresh,
                    time_to_hit_thresh);
  nh_private_.param("min_dist", min_dist, min_dist);
  nh_private_.param("flight_alt", flight_alt, flight_alt);
  nh_private_.param("delta_z", delta_z, delta_z);
  nh_private_.param("mpc_enable", mpc_enable, mpc_enable);
  nh_private_.param("mav_id", mav_id, mav_id);

  // ros subscribers
  mav1_pose_sub = nh_.subscribe("pose_mav1", 1, &Avoidance::mav1Callback, this);
  mav2_pose_sub = nh_.subscribe("pose_mav2", 1, &Avoidance::mav2Callback, this);
  mav3_pose_sub = nh_.subscribe("pose_mav3", 1, &Avoidance::mav3Callback, this);
  mav_state_sub =
      nh_.subscribe("pilot/state", 1, &Avoidance::mavStateCallback, this);
  traj_des_local_sub = nh_.subscribe("pilot/trajectory/desired", 1,
                                     &Avoidance::trajCallback, this);
  mpc_rpyth_sub = nh_.subscribe("command/roll_pitch_yawrate_thrust", 1,
                                &Avoidance::mpcCallback, this);
  mission_setpoint_sub = nh_.subscribe(
      "mission_setpoint", 1, &Avoidance::missionSetpointCallback, this);

  // ros publishers
  mav_comp_proc_pub = nh_.advertise<mavros_msgs::CompanionProcessStatus>(
      "pilot/companion_process/status", 1, true);
  traj_gen_local_pub = nh_.advertise<mavros_msgs::Trajectory>(
      "pilot/trajectory/generated", 1, true);
  command_pose_pub =
      nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1, true);
  mpc_rpyth_pub = nh_.advertise<mav_msgs::RollPitchYawrateThrust>(
      "pilot/setpoint_raw/roll_pitch_yawrate_thrust", 1, true);
}

void Avoidance::mav1Callback(const mav_utils_msgs::UTMPose &msg) {
  mav1_pose = msg;
  compProcPubCallback();
  offboardControlPubCallback();
}

void Avoidance::mav2Callback(const mav_utils_msgs::UTMPose &msg) {
  mav2_pose = msg;
}

void Avoidance::mav3Callback(const mav_utils_msgs::UTMPose &msg) {
  mav3_pose = msg;
  mav3_check = true;
}

void Avoidance::trajCallback(const mavros_msgs::Trajectory &msg) {
  self_traj = msg;
  trajPubCallback();
}

void Avoidance::mpcCallback(const mav_msgs::RollPitchYawrateThrust &msg) {
  mpc_rpyth = msg;
}

void Avoidance::missionSetpointCallback(
    const geometry_msgs::PointStamped &msg) {
  mission_setpoint = msg;
}

void Avoidance::mavStateCallback(const mavros_msgs::State &msg) {
  mav_state = msg;
}

double Avoidance::tToHit(mav_utils_msgs::UTMPose mav1,
                         mav_utils_msgs::UTMPose mav2) {
  struct Avoidance::MAVRelPose relPose;

  // distance between two mavs
  relPose.rel_dist = sqrt(pow(mav2.pose.position.x - mav1.pose.position.x, 2) +
                          pow(mav2.pose.position.y - mav1.pose.position.y, 2));
  // relative heading
  relPose.rel_head = (atan2(mav2.pose.position.y - mav1.pose.position.y,
                            mav2.pose.position.x - mav1.pose.position.x) >= 0)
                         ? atan2(mav2.pose.position.y - mav1.pose.position.y,
                                 mav2.pose.position.x - mav1.pose.position.x)
                         : atan2(mav2.pose.position.y - mav1.pose.position.y,
                                 mav2.pose.position.x - mav1.pose.position.x) +
                               2 * PI;

  tf::Quaternion q1(mav1.pose.orientation.x, mav1.pose.orientation.y,
                    mav1.pose.orientation.z, mav1.pose.orientation.w);

  tf::Matrix3x3 m1(q1);
  double r, p, yaw1;
  m1.getRPY(r, p, yaw1);

  tf::Quaternion q2(mav2.pose.orientation.x, mav2.pose.orientation.y,
                    mav2.pose.orientation.z, mav2.pose.orientation.w);

  tf::Matrix3x3 m2(q2);
  double yaw2;
  m2.getRPY(r, p, yaw2);

  double vel_x =
      ((mav2.linear_twist.x) * cos(yaw2) - (mav2.linear_twist.y) * sin(yaw2)) -
      ((mav1.linear_twist.x) * cos(yaw1) - (mav1.linear_twist.y) * sin(yaw1));
  double vel_y =
      ((mav2.linear_twist.y) * cos(yaw2) + (mav2.linear_twist.x) * sin(yaw2)) -
      ((mav1.linear_twist.y) * cos(yaw1) + (mav1.linear_twist.x) * sin(yaw1));

  // relative headig between velocities
  relPose.rel_vel_head = (atan2(vel_y, vel_x) >= 0)
                             ? atan2(vel_y, vel_x)
                             : atan2(vel_y, vel_x) + 2 * PI;

  // relative velocity
  relPose.rel_vel = fabs(sqrt(pow(vel_x, 2) + pow(vel_y, 2)) *
                         cos(relPose.rel_vel_head - relPose.rel_head));

  // handling almost zero velocity before publishing time to hit
  if (relPose.rel_vel > 0.005 && relPose.rel_dist > min_dist)
    return (relPose.rel_dist / relPose.rel_vel);
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

bool Avoidance::collision() {
  // time to hit first mav
  double t_to_hit_mav = tToHit(mav1_pose, mav2_pose);

  // if not hitting first then check for other
  if (t_to_hit_mav > time_to_hit_thresh && mav3_check) {
    t_to_hit_mav = tToHit(mav1_pose, mav3_pose);
  }

  if (t_to_hit_mav < time_to_hit_thresh && mission_obs)
    return true;
  else
    return false;
}

void Avoidance::trajPubCallback() {
  mavros_msgs::Trajectory traj_gen_local;
  traj_gen_local.type = 0;
  traj_gen_local.header.frame_id = "local_origin";
  traj_gen_local.header.stamp = ros::Time::now();
  // only publishing first point, filling others with NAN
  traj_gen_local.point_valid = {true, false, false, false, false};
  fillUnusedTrajectoryPoint(traj_gen_local.point_2);
  fillUnusedTrajectoryPoint(traj_gen_local.point_3);
  fillUnusedTrajectoryPoint(traj_gen_local.point_4);
  fillUnusedTrajectoryPoint(traj_gen_local.point_5);

  // keeping all points same as mission setpoint and changing yaw to fix
  traj_gen_local.point_1 = self_traj.point_2;
  traj_gen_local.point_1.yaw = 1.57;

  // if time to hit less than threshold then modify setpoints
  if (mav_state.mode == "AUTO.MISSION" &&
      (((flight_alt - delta_z) < self_traj.point_2.position.z) &&
       ((flight_alt + delta_z) > self_traj.point_2.position.z)) &&
      collision() && mission_obs) {
    traj_gen_local.point_1.position.z =
        self_traj.point_2.position.z - ((mav_id - 2));
    traj_gen_local.point_1.velocity.z = (5 * (2 - mav_id));
  }
  traj_gen_local_pub.publish(traj_gen_local);
}

void Avoidance::offboardControlPubCallback() {
  geometry_msgs::PoseStamped pose_setpoint;

  pose_setpoint.pose.position = mission_setpoint.point;

  if (mav_state.mode == "OFFBOARD" &&
      (((flight_alt - delta_z) < mission_setpoint.point.z) &&
       ((flight_alt + delta_z) > mission_setpoint.point.z)) &&
      collision() && mission_obs) {
    pose_setpoint.pose.position.z = mission_setpoint.point.z - (mav_id - 2);
  }

  tf2::Quaternion setpoint_quat;
  setpoint_quat.setRPY(0, 0, 1.57);

  pose_setpoint.pose.orientation = tf2::toMsg(setpoint_quat);
  command_pose_pub.publish(pose_setpoint);

  if (mpc_enable) mpc_rpyth_pub.publish(mpc_rpyth);
}

}  // namespace avoidance