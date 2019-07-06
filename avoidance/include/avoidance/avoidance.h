#ifndef AVOIDANCE_AVOIDANCE_H_
#define AVOIDANCE_AVOIDANCE_H_

#include <mav_utils_msgs/GlobalPose.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/Trajectory.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <GeographicLib/UTMUPS.hpp>

#define PI 3.14159265

namespace geoLib = GeographicLib;

mav_utils_msgs::GlobalPose mav1_pose, mav2_pose, mav3_pose;
mavros_msgs::Trajectory self_traj;

bool mav1 = false, mav2 = false, mav3 = false;

struct MAVRelPose {
  double rel_dist;
  double rel_head;
  double rel_vel_head;
  double rel_vel;
};

void mav1Callback(const mav_utils_msgs::GlobalPose &msg) {
  mav1_pose = msg;
  mav1 = true;
}

void mav2Callback(const mav_utils_msgs::GlobalPose &msg) {
  mav2_pose = msg;
  mav2 = true;
}

void mav3Callback(const mav_utils_msgs::GlobalPose &msg) {
  mav3_pose = msg;
  mav3 = true;
}

void trajCallback(const mavros_msgs::Trajectory &msg) { self_traj = msg; }

double tToHit(mav_utils_msgs::GlobalPose mav1,
              mav_utils_msgs::GlobalPose mav2) {
  int zone;
  bool northp1, northp2;
  double x1, y1, conv1, conv_rad1, scale1;
  double x2, y2, conv2, conv_rad2, scale2;
  geoLib::UTMUPS::Forward(mav1.latitude, mav1.longitude, zone, northp1, x1, y1,
                          conv2, scale1);
  geoLib::UTMUPS::Forward(mav2.latitude, mav2.longitude, zone, northp2, x2, y2,
                          conv2, scale2);
  struct MAVRelPose relPose;
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
  if (relPose.rel_vel > 0.005 && relPose.rel_dist > 2.0)
    return (relPose.rel_dist / relPose.rel_vel);
  // put this as a prarmeter
  else if (relPose.rel_dist < 2.0)
    return 0;
  else
    return (relPose.rel_dist / 0.005);
}

void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget &point) {
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

#endif