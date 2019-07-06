#include <avoidance/avoidance.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "avoidance_node");
  ros::NodeHandle nh;

  int mav_id;
  double time_to_hit_thresh = 5;
  bool mission_obs = true;

  nh.getParam("avoidance_node/mav_id", mav_id);

  ros::Subscriber mav1_pose_sub = nh.subscribe("pose_mav1", 1, mav1Callback);
  ros::Subscriber mav2_pose_sub = nh.subscribe("pose_mav2", 1, mav2Callback);
  ros::Subscriber mav3_pose_sub = nh.subscribe("pose_mav3", 1, mav3Callback);
  ros::Subscriber traj_des_local_sub =
      nh.subscribe("pilot/trajectory/desired", 1, trajCallback);

  ros::Publisher mav_comp_proc_pub =
      nh.advertise<mavros_msgs::CompanionProcessStatus>(
          "pilot/companion_process/status", 1);
  ros::Publisher traj_gen_local_pub =
      nh.advertise<mavros_msgs::Trajectory>("pilot/trajectory/generated", 1);

  ros::Rate loop_rate(10);

  mavros_msgs::CompanionProcessStatus mav_comp_proc;
  mav_comp_proc.component = 196;
  mav_comp_proc.state = 4;

  mavros_msgs::Trajectory traj_gen_local;
  traj_gen_local.type = 0;
  traj_gen_local.header.frame_id = "local_origin";

  while (ros::ok()) {
    nh.getParam("avoidance_node/t_to_hit_thresh", time_to_hit_thresh);
    nh.getParam("avoidance_node/mis_obs", mission_obs);

    mav_comp_proc.header.stamp = ros::Time::now();
    traj_gen_local.header.stamp = ros::Time::now();

    double t_to_hit_mav = tToHit(mav1_pose, mav2_pose);
    if (t_to_hit_mav > time_to_hit_thresh && mav3)
      t_to_hit_mav = tToHit(mav1_pose, mav3_pose);

    traj_gen_local.point_valid = {true, false, false, false, false};
    traj_gen_local.point_1 = self_traj.point_2;

    if (t_to_hit_mav < time_to_hit_thresh && mission_obs)
      traj_gen_local.point_1.position.z =
          self_traj.point_2.position.z - ((mav_id - 2));

    traj_gen_local.point_1.velocity.x = NAN;
    traj_gen_local.point_1.velocity.y = NAN;
    traj_gen_local.point_1.velocity.z = NAN;
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
    mav_comp_proc_pub.publish(mav_comp_proc);
    loop_rate.sleep();
    mav1 = mav2 = mav3 = false;
    ros::spinOnce();
  }
}
