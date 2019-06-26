#include <ros/ros.h>
#include <GeographicLib/UTMUPS.hpp>
#include <nav_msgs/Odometry.h>
#include <mav_utils_msgs/GlobalPose.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/CompanionProcessStatus.h>

#define PI 3.14159265

namespace geoLib = GeographicLib;
mav_utils_msgs::GlobalPose MAV1Pose, MAV2Pose, MAV3Pose;
mavros_msgs::Trajectory SelfTraj;
bool MAV1 = false, MAV2 = false, MAV3 = false;

struct mavRelPose{
    double relDist;
    double relHead;
    double relVelHead;
    double relVel;
};

void MAV1Callback(const mav_utils_msgs::GlobalPose &msg){
    MAV1Pose = msg;
    MAV1 = true;
}

void MAV2Callback(const mav_utils_msgs::GlobalPose &msg){
    MAV2Pose = msg;
    MAV2 = true;
}

void MAV3Callback(const mav_utils_msgs::GlobalPose &msg){
    MAV3Pose = msg;
    MAV3 = true;
}

void trajCallback(const mavros_msgs::Trajectory &msg){
    SelfTraj = msg;
}

double tToHit(mav_utils_msgs::GlobalPose mav1, mav_utils_msgs::GlobalPose mav2);
void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);

int main(int argc, char** argv){
    ros::init(argc, argv, "avoidance_node");
    ros::NodeHandle nh;

    int MAVId;
    double tToHitThresh=5;
    bool misObs=true;

    nh.getParam("avoidance_node/mav_id",MAVId);
    
    ros::Subscriber mav1PoseSub = nh.subscribe("pose_MAV1", 1, MAV1Callback);
    ros::Subscriber mav2PoseSub = nh.subscribe("pose_MAV2", 1, MAV2Callback);
    ros::Subscriber mav3PoseSub = nh.subscribe("pose_MAV3", 1, MAV3Callback);
    ros::Subscriber trajDesLocalSub = nh.subscribe("pilot/trajectory/desired", 1, trajCallback);

    ros::Publisher mavCompProcPub = nh.advertise<mavros_msgs::CompanionProcessStatus>("pilot/companion_process/status",1);
    ros::Publisher trajGenLocalPub = nh.advertise<mavros_msgs::Trajectory>("pilot/trajectory/generated", 1);
    
    ros::Rate loopRate(10);

    mavros_msgs::CompanionProcessStatus mavCompProc;
    mavCompProc.component = 196;
    mavCompProc.state = 4;

    mavros_msgs::Trajectory trajGenLocal;
    trajGenLocal.type = 0;
    trajGenLocal.header.frame_id = "local_origin";

    while(ros::ok()){

        nh.getParam("avoidance_node/t_to_hit_thresh", tToHitThresh);
        nh.getParam("avoidance_node/mis_obs", misObs);

        mavCompProc.header.stamp = ros::Time::now();
        trajGenLocal.header.stamp = ros::Time::now();

        double tToHitMAV = tToHit(MAV1Pose, MAV2Pose);
        if (tToHitMAV>tToHitThresh && MAV3) tToHitMAV = tToHit(MAV1Pose, MAV3Pose);

        trajGenLocal.point_valid = {true, false, false, false, false};
        trajGenLocal.point_1 = SelfTraj.point_2;

        if (tToHitMAV<tToHitThresh && misObs) trajGenLocal.point_1.position.z = SelfTraj.point_2.position.z-((MAVId-2));
            
        trajGenLocal.point_1.velocity.x = NAN;
        trajGenLocal.point_1.velocity.y = NAN;
        trajGenLocal.point_1.velocity.z = NAN;
        trajGenLocal.point_1.acceleration_or_force.x = NAN;
        trajGenLocal.point_1.acceleration_or_force.y = NAN;
        trajGenLocal.point_1.acceleration_or_force.z = NAN;
        trajGenLocal.point_1.yaw = 1.57;
        trajGenLocal.point_1.yaw_rate = NAN;
        trajGenLocal.time_horizon = {NAN, NAN, NAN, NAN, NAN};
        fillUnusedTrajectoryPoint(trajGenLocal.point_2);
        fillUnusedTrajectoryPoint(trajGenLocal.point_3);
        fillUnusedTrajectoryPoint(trajGenLocal.point_4);
        fillUnusedTrajectoryPoint(trajGenLocal.point_5);

        trajGenLocalPub.publish(trajGenLocal);
        mavCompProcPub.publish(mavCompProc);
        loopRate.sleep();
        MAV1=MAV2=MAV3=false;
        ros::spinOnce();
    }
}

double tToHit(mav_utils_msgs::GlobalPose mav1, mav_utils_msgs::GlobalPose mav2){
    int zone;
    bool northp1, northp2;
    double x1, y1, conv1, convRad1, scale1;
    double x2, y2, conv2, convRad2, scale2;
    geoLib::UTMUPS::Forward(mav1.latitude, mav1.longitude, zone, northp1, x1, y1, conv2, scale1);
    geoLib::UTMUPS::Forward(mav2.latitude, mav2.longitude, zone, northp2, x2, y2, conv2, scale2);
    struct mavRelPose relPose;
    relPose.relDist = sqrt(pow(x2-x1,2)+pow(y2-y1,2));
    relPose.relHead = (atan2(y2-y1,x2-x1) >= 0) ? atan2(y2-y1,x2-x1) : atan2(y2-y1, x2-x1)+2*PI;

    tf::Quaternion q1(
        mav1.orientation.x,
        mav1.orientation.y,
        mav1.orientation.z,
        mav1.orientation.w);

    tf::Matrix3x3 m1(q1);
    double r, p, yaw1;
    m1.getRPY(r, p, yaw1);

    tf::Quaternion q2(
        mav2.orientation.x,
        mav2.orientation.y,
        mav2.orientation.z,
        mav2.orientation.w);

    tf::Matrix3x3 m2(q2);
    double  yaw2;
    m2.getRPY(r, p, yaw2);

    double vel_x = ((mav2.linear_twist.x) * cos(yaw2) - (mav2.linear_twist.y) * sin(yaw2)) - ((mav1.linear_twist.x) * cos(yaw1) - (mav1.linear_twist.y) * sin(yaw1));
    double vel_y = ((mav2.linear_twist.y) * cos(yaw2) + (mav2.linear_twist.x) * sin(yaw2)) - ((mav1.linear_twist.y) * cos(yaw1) + (mav1.linear_twist.x) * sin(yaw1));
    relPose.relVelHead = (atan2(vel_y, vel_x) >= 0) ? atan2(vel_y, vel_x) : atan2(vel_y, vel_x) + 2 * PI;
    relPose.relVel = fabs(sqrt(pow(vel_x, 2) + pow(vel_y, 2))*cos(relPose.relVelHead-relPose.relHead));
    if(relPose.relVel>0.005 && relPose.relDist>2.0) return (relPose.relDist/relPose.relVel);
    //put this as a prarmeter
    else if (relPose.relDist<2.0) return 0;
    else return (relPose.relDist/0.005);
}


void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point) {
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