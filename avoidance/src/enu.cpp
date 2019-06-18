#include <ros/ros.h>
#include <GeographicLib/UTMUPS.hpp>
#include <nav_msgs/Odometry.h>
#include <avoidance_msgs/RelPose.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/CompanionProcessStatus.h>

#define PI 3.14159265

namespace geoLib = GeographicLib;
avoidance_msgs::RelPose MAV1Pose, MAV2Pose, MAV3Pose;
mavros_msgs::Trajectory SelfTraj;
bool MAV1 = false, MAV2 = false, MAV3 = false;

struct mavRelPose{
    double relDist;
    double relHead;
    double relVelHead;
    double relVel;
};

void MAV1Callback(const avoidance_msgs::RelPose &msg){
    MAV1Pose = msg;
    MAV1 = true;
}

void MAV2Callback(const avoidance_msgs::RelPose &msg){
    MAV2Pose = msg;
    MAV2 = true;
}

void MAV3Callback(const avoidance_msgs::RelPose &msg){
    MAV3Pose = msg;
    MAV3 = true;
}

void trajCallback(const mavros_msgs::Trajectory &msg){
    SelfTraj = msg;
}

double tToHit(avoidance_msgs::RelPose mav1, avoidance_msgs::RelPose mav2);
void fillUnusedTrajectoryPoint(mavros_msgs::PositionTarget& point);

int main(int argc, char** argv){
    ros::init(argc, argv, "enu");
    ros::NodeHandle nh;
    std::string MAVId;
    double tToHitThresh=5;
   
    nh.getParam("MAVId", MAVId);
    
    ros::Subscriber mav1PoseSub = nh.subscribe("pose_MAV1", 1, MAV1Callback);
    ros::Subscriber mav2PoseSub = nh.subscribe("pose_MAV2", 1, MAV2Callback);
    ros::Subscriber mav3PoseSub = nh.subscribe("pose_MAV3", 1, MAV3Callback);
    ros::Subscriber trajDesLocalSub = nh.subscribe("pilot/trajectory/desired", 1, trajCallback);

    ros::Publisher mavCompProcPub = nh.advertise<mavros_msgs::CompanionProcessStatus>("pilot/companion_process/status",1);
    ros::Publisher trajGenLocalPub = nh.advertise<mavros_msgs::Trajectory>("pilot/trajectory/generated", 1);
    ros::Rate loopRate(10);

    mavros_msgs::CompanionProcessStatus mavCompProc;
    mavCompProc.header.frame_id = "";
    mavCompProc.component = 196;

    mavros_msgs::Trajectory trajGenLocal;
    trajGenLocal.header.stamp = ros::Time::now();
    trajGenLocal.type = 0;
    trajGenLocal.header.frame_id = "local_origin";

    while(ros::ok()){
        double tToHitMAV = tToHit(MAV1Pose, MAV2Pose);
        if (tToHitMAV>tToHitThresh && MAV3) double tToHitMAV = tToHit(MAV1Pose, MAV3Pose);

        if (tToHitMAV<tToHitThresh){
            trajGenLocal.point_valid = {true, false, false, false, false};
            trajGenLocal.point_1 = SelfTraj.point_2;
            trajGenLocal.point_1.position.z = SelfTraj.point_2.position.z-1;
            trajGenLocal.point_1.velocity.x = NAN;
            trajGenLocal.point_1.velocity.y = NAN;
            trajGenLocal.point_1.velocity.z = NAN;
            trajGenLocal.point_1.acceleration_or_force.x = NAN;
            trajGenLocal.point_1.acceleration_or_force.y = NAN;
            trajGenLocal.point_1.acceleration_or_force.z = NAN;
            trajGenLocal.point_1.yaw = (SelfTraj.point_2.yaw);
            trajGenLocal.point_1.yaw_rate = NAN;
            trajGenLocal.time_horizon = {NAN, NAN, NAN, NAN, NAN};
            fillUnusedTrajectoryPoint(trajGenLocal.point_2);
            fillUnusedTrajectoryPoint(trajGenLocal.point_3);
            fillUnusedTrajectoryPoint(trajGenLocal.point_4);
            fillUnusedTrajectoryPoint(trajGenLocal.point_5);
        }
        else trajGenLocal.point_valid = {false, false, false, false, false};
        trajGenLocalPub.publish(trajGenLocal);
        mavCompProcPub.publish(mavCompProc);
        loopRate.sleep();
        MAV1=MAV2=MAV3=false;
        ros::spinOnce();
    }
}

double tToHit(avoidance_msgs::RelPose mav1, avoidance_msgs::RelPose mav2){
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

    double vel_x = ((mav2.twist.linear.x) * cos(yaw2) - (mav2.twist.linear.y) * sin(yaw2)) - ((mav1.twist.linear.x) * cos(yaw1) - (mav1.twist.linear.y) * sin(yaw1));
    double vel_y = ((mav2.twist.linear.y) * cos(yaw2) + (mav2.twist.linear.x) * sin(yaw2)) - ((mav1.twist.linear.y) * cos(yaw1) + (mav1.twist.linear.x) * sin(yaw1));
    relPose.relVelHead = (atan2(vel_y, vel_x) >= 0) ? atan2(vel_y, vel_x) : atan2(vel_y, vel_x) + 2 * PI;
    relPose.relVel = fabs(sqrt(pow(vel_x, 2) + pow(vel_y, 2))*cos(relPose.relVelHead-relPose.relHead));
    if(relPose.relVel>0.005) return (relPose.relDist/relPose.relVel);
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