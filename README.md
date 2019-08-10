# imav_avoidance

## Package Summary

Contains an MAV avoidance module for MultiMAV system.

Simply based on the documentation provided here: [PX4_docs on Avoidance](https://docs.px4.io/v1.9.0/en/computer_vision/obstacle_avoidance.html)

Maintainer : Gajendra Nagar (gajendranagar02@gmail.com)

## ROS Node API

### Subscribed Topics
* pose_mav1([mav_utils_msgs/UTMPose](https://github.com/gajena/mav_utils_ariitk/blob/master/mav_utils_msgs/msg/UTMPose.msg))

    position of mav on which the node is running.

* pose_mav2([mav_utils_msgs/UTMPose](https://github.com/gajena/mav_utils_ariitk/blob/master/mav_utils_msgs/msg/UTMPose.msg))

    position of the other mav.

* pose_mav3([mav_utils_msgs/UTMPose](https://github.com/gajena/mav_utils_ariitk/blob/master/mav_utils_msgs/msg/UTMPose.msg))

    position of the other mav.

* pilot/state([mavros_msgs/State](http://docs.ros.org/jade/api/mavros_msgs/html/msg/State.html))
    
    Current autopilot state

* pilot/trajectory/desired([mavros_msgs/Trajectory](http://docs.ros.org/melodic/api/mavros_msgs/html/msg/Trajectory.html))

    Desired trajectory calculated by autopilot

* command/roll_pitch_yawrate_thrust([mav_msgs/RollPitchYawrateThrust](http://docs.ros.org/indigo/api/mav_msgs/html/msg/RollPitchYawrateThrust.html))

    Commands provided by high-level controller

* mission_setpoint([geometry_msgs/PointStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PointStamped.html))

    Position setpoint in offboard mode

### Published Topics

* pilot/companion_process/status([mavros_msgs/CompanionProcessStatus](http://docs.ros.org/api/mavros_msgs/html/msg/CompanionProcessStatus.html)

* pilot/trajectory/generated([mavros_msgs/Trajectory](http://docs.ros.org/melodic/api/mavros_msgs/html/msg/Trajectory.html))

* command/pose([geometry_msgs/PoseStamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html))

* pilot/setpoint_raw/roll_pitch_yawrate_thrust([mav_msgs/RollPitchYawrateThrust](http://docs.ros.org/indigo/api/mav_msgs/html/msg/RollPitchYawrateThrust.html))


### Parameters

* mpc_enable(bool, default:=true) 
    
    enable mpc controller setpoints

* flight_alt(double, defalut:=5.0)
    
    [meter] altitude where vehicle will perform search

* delta_z(double, default:=1.0)

    [meter] avoidance envelope = flight_alt +/- delta_z
* mission_obs(bool, default:=true)
    
    enable obstacle avoidance in mission_mode

* time_to_hit_thresh(double, default:=5.0)

    [sec] threshold for time to hit any other mav, by increasing this: mav will show response from far distance with less velocity

* min_dist(double, default:=5.0)
    
    [meter] minimum distance for avoidance if time to hit is greater than time_to_hit_thresh

* mav_id(int, default:=2)
    
    [id] based on id mav decide its trajectory in z-direction
    
    z_sp = z-(mav_id-2)

