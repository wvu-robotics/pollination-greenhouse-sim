/* 
 * File:   nearestflower.hpp
 * Author: nhewitt
 *
 * Created on August 13, 2020, 11:14 AM
 */

#ifndef NEARESTFLOWER_HPP
#define NEARESTFLOWER_HPP

#include <vector>
#include <string>
#include <limits>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "flower_tf_publisher/Poses.h"

namespace flower_tf_publisher
{
class NearestFlower {
public:
    NearestFlower();
    
    bool getNearest(flower_tf_publisher::Poses::Request &req,
                    flower_tf_publisher::Poses::Response &res);
    
    ros::NodeHandle nh;
    ros::ServiceServer nearestServ;
    std::string flowerName;
    std::vector<geometry_msgs::Pose> flowerPoseVec;
private:
    double distance(geometry_msgs::Pose a, geometry_msgs::Pose b);
};
}

#endif /* NEARESTFLOWER_HPP */

