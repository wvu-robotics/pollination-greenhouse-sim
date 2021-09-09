#include "tf2_ros/buffer.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <manipulation_common/FlowerMap.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

class ResultsEvaluator
{
public:
    // Methods
    void flowerMapCallback(const manipulation_common::FlowerMap::ConstPtr& msg);
    // Members
    tf2_ros::TransformListener tf2_listener;
    tf2_ros::Buffer tf_buffer;

};
