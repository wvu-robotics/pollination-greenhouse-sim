/*  Service to return the closest flower position,
 *  given an estimated position. Input and outputs
 *  are geometry_msgs Pose type.
 */

#include <flower_tf_publisher/nearestflower.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nearest_flower_node");
    ROS_INFO("nearest_flower_node running...");
    flower_tf_publisher::NearestFlower nearest;
    ros::spin();
    return 0;
}