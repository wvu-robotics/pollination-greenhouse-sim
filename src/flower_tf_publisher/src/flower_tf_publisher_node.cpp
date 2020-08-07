#include <vector>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "gazebo_msgs/LinkState.h"
#include "gazebo_msgs/LinkStates.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using gazebo_msgs::LinkState;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flower_tf_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    
    // Parameter to set string to match
    std::string flowerName;
    nh.param<std::string>("flower_link_name", flowerName, "flower_link");
    
    // Tf static broadcaster will push transforms to tree
    static tf2_ros::StaticTransformBroadcaster tfBroadcaster;
    
    // Wait for and receive a message with gazebo model states
    auto linkStatesPtr = ros::topic::waitForMessage<gazebo_msgs::LinkStates>("/gazebo/link_states", n);
    while(linkStatesPtr == NULL) {
        ROS_ERROR("Timeout on gazebo link topic!");
        auto linkStatesPtr = ros::topic::waitForMessage<gazebo_msgs::LinkStates>("/gazebo/link_states", n);
    }
    auto linkStates = *linkStatesPtr;
    ROS_INFO("Got gazebo link information");
    
    // Structure to hold all the transforms
    std::vector<geometry_msgs::TransformStamped> tfVector;
    
    // Find links matching name
    for(int i = 0; i < linkStates.name.size(); i++) {
        std::string name = linkStates.name.at(i);
        
        if(name.find(flowerName) != std::string::npos) {
            // Get the current pose
            auto bramblePose = linkStates.pose.at(i);
            
            // Create a transform
            geometry_msgs::TransformStamped tfStamped;
            tfStamped.header.stamp = ros::Time::now();
            tfStamped.header.frame_id = "world";
            tfStamped.child_frame_id = name;
            tfStamped.transform.translation.x = bramblePose.position.x;
            tfStamped.transform.translation.y = bramblePose.position.y;
            tfStamped.transform.translation.z = bramblePose.position.z;
            tfStamped.transform.rotation.x = bramblePose.orientation.x;
            tfStamped.transform.rotation.y = bramblePose.orientation.y;
            tfStamped.transform.rotation.z = bramblePose.orientation.z;
            tfStamped.transform.rotation.w = bramblePose.orientation.w;
            
            tfVector.push_back(tfStamped);
        }
    }
    if(tfVector.size() == 0)
        ROS_ERROR("No flowers found!");
    else
        ROS_INFO("Found %d flower links", tfVector.size());
    
    // Publish all the found transforms
    for(auto t : tfVector) {
        tfBroadcaster.sendTransform(t);
    }
    
    ROS_INFO("Spinning until killed");
    ros::spin();
    
    return 0;
}