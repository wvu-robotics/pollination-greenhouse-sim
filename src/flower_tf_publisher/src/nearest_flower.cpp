#include <flower_tf_publisher/nearestflower.hpp>

using geometry_msgs::Pose;
using gazebo_msgs::LinkStates;
using gazebo_msgs::LinkState;

namespace flower_tf_publisher
{
    
    NearestFlower::NearestFlower()
    {
        nearestServ = nh.advertiseService("nearest_flower", &NearestFlower::getNearest, this);
        nh.param<std::string>("flower_link_name", flowerName, "flower_link");
    }
    
    bool NearestFlower::getNearest(flower_tf_publisher::Poses::Request &req,
                                   flower_tf_publisher::Poses::Response &res)
    {
        // Poll Gazebo for link states if none yet found
        if(flowerPoseVec.size() == 0) {
            ROS_INFO("No flower poses stored yet, querying Gazebo...");
            
            // Wait for and receive a message with gazebo model states
            auto linkStatesPtr = ros::topic::waitForMessage<LinkStates>("/gazebo/link_states", nh, ros::Duration(2));
            if(linkStatesPtr == NULL) {
                ROS_ERROR("Timeout on gazebo link topic!");
                return false;
            }
            auto linkStates = *linkStatesPtr;
            ROS_INFO("Got gazebo link information");
            
            // Find links matching name
            for(int i = 0; i < linkStates.name.size(); i++) {
                std::string name = linkStates.name.at(i);

                if(name.find(flowerName) != std::string::npos) {
                    // Grab this pose and add to list
                    Pose currentPose = linkStates.pose.at(i);
                    flowerPoseVec.push_back(currentPose);
                }
            }
            
            if(flowerPoseVec.size() == 0) {
                ROS_ERROR("No flowers found!");
                return false;
            }
            else {
                ROS_INFO("Found %d flowers.", (int)flowerPoseVec.size());
            }
        }
        // End querying Gazebo links
        
        // Storage to find minimum
        double minDistance = std::numeric_limits<double>::max();
        Pose nearestPose;
        
        // Unwrap requested pose
        Pose estimate = req.estimatedPose;
        
        // Iterate through flowers
        for(Pose p : flowerPoseVec) {
            double currentDistance = distance(estimate, p);
            
            // Store new minimum if this is smaller
            if(currentDistance < minDistance) {
                minDistance = currentDistance;
                nearestPose = p;
            }
        }
        
        ROS_INFO("Found nearest flower %f units away.", minDistance);
        
        res.nearestPose = nearestPose;
        return true;
    }
    
    // Square distance, no need to make euclidean since we're just looking for min
    double NearestFlower::distance(geometry_msgs::Pose a, geometry_msgs::Pose b)
    {
        double x = a.position.x - b.position.x;
        double y = a.position.y - b.position.y;
        double z = a.position.z - b.position.z;
        
        return x*x + y*y + z*z;
    }
}