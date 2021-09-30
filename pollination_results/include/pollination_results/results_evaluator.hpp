#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <manipulation_common/FlowerMap.h>
#include <manipulation_common/StateMachine.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/Bool.h>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <map>

class ResultsEvaluator
{
public:
    // Methods
    ResultsEvaluator();
    void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void flowerMapCallback(const manipulation_common::FlowerMap::ConstPtr& msg);
    void manipStateMachineCallback(const manipulation_common::StateMachine::ConstPtr& msg);
    void missionCompleteCallback(const std_msgs::Bool::ConstPtr& msg);
    void updateFlowersDetected();
    void updateFlowersApproached();
    void updateFlowersPollinated();
    void saveResultsFiles();
    void setWorldToBaseLinkTF();
    void setWorldtoEndEffectorLinkTF();

    // Members
    tf2_ros::TransformListener tf2_listener;
    tf2_ros::Buffer tf_tree_buffer;
    tf2_ros::Buffer gazebo_truth_buffer;
    unsigned int num_flowers;
    std::vector<geometry_msgs::Pose> flower_true_poses;
    std::vector<std::string> flower_names;
    std::string flower_stats_results_file_name;
    std::string flower_mapping_error_results_file_name;
    std::string link_states_topic_name;
    std::string flower_map_topic_name;
    std::string manip_state_machine_topic_name;
    std::string mission_complete_topic_name;
    std::ofstream flower_stats_results_file;
    std::ofstream flower_mapping_error_results_file; 
    ros::Subscriber link_states_sub;
    ros::Subscriber flower_map_sub;
    ros::Subscriber manip_state_machine_sub;
    ros::Subscriber mission_complete_sub;
    gazebo_msgs::LinkStates link_states;
    manipulation_common::FlowerMap detected_flowers;
    std::vector<unsigned int> detected_flower_true_ids;
    std::vector<unsigned int> approached_flower_true_ids;
    std::vector<unsigned int> pollinated_flower_true_ids;
    std::map<int, unsigned int> detected_flower_id_to_true_flower_id_map;
    std::vector<geometry_msgs::PointStamped> detected_flower_estimated_positions;
    manipulation_common::StateMachine manip_state_machine_msg;
    double mission_start_time; // sec
    double approached_distance_threshold; // m
    double end_effector_tip_diameter; // m
    double end_effector_extension_length; // m
    double flower_pollinated_zenith_angle_range; // rad
};
