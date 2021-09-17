#include <pollination_results/results_evaluator.hpp>

ResultsEvaluator::ResultsEvaluator() :
    tf2_listener(tf_tree_buffer)
{
    ros::NodeHandle nh;
    // Get launch params
    if(ros::param::get(ros::this_node::getName()+"/results_file_name", this->results_file_name) == false)
    {
        ROS_FATAL("No results_file_name param provided");
        exit(1);
    }
    if(ros::param::get(ros::this_node::getName()+"/link_states_topic_name", this->link_states_topic_name) == false)
    {
        ROS_FATAL("No link_states_topic_name param provided");
        exit(1);
    }
    if(ros::param::get(ros::this_node::getName()+"/flower_map_topic_name", this->flower_map_topic_name) == false)
    {
        ROS_FATAL("No flower_map_topic_name param provided");
        exit(1);
    }
    if(ros::param::get(ros::this_node::getName()+"/manip_state_machine_topic_name", this->manip_state_machine_topic_name) == false)
    {
        ROS_FATAL("No manip_state_machine_topic_name param provided");
        exit(1);
    }

    // Wait for /gazebo/link_states to be published in order to get flower true poses and names
    gazebo_msgs::LinkStatesConstPtr link_states_msg_ptr = ros::topic::waitForMessage<gazebo_msgs::LinkStates>(this->link_states_topic_name, nh);
    
    // Find flowers in link states by name
    for(unsigned int i = 0; i < link_states_msg_ptr->name.size(); i++)
    {
        if(link_states_msg_ptr->name.at(i).find("flower_link") != std::string::npos) // If link is a flower, record pose, names, and tf
        {
            this->flower_true_poses.push_back(link_states_msg_ptr->pose.at(i));
            this->flower_names.push_back(link_states_msg_ptr->name.at(i));
            geometry_msgs::TransformStamped flower_transform;
            flower_transform.header.stamp = ros::Time::now();
            flower_transform.header.frame_id = "world";
            flower_transform.child_frame_id = link_states_msg_ptr->name.at(i);
            flower_transform.transform.translation.x = link_states_msg_ptr->pose.at(i).position.x;
            flower_transform.transform.translation.y = link_states_msg_ptr->pose.at(i).position.y;
            flower_transform.transform.translation.z = link_states_msg_ptr->pose.at(i).position.z;
            flower_transform.transform.rotation.x = link_states_msg_ptr->pose.at(i).orientation.x;
            flower_transform.transform.rotation.y = link_states_msg_ptr->pose.at(i).orientation.y;
            flower_transform.transform.rotation.z = link_states_msg_ptr->pose.at(i).orientation.z;
            flower_transform.transform.rotation.w = link_states_msg_ptr->pose.at(i).orientation.w;
            this->gazebo_truth_buffer.setTransform(flower_transform, "gazebo", true);
        }
    }
    this->num_flowers = flower_names.size();

    // Initialize ROS subscribers
    this->link_states_sub = nh.subscribe(this->link_states_topic_name, 1, &ResultsEvaluator::linkStatesCallback, this);
    this->flower_map_sub = nh.subscribe(this->flower_map_topic_name, 1, &ResultsEvaluator::flowerMapCallback, this);
    this->manip_state_machine_sub = nh.subscribe(this->manip_state_machine_topic_name, 1, &ResultsEvaluator::manipStateMachineCallback, this);
}

void ResultsEvaluator::saveResultsFile()
{
    this->results_file.open(this->results_file_name, std::ios::out | std::ios::trunc);
    // TODO: fill out results file format
}

void ResultsEvaluator::setWorldToBaseLinkTF()
{
    // Set world -> base_link truth transform
    for(unsigned int i = 0; i < link_states.name.size(); i++)
    {
        if(link_states.name.at(i).find("base_link") != std::string::npos) // If link is base_link, record pose, names, and tf
        {
            geometry_msgs::TransformStamped base_link_transform;
            base_link_transform.header.stamp = ros::Time::now();
            base_link_transform.header.frame_id = "world";
            base_link_transform.child_frame_id = "base_link";
            base_link_transform.transform.translation.x = link_states.pose.at(i).position.x;
            base_link_transform.transform.translation.y = link_states.pose.at(i).position.y;
            base_link_transform.transform.translation.z = link_states.pose.at(i).position.z;
            base_link_transform.transform.rotation.x = link_states.pose.at(i).orientation.x;
            base_link_transform.transform.rotation.y = link_states.pose.at(i).orientation.y;
            base_link_transform.transform.rotation.z = link_states.pose.at(i).orientation.z;
            base_link_transform.transform.rotation.w = link_states.pose.at(i).orientation.w;
            this->gazebo_truth_buffer.setTransform(base_link_transform, "gazebo");
        }
    }

}

void ResultsEvaluator::setWorldtoEndEffectorLinkTF()
{
    // Set truth tf of world -> base_link from gazebo link_states
    
}

void ResultsEvaluator::updateFlowersDetected()
{
    // Set truth tf of world -> base_link
    setWorldToBaseLinkTF();

    // For each flower, transform its location to the world frame
    geometry_msgs::PointStamped detected_flower_position;
    for(unsigned int i = 0; i < this->detected_flowers.map.size(); i++)
    {
        gazebo_truth_buffer.transform(detected_flowers.map.at(i).point, detected_flower_position, "world");
        double distance_squared;
        double best_distance_squared = std::numeric_limits<double>::max();
        unsigned int best_id = -1;
        for(unsigned int j = 0; j < this->flower_true_poses.size(); i++)
        {
            geometry_msgs::Point true_flower_position = this->flower_true_poses.at(j).position;
            distance_squared = pow(detected_flower_position.point.x - true_flower_position.x, 2.0)
                                + pow(detected_flower_position.point.y - true_flower_position.y, 2.0)
                                + pow(detected_flower_position.point.z - true_flower_position.z, 2.0);
            if(distance_squared < best_distance_squared)
            {
                best_distance_squared = distance_squared;
                best_id = j;
                // TODO: need to record mapping of true ID to detected id as well. Need that for approach and pollinated
            }
        }
        if(best_id == -1) // No closest flower found
        {
            ROS_WARN("Detected flower index %u, id %u not matched to any true flower", i, this->detected_flowers.map.at(i).id);
        }
        else
        {
            if(this->detected_flower_true_ids.size() > 0) // If there are already some detected flower IDs recorded, need to check that closest flower ID not already recorded
            {
                bool matching_id_found = false;
                for(unsigned int k = 0; k < this->detected_flower_true_ids.size(); k++)
                {
                    if(best_id == this->detected_flower_true_ids.at(k)) // If ID is already recorded, take not and stop searching list
                    {
                        matching_id_found = true;
                        break;
                    }
                }
                if(matching_id_found == false) // If ID not already recorded, add it to list
                {
                    this->detected_flower_true_ids.push_back(best_id);
                }
            }
            else // No IDs recorded so far, so add it to list
            {
                this->detected_flower_true_ids.push_back(best_id);
            }

        }
    }
}

void ResultsEvaluator::linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    this->link_states = *msg;
}

void ResultsEvaluator::flowerMapCallback(const manipulation_common::FlowerMap::ConstPtr& msg)
{
    detected_flowers = *msg;
    updateFlowersDetected();
}

void ResultsEvaluator::manipStateMachineCallback(const manipulation_common::StateMachine::ConstPtr& msg)
{
    
}
