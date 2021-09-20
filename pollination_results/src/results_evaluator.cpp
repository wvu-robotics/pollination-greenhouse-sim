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
    if(ros::param::get(ros::this_node::getName()+"/approached_distance_threshold", this->approached_distance_threshold) == false)
    {
        ROS_FATAL("No approached_distance_threshold param provided");
        exit(1);
    }
    this->approached_distance_threshold *= 0.01; // cm -> m
    if(ros::param::get(ros::this_node::getName()+"/end_effector_tip_diameter", this->end_effector_tip_diameter) == false)
    {
        ROS_FATAL("No end_effector_tip_diameter param provided");
        exit(1);
    }
    this->end_effector_tip_diameter *= 0.01; // cm -> m
    if(ros::param::get(ros::this_node::getName()+"/end_effector_extension_length", this->end_effector_extension_length) == false)
    {
        ROS_FATAL("No end_effector_extension_length param provided");
        exit(1);
    }
    this->end_effector_extension_length *= 0.01; // cm -> m
    if(ros::param::get(ros::this_node::getName()+"/flower_pollinated_zenith_angle_range", this->flower_pollinated_zenith_angle_range) == false)
    {
        ROS_FATAL("No flower_pollinated_zenith_angle_range");
        exit(1);
    }
    this->flower_pollinated_zenith_angle_range *= M_PI/180.0; // deg -> rad

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
    setWorldToBaseLinkTF();
    
    // Look up transform from base_link -> end_effector from broadcast tf tree
    geometry_msgs::TransformStamped base_link_to_end_effector_tf = tf_tree_buffer.lookupTransform("j2n6s300_end_effector", "base_link", ros::Time(0));

    // Set base_link -> end_effector transform in truth tf buffer
    this->gazebo_truth_buffer.setTransform(base_link_to_end_effector_tf, "gazebo");
}

void ResultsEvaluator::updateFlowersDetected()
{
    // Set truth tf of world -> base_link
    setWorldToBaseLinkTF();

    // For each flower, transform its location to the world frame
    geometry_msgs::PointStamped detected_flower_position;
    this->detected_flower_id_to_true_flower_id_map.clear();
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
                    if(best_id == this->detected_flower_true_ids.at(k)) // If ID is already recorded, take note and stop searching list
                    {
                        matching_id_found = true;
                        break;
                    }
                }
                if(matching_id_found == false) // If ID not already recorded, add it to list
                {
                    this->detected_flower_true_ids.push_back(best_id);
                    this->detected_flower_id_to_true_flower_id_map.insert(std::make_pair(this->detected_flowers.map.at(i).id, best_id));
                }
            }
            else // No IDs recorded so far, so add it to list
            {
                this->detected_flower_true_ids.push_back(best_id);
                this->detected_flower_id_to_true_flower_id_map.insert(std::make_pair(this->detected_flowers.map.at(i).id, best_id));
            }

        }
    }
}

void ResultsEvaluator::updateFlowersApproached()
{
    // Set truth tf of world -> end_effector
    setWorldtoEndEffectorLinkTF();

    // Look up tf of world -> end_effector
    geometry_msgs::TransformStamped end_effector_pose = this->gazebo_truth_buffer.lookupTransform("j2n6s300_end_effector", "world", ros::Time(0));

    // Check if distance to flower being approached is less than distance threshold
    unsigned int true_flower_id = this->detected_flower_id_to_true_flower_id_map.find(this->manip_state_machine_msg.target_flower_id)->second;
    geometry_msgs::Point true_flower_position = this->flower_true_poses.at(true_flower_id).position;
    double distance_error = sqrt(pow(end_effector_pose.transform.translation.x - true_flower_position.x, 2.0) +
                                 pow(end_effector_pose.transform.translation.y - true_flower_position.y, 2.0) +
                                 pow(end_effector_pose.transform.translation.z - true_flower_position.z, 2.0));
    if(distance_error < this->approached_distance_threshold) // If end effector is close enough to true flower, record flower as approached
    {
        this->approached_flower_true_ids.push_back(true_flower_id); // TODO: need to add a check, similar to that for detected flowers, to check that this true ID hasn't already been approached to avoid double counting
    }
}

void ResultsEvaluator::updateFlowersPollinated()
{
    // Set truth tf of world -> end_effector
    setWorldtoEndEffectorLinkTF();

    // Get frame name of flower being pollinated
    unsigned int true_flower_id = this->detected_flower_id_to_true_flower_id_map.find(this->manip_state_machine_msg.target_flower_id)->second;
    std::string flower_frame_name = this->flower_names.at(true_flower_id);

    // Look up tf of end_effector -> flower
    geometry_msgs::TransformStamped flower_in_end_effector_pose = this->gazebo_truth_buffer.lookupTransform(flower_frame_name, "j2n6s300_end_effector", ros::Time(0));
    
    // Look up tf of flower -> end_effector
    geometry_msgs::TransformStamped end_effector_in_flower_pose = this->gazebo_truth_buffer.lookupTransform("j2n6s300_end_effector", flower_frame_name, ros::Time(0));

    // Compute distance and angle metrics needed to check if flower has been pollinated by end effector
    double flower_radial_position_error = hypot(flower_in_end_effector_pose.transform.translation.x, flower_in_end_effector_pose.transform.translation.y);
    double flower_z_error = flower_in_end_effector_pose.transform.translation.z;
    double end_effector_distance_error = sqrt(pow(end_effector_in_flower_pose.transform.translation.x, 2.0) +
                                              pow(end_effector_in_flower_pose.transform.translation.y, 2.0) +
                                              pow(end_effector_in_flower_pose.transform.translation.z, 2.0));
    double end_effector_zenith_angle = acos(end_effector_in_flower_pose.transform.translation.z / end_effector_distance_error);

    // Check if flower is within cylindrical projection of end effector and end effector is within shperical sector of flower
    if((flower_radial_position_error < this->end_effector_tip_diameter) &&
            (flower_z_error > 0.0 && flower_z_error < this->end_effector_extension_length) &&
            (end_effector_distance_error < this->end_effector_extension_length) &&
            (end_effector_zenith_angle < this->flower_pollinated_zenith_angle_range))
    {
        this->pollinated_flower_true_ids.push_back(true_flower_id); // TODO: need to add a check, similar to that for detected flowers, to check that this true ID hasn't already been approached to avoid double counting
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
    this->manip_state_machine_msg = *msg;
    if(this->manip_state_machine_msg.state == "approaching") // TODO: update these state names and add additional logic if necessary
    {
        updateFlowersApproached();
    }
    else if(this->manip_state_machine_msg.state == "pollinating") // TODO: ^^^
    {
        updateFlowersPollinated();
    }
}
