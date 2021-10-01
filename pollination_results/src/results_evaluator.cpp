#include <pollination_results/results_evaluator.hpp>

ResultsEvaluator::ResultsEvaluator() :
    tf2_listener(tf_tree_buffer)
{
    ros::NodeHandle nh;
    // Get launch params
    if(ros::param::get(ros::this_node::getName()+"/flower_stats_results_file_name", this->flower_stats_results_file_name) == false)
    {
        ROS_FATAL("No flower_stats_results_file_name param provided");
        exit(1);
    }
    if(ros::param::get(ros::this_node::getName()+"/flower_mapping_error_results_file_name", this->flower_mapping_error_results_file_name) == false)
    {
        ROS_FATAL("No flower_mapping_error_results_file_name param provided");
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
    if(ros::param::get(ros::this_node::getName()+"/mission_complete_topic_name", this->mission_complete_topic_name) == false)
    {
        ROS_FATAL("No mission_complete_topic_name param provided");
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
        ROS_FATAL("No flower_pollinated_zenith_angle_range param provided");
        exit(1);
    }
    this->flower_pollinated_zenith_angle_range *= M_PI/180.0; // deg -> rad
    if(ros::param::get(ros::this_node::getName()+"/loop_rate", this->loop_rate) == false)
    {
        ROS_FATAL("No loop_rate param provided");
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
    this->mission_complete_sub = nh.subscribe(this->mission_complete_topic_name, 1, &ResultsEvaluator::missionCompleteCallback, this);

    // Record start time for computing total mission duration at end
    this->mission_start_time = ros::Time::now().toSec();

    // Set results evaluator initial state
    this->evaluator_state = _idle;
}

void ResultsEvaluator::run()
{
    ros::Rate rate(this->loop_rate);
    while(ros::ok())
    {
        if(this->evaluator_state == _approaching)
        {
            ROS_INFO("approaching...");
            updateFlowersApproached();
        }
        else if(this->evaluator_state == _pollinating)
        {
            ROS_INFO("pollinating...");
            updateFlowersPollinated();
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void ResultsEvaluator::saveResultsFiles()
{
    // Assemble stats file strings
    std::string stats_header = "total_flowers,num_flower_detected,num_flowers_approached,num_flowers_pollinated,per_flowers_detected,per_flowers_pollinated,per_flowers_pollinated_out_of_approached,mission_time_duration\n";
    unsigned int num_flowers_detected = this->detected_flower_true_ids.size();
    unsigned int num_flowers_approached = this->approached_flower_true_ids.size();
    unsigned int num_flowers_pollinated = this->pollinated_flower_true_ids.size();
    float per_flowers_detected = (float)num_flowers_detected / (float)this->num_flowers * 100.0;
    float per_flowers_pollinated = (float)num_flowers_pollinated / (float)this->num_flowers * 100.0;
    float per_flowers_pollinated_out_of_approached = (float)num_flowers_pollinated / (float)num_flowers_approached * 100.0;
    double mission_time_duration = ros::Time::now().toSec() - this->mission_start_time;
    std::string stats_data = std::to_string(this->num_flowers) + "," + 
        std::to_string(num_flowers_detected) + "," + 
        std::to_string(num_flowers_approached) + "," +
        std::to_string(num_flowers_pollinated) + "," +
        std::to_string(per_flowers_detected) + "," +
        std::to_string(per_flowers_pollinated) + "," +
        std::to_string(per_flowers_pollinated_out_of_approached) + "," +
        std::to_string(mission_time_duration);
    
    // Write to stats file
    this->flower_stats_results_file.open(this->flower_stats_results_file_name, std::ios::out | std::ios::trunc);
    this->flower_stats_results_file << stats_header;
    this->flower_stats_results_file << stats_data;
    this->flower_stats_results_file.close();

    // Write to mapping error file
    std::string mapping_error_header = "x,y,z\n";
    this->flower_mapping_error_results_file.open(this->flower_mapping_error_results_file_name, std::ios::out | std::ios::trunc);
    this->flower_mapping_error_results_file << mapping_error_header;
    for(unsigned int i = 0; i < this->detected_flower_estimated_positions.size(); i++)
    {
        double x_err = (this->detected_flower_estimated_positions.at(i).point.x - this->flower_true_poses.at(this->detected_flower_true_ids.at(i)).position.x) * 100.0; // cm
        double y_err = (this->detected_flower_estimated_positions.at(i).point.y - this->flower_true_poses.at(this->detected_flower_true_ids.at(i)).position.y) * 100.0; // cm
        double z_err = (this->detected_flower_estimated_positions.at(i).point.z - this->flower_true_poses.at(this->detected_flower_true_ids.at(i)).position.z) * 100.0; // cm
        std::string mapping_error_data = std::to_string(x_err) + "," + std::to_string(y_err) + "," + std::to_string(z_err);
        if(i < (this->detected_flower_estimated_positions.size() - 1)) // Add newline char to all lines except the final line
        {
            mapping_error_data += "\n";
        }
        this->flower_mapping_error_results_file << mapping_error_data;
    }
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
    geometry_msgs::TransformStamped base_link_to_end_effector_tf = tf_tree_buffer.lookupTransform("base_link", "j2n6s300_end_effector", ros::Time(0));

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
        for(unsigned int j = 0; j < this->flower_true_poses.size(); j++)
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
                        ROS_INFO("Duplicate flower detected (ID: %u, name: %s)", best_id, this->flower_names.at(best_id).c_str());
                        break;
                    }
                }
                if(matching_id_found == false) // If ID not already recorded, add it to list
                {
                    this->detected_flower_true_ids.push_back(best_id);
                    this->detected_flower_id_to_true_flower_id_map.insert(std::make_pair(this->detected_flowers.map.at(i).id, best_id));
                    this->detected_flower_estimated_positions.push_back(detected_flower_position);
                    ROS_INFO("detected flower index %u", best_id);
                }
            }
            else // No IDs recorded so far, so add it to list
            {
                this->detected_flower_true_ids.push_back(best_id);
                this->detected_flower_id_to_true_flower_id_map.insert(std::make_pair(this->detected_flowers.map.at(i).id, best_id));
                this->detected_flower_estimated_positions.push_back(detected_flower_position);
                ROS_INFO("detected flower index %u", best_id);
            }
        }
    }
}

void ResultsEvaluator::updateFlowersApproached()
{
    // Set truth tf of world -> end_effector
    setWorldtoEndEffectorLinkTF();

    // Look up tf of world -> end_effector
    geometry_msgs::TransformStamped end_effector_pose = this->gazebo_truth_buffer.lookupTransform("world", "j2n6s300_end_effector", ros::Time(0));

    // Check if distance to flower being approached is less than distance threshold
    unsigned int true_flower_id = this->detected_flower_id_to_true_flower_id_map.find(this->manip_state_machine_msg.target_flower_id)->second;
    ROS_INFO("target_flower_id = %u, true_flower_id = %u",this->manip_state_machine_msg.target_flower_id, true_flower_id);
    geometry_msgs::Point true_flower_position = this->flower_true_poses.at(true_flower_id).position;
    ROS_INFO("true flower position = [%f, %f, %f]", true_flower_position.x, true_flower_position.y, true_flower_position.z);
    ROS_INFO("end effector position = [%f, %f, %f]", end_effector_pose.transform.translation.x, end_effector_pose.transform.translation.y, end_effector_pose.transform.translation.z);
    double distance_error = sqrt(pow(end_effector_pose.transform.translation.x - true_flower_position.x, 2.0) +
                                 pow(end_effector_pose.transform.translation.y - true_flower_position.y, 2.0) +
                                 pow(end_effector_pose.transform.translation.z - true_flower_position.z, 2.0));
    ROS_INFO("distance_error = %lf",distance_error);
    if(distance_error < this->approached_distance_threshold) // If end effector is close enough to true flower, record flower as approached
    {
        if(this->approached_flower_true_ids.size() > 0) // If there are already some approached flower IDs recorded, need to check that the approached flower ID is not already recorded
        {
            bool matching_id_found = false;
            for(unsigned int k = 0; k < this->approached_flower_true_ids.size(); k++)
            {
                if(true_flower_id == this->approached_flower_true_ids.at(k))
                {
                    matching_id_found = true;
                    ROS_INFO("Duplicate flower approached (ID: %u, name: %s)",true_flower_id, this->flower_names.at(true_flower_id).c_str());
                    break;
                }
            }
            if(matching_id_found == false) // If ID not already recorded, add it to list
            {
                this->approached_flower_true_ids.push_back(true_flower_id);
            }
        }
        else // No IDs recorded so far, so add it to list
        {
            this->approached_flower_true_ids.push_back(true_flower_id);
        }
        this->evaluator_state = _idle;
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
    geometry_msgs::TransformStamped flower_in_end_effector_pose;
    try
    {
        flower_in_end_effector_pose = this->gazebo_truth_buffer.lookupTransform("j2n6s300_end_effector", flower_frame_name, ros::Time(0));
    }
    catch(tf2::ExtrapolationException& e)
    {
        ROS_WARN(e.what());
        return;
    }
    ROS_INFO("look up flower in end effector transform");
    
    // Look up tf of flower -> end_effector
    geometry_msgs::TransformStamped end_effector_in_flower_pose;
    try
    {
        end_effector_in_flower_pose = this->gazebo_truth_buffer.lookupTransform(flower_frame_name, "j2n6s300_end_effector", ros::Time(0));
    }
    catch(tf2::ExtrapolationException& e)
    {
        ROS_WARN(e.what());
        return;
    }
    ROS_INFO("look up end effector in flower transform");

    // Compute distance and angle metrics needed to check if flower has been pollinated by end effector
    double flower_radial_position_error = hypot(flower_in_end_effector_pose.transform.translation.x, flower_in_end_effector_pose.transform.translation.y);
    double flower_z_error = flower_in_end_effector_pose.transform.translation.z;
    double end_effector_distance_error = sqrt(pow(end_effector_in_flower_pose.transform.translation.x, 2.0) +
                                              pow(end_effector_in_flower_pose.transform.translation.y, 2.0) +
                                              pow(end_effector_in_flower_pose.transform.translation.z, 2.0));
    double end_effector_zenith_angle = acos(end_effector_in_flower_pose.transform.translation.z / end_effector_distance_error);
    ROS_INFO("flower_radial_position_error = %lf",flower_radial_position_error);
    ROS_INFO("flower_z_error = %lf",flower_z_error);
    ROS_INFO("end_effector_distance_error = %lf",end_effector_distance_error);
    ROS_INFO("end_effector_zenith_angle = %lf",180.0/M_PI*end_effector_zenith_angle);

    // Check if flower is within cylindrical projection of end effector and end effector is within shperical sector of flower
    if((flower_radial_position_error < this->end_effector_tip_diameter) &&
            (flower_z_error > 0.0 && flower_z_error < this->end_effector_extension_length) &&
            (end_effector_distance_error < this->end_effector_extension_length) &&
            (end_effector_zenith_angle < this->flower_pollinated_zenith_angle_range))
    {
        if(this->pollinated_flower_true_ids.size() > 0) // If there are already some pollinated flower IDs recorded, need to check that the pollinated flower ID is not already recorded
        {
            bool matching_id_found = false;
            for(unsigned int k = 0; k < this->pollinated_flower_true_ids.size(); k++)
            {
                if(true_flower_id == this->pollinated_flower_true_ids.at(k))
                {
                    matching_id_found = true;
                    ROS_INFO("Duplicate flower pollinated (ID: %u, name: %s)",true_flower_id, this->flower_names.at(true_flower_id).c_str());
                    break;
                }
            }
            if(matching_id_found == false) // If ID not already found, add it to the list
            {
                this->pollinated_flower_true_ids.push_back(true_flower_id);
            }
        }
        else // No IDs recorded so far, so add it to list
        {
            this->pollinated_flower_true_ids.push_back(true_flower_id);
        }
        this->evaluator_state = _idle;
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
    if(this->manip_state_machine_msg.state == "approaching")
    {
        this->evaluator_state = _approaching;
    }
    else if(this->manip_state_machine_msg.state == "pollinating")
    {
        this->evaluator_state = _pollinating;
    }
    else
    {
        this->evaluator_state = _idle;
    }
    ROS_INFO("manipStateMachineCallback");
}

void ResultsEvaluator::missionCompleteCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data == true)
    {
        saveResultsFiles();
    }
}
