#include "ros_jog/controller_clients.h"
#include <ros/ros.h>
#include <ros/master.h>
#include <vector>
#include <algorithm>

namespace ros_jog
{

ControllerClient::ControllerClient()
{
  // Initialize the controller by simply searching for all controller managers
  potential_controllers_ = GetPoseControllers();
  if (potential_controllers_.empty())
  {
    ROS_ERROR("No pose controllers found in the ROS network.");
    return;
  }
  ROS_INFO("Found %zu pose controllers in the ROS network.", potential_controllers_.size());
  for (const auto& controller : potential_controllers_)
  {
    ROS_INFO("Pose controller found: %s", controller.c_str());
  }
}

//--------------------------------------------------------------
// Function to get all available pose controllers
// in the ROS network

std::vector<std::string> ControllerClient::GetPoseControllers()
{
    std::vector<std::string> potential_managers;

    // Query the ROS master for all available services
    ros::master::V_TopicInfo topics;
    if (!ros::master::getTopics(topics))
    {
        ROS_ERROR("Failed to query ROS master for topics.");
        return potential_managers;
    }

    // Define the match string for controller manager topics
    const std::string match_str = "/parameter_descriptions";

    // Iterate through the list of topics
    for (const auto& topic : topics)
    {
        const std::string& topic_name = topic.name;
        // ROS_INFO("Topic name: %s", topic_name.c_str());

        // Check if the topic name contains the match string
        if (topic_name.find(match_str) != std::string::npos)
        {
            // Extract the namespace of the 
            std::string ns = topic_name.substr(0, topic_name.find(match_str));
            if (ns.empty())
            {
                // If the namespace is empty, it means the topic is in the root namespace
                ns = "/";
            }
            potential_managers.push_back(ns);
        }
    }

    // Sort the potential managers
    std::sort(potential_managers.begin(), potential_managers.end());

    return potential_managers;
}

//--------------------------------------------------------------

std::vector<std::string> ControllerClient::getPotentialConrollers()
{
  potential_controllers_ = GetPoseControllers();
  return potential_controllers_;
}

//--------------------------------------------------------------

void ControllerClient::publishSetpoint(
    const taskspace_control_msgs::PoseTwistSetpoint& setpoint)
{
  setpoint_pub_.publish(setpoint);
}

//--------------------------------------------------------------

bool ControllerClient::getPose(geometry_msgs::Pose& pose)
{
  taskspace_control_msgs::QueryPose srv;
  if (!pose_client_.call(srv))
  {
    ROS_ERROR("Failed to call query_pose service for controller %s",
              name_.c_str());
    return false;
  }
  pose = srv.response.pose;
  return true;
}

//--------------------------------------------------------------

void ControllerClient::updateDevice(const std::string& pose_controller_name)
{
  // Update the controller name
  name_ = pose_controller_name;

  // Extract the text before the previous backslash
  size_t last_slash_pos = name_.find_last_of('/');
  ns_ = (last_slash_pos != std::string::npos) 
                                      ? name_.substr(0, last_slash_pos) 
                                      : "";

  // Initialize the ROS node handle
  ros::NodeHandle nh(name_);

  std::string setpoint_topic;
  if (!nh.getParam(name_ + "/setpoint_topic", setpoint_topic))
  {
    ROS_ERROR("Failed to get setpoint_topic parameter for controller %s",
              name_.c_str());
    return;
  }

  // Create publisher and service client
  setpoint_pub_ = nh.advertise<taskspace_control_msgs::PoseTwistSetpoint>(
      name_ + "/" + setpoint_topic, 1, true);
  pose_client_ =
      nh.serviceClient<taskspace_control_msgs::QueryPose>(name_ + "/query_"
                                                                  "pose");
  joint_state_sub_ =
      nh.subscribe<sensor_msgs::JointState>(ns_ + "/joint_states", 1,
                                            &ControllerClient::jointStateCallback,
                                            this);


  ROS_INFO("Connected to controller: %s", name_.c_str());
}

//--------------------------------------------------------------

void ControllerClient::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // Save the joint positions in an array of floats
    joint_positions_ = {msg->position.begin(), msg->position.end()};
}

//--------------------------------------------------------------
std::vector<double> ControllerClient::getJointPositions()
{
    return joint_positions_;
}
//--------------------------------------------------------------

} // namespace ros_jog