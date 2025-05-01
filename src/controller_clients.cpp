#include "ros_jog/controller_clients.h"
#include <ros/ros.h>
#include <ros/master.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <vector>
#include <algorithm>

namespace ros_jog
{

//--------------------------------------------------------------
// ControllerBase Class
//--------------------------------------------------------------

ControllerBase::ControllerBase(std::string controller_service_name)
{
  controller_service_name_ = controller_service_name;
  std::vector<std::string> controllers = getControllers();
  if (controllers.empty())
  {
    ROS_ERROR(
        "No controllers found in the ROS network with this service available.");
    return;
  }
  ROS_INFO("Found %zu controllers in the ROS network.", controllers.size());
  for (const auto& controller : controllers)
  {
    ROS_INFO("Pose controller found: %s", controller.c_str());
  }
}

//--------------------------------------------------------------

void ControllerBase::updateDevice(const std::string& device_name)
{
  // Update the controller name
  name_ = device_name;
  connected_ = true;

  // Extract the text before the previous backslash
  size_t last_slash_pos = name_.find_last_of('/');
  ns_ = (last_slash_pos != std::string::npos) ?
            name_.substr(0, last_slash_pos) :
            "";

  ros::NodeHandle nh_(name_);

  // Create publisher and service client
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>(
      ns_ + "/joint_states", 1, &ControllerBase::jointStateCallback, this);

  ROS_INFO("Connected to controller: %s", name_.c_str());
}

//--------------------------------------------------------------

std::vector<std::string> ControllerBase::getControllers()
{
  std::vector<std::string> controllers;
  XmlRpc::XmlRpcValue args, result, payload;

  args[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", args, result, payload, true))
  {
    ROS_ERROR("Failed to query ROS master for system state.");
    return controllers;
  }

  // Extract the list of services from the payload
  XmlRpc::XmlRpcValue& services = payload[2];

  for (int i = 0; i < services.size(); i++)
  {
    std::string service_name = static_cast<std::string>(services[i][0]);

    // Check if the topic name contains the match string
    if (service_name.find(controller_service_name_) != std::string::npos)
    {
      // Extract the namespace of the
      std::string ns =
          service_name.substr(0, service_name.find(controller_service_name_));
      if (ns.empty())
      {
        // If the namespace is empty, it means the topic is in the root
        // namespace
        ns = "/";
      }
      controllers.push_back(ns);
    }
  }

  return controllers;
}

//--------------------------------------------------------------

void ControllerBase::jointStateCallback(
    const sensor_msgs::JointState::ConstPtr& msg)
{
  // Save the joint positions in an array of floats
  std::lock_guard<std::mutex> lock(joint_positions_mutex_);
  joint_positions_ = { msg->position.begin(), msg->position.end() };
}

//--------------------------------------------------------------

std::vector<double> ControllerBase::getJointPositions()
{
  std::lock_guard<std::mutex> lock(joint_positions_mutex_);
  return joint_positions_;
}

//--------------------------------------------------------------
// PoseClient Class
//--------------------------------------------------------------

PoseClient::PoseClient(const std::string& service) : ControllerBase(service) {}

//--------------------------------------------------------------

void PoseClient::updateDevice(const std::string& pose_controller_name)
{
  ControllerBase::updateDevice(pose_controller_name);

  std::string setpoint_topic;
  if (!nh_.getParam(name_ + "/setpoint_topic", setpoint_topic))
  {
    ROS_ERROR("Failed to get setpoint_topic parameter for controller %s",
              name_.c_str());
    return;
  }

  setpoint_pub_ = nh_.advertise<taskspace_control_msgs::PoseTwistSetpoint>(
      name_ + "/" + setpoint_topic, 1, true);
  pose_client_ = nh_.serviceClient<taskspace_control_msgs::QueryPose>(name_ +
                                                                      "/query_"
                                                                      "pose");
}

//--------------------------------------------------------------

void PoseClient::publishSetpoint(
    const taskspace_control_msgs::PoseTwistSetpoint& setpoint)
{
  if (!connected_)
  {
    return;
  }
  setpoint_pub_.publish(setpoint);
}

//--------------------------------------------------------------

bool PoseClient::getPose(geometry_msgs::Pose& pose)
{
  if (!connected_)
  {
    return false;
  }

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
// TrajClient Class
//--------------------------------------------------------------

TrajClient::TrajClient()
  : ControllerBase("/query_state"), ac_(nullptr), joint_names_()
{
}

//--------------------------------------------------------------

void TrajClient::updateDevice(const std::string& pose_controller_name)
{
  ControllerBase::updateDevice(pose_controller_name);

  ac_ = std::make_unique<
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(
      name_ + "/follow_joint_trajectory", true);
  ac_->waitForServer();

  // Get Joint Names
  joint_names_.clear();
  if (nh_.getParam(name_ + "/joints", joint_names_))
  {
  }
  else
  {
    ROS_ERROR("Failed to get joints parameter for controller %s",
              name_.c_str());
  }

  ROS_INFO("Connected to simple action client");
}

//--------------------------------------------------------------

void TrajClient::publishJoints(double position, int index)
{
  if (!ac_)
  {
    ROS_ERROR("Action client not initialized!");
    return;
  }


  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = joint_names_;  // Replace with your joint names

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = std::vector<double>(joint_names_.size(), 0.0);
  ;  // Target positions
  {
    std::lock_guard<std::mutex> lock(joint_positions_mutex_);
    point.positions = joint_positions_;
  }
  point.positions[index] = position;
  point.time_from_start = ros::Duration(0.3);  // 2 seconds to reach
  point.velocities = std::vector<double>(joint_names_.size(), 0.0);
  ;
  point.accelerations = std::vector<double>(joint_names_.size(), 0.0);
  ;

  goal.trajectory.points.push_back(point);

  ac_->sendGoal(goal);
}

//--------------------------------------------------------------

}  // namespace ros_jog