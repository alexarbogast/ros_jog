#include "ros_jog/controller_clients.h"
#include <ros/ros.h>

namespace ros_jog
{

ControllerClient::ControllerClient(const std::string& name) : name_(name)
{
  ros::NodeHandle nh;

  // Get setpoint topic from parameter server
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

  ROS_INFO("Connected to controller: %s", name_.c_str());
}

void ControllerClient::publishSetpoint(
    const taskspace_control_msgs::PoseTwistSetpoint& setpoint)
{
  setpoint_pub_.publish(setpoint);
}

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

JointControllerClient::JointControllerClient(const std::string& name) 
  : name_(name) {
  
  ros::NodeHandle nh;

  // Get joint names from parameter server
  if (!nh.getParam(name_ + "/joints", joint_names_))
  {
    ROS_ERROR("Failed to get joints parameter for controller %s",
              name_.c_str());
    return;
  }
  n_joints_ = joint_names_.size();

  // Create publisher
  joint_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
    name_ + "/command", 1, true);

  //Create joint state subscriber
  // ros::Subscriber joint_state_sub = nh.subscribe(
  //   "/joint_states", 1, &JointControllerClient::jointStateCallback, this);
  
  ROS_INFO("Connected to controller: %s", name_.c_str());
}

bool JointControllerClient::moveJoint(const std::vector<double>& joint_goal) {
  if (joint_goal.size() != static_cast<size_t>(n_joints_)) {
    ROS_ERROR("Joint goal size (%zu) does not match number of joints (%d)", 
              joint_goal.size(), n_joints_);
    return false;
  }

  // Create and populate the Float64MultiArray message
  std_msgs::Float64MultiArray command_msg;
  command_msg.data = joint_goal;

  // Publish the joint positions
  joint_pub_.publish(command_msg);

  return true;
}

// void JointControllerClient::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
//   // Update the local joint positions
//   std::lock_guard<std::mutex> lock(joint_state_mutex_);
//   current_joint_positions_.clear();
//   for (const auto& name : joint_names_) {
//     auto it = std::find(msg->name.begin(), msg->name.end(), name);
//     if (it != msg->name.end()) {
//       size_t index = std::distance(msg->name.begin(), it);
//       current_joint_positions_.push_back(msg->position[index]);
//     }
//   }
// }

// std::vector<double> JointControllerClient::getCurrentJointPositions() const {
//   std::lock_guard<std::mutex> lock(joint_state_mutex_);
//   return current_joint_positions_;
// }

} // namespace ros_jog 