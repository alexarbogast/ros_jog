#include "ros_jogger/controller_clients.h"
#include <ros/ros.h>

namespace ros_jogger {

ControllerClient::ControllerClient(const std::string& name) : name_(name) {
  ros::NodeHandle nh;
  
  // Get setpoint topic from parameter server
  std::string setpoint_topic;
  if (!nh.getParam(name_ + "/setpoint_topic", setpoint_topic)) {
    ROS_ERROR("Failed to get setpoint_topic parameter for controller %s", name_.c_str());
    return;
  }
  
  // Create publisher and service client
  setpoint_pub_ = nh.advertise<taskspace_control_msgs::PoseTwistSetpoint>(
    name_ + "/" + setpoint_topic, 1, true);
  pose_client_ = nh.serviceClient<taskspace_control_msgs::QueryPose>(
    name_ + "/query_pose");
  
  ROS_INFO("Connected to controller: %s", name_.c_str());
}

void ControllerClient::publishSetpoint(const taskspace_control_msgs::PoseTwistSetpoint& setpoint) {
  setpoint_pub_.publish(setpoint);
}

bool ControllerClient::getPose(geometry_msgs::Pose& pose) {
  taskspace_control_msgs::QueryPose srv;
  if (!pose_client_.call(srv)) {
    ROS_ERROR("Failed to call query_pose service for controller %s", name_.c_str());
    return false;
  }
  pose = srv.response.pose;
  return true;
}

JointControllerClient::JointControllerClient(const std::string& name) 
  : name_(name), joint_traj_client_(name + "/follow_joint_trajectory", true) {
  
  ros::NodeHandle nh;
  
  // Get joint names from parameter server
  if (!nh.getParam(name_ + "/joints", joint_names_)) {
    ROS_ERROR("Failed to get joints parameter for controller %s", name_.c_str());
    return;
  }
  n_joints_ = joint_names_.size();
  
  ROS_INFO("Waiting for follow_joint_trajectory action");
  if (!joint_traj_client_.waitForServer(ros::Duration(5.0))) {
    ROS_ERROR("Failed to connect to follow_joint_trajectory action server for controller %s", 
              name_.c_str());
    return;
  }
  
  ROS_INFO("Connected to controller: %s", name_.c_str());
}

bool JointControllerClient::moveJoint(const std::vector<double>& joint_goal, 
                                    double duration, bool blocking) {
  if (joint_goal.size() != static_cast<size_t>(n_joints_)) {
    ROS_ERROR("Joint goal size (%zu) does not match number of joints (%d)", 
              joint_goal.size(), n_joints_);
    return false;
  }
  
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = joint_names_;
  
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = joint_goal;
  point.velocities.resize(n_joints_, 0.0);
  point.accelerations.resize(n_joints_, 0.0);
  point.time_from_start = ros::Duration(duration);
  goal.trajectory.points.push_back(point);
  
  joint_traj_client_.sendGoal(goal);
  
  if (blocking) {
    return waitForGoal();
  }
  return true;
}

bool JointControllerClient::waitForGoal() {
  if (!joint_traj_client_.waitForResult(ros::Duration(30.0))) {
    ROS_ERROR("Timeout waiting for joint trajectory result");
    return false;
  }
  
  if (joint_traj_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Joint trajectory execution failed: %s", 
              joint_traj_client_.getState().toString().c_str());
    return false;
  }
  
  return true;
}

} // namespace ros_jogger 