// Copyright 2025 Alex Arbogast, Rowan Ramamurthy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROLLER_CLIENTS_H
#define CONTROLLER_CLIENTS_H

#include <ros/ros.h>
#include <taskspace_control_msgs/PoseTwistSetpoint.h>
#include <taskspace_control_msgs/QueryPose.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

namespace ros_jog
{

class ControllerClient
{
public:
  ControllerClient(const std::string& name);

  void
  publishSetpoint(const taskspace_control_msgs::PoseTwistSetpoint& setpoint);
  bool getPose(geometry_msgs::Pose& pose);

private:
  std::string name_;
  ros::Publisher setpoint_pub_;
  ros::ServiceClient pose_client_;
};

class JointControllerClient
{
public:
  JointControllerClient(const std::string& name);
  
  bool moveJoint(const std::vector<double>& joint_goal);
  // void JointControllerClient::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  // std::vector<double> JointControllerClient::getCurrentJointPositions()

private:
  std::string name_;
  std::vector<std::string> joint_names_;
  int n_joints_;
  ros::Publisher joint_pub_;
  // ros::Subscriber joint_state_sub_;
};

}  // namespace ros_jog

#endif  // CONTROLLER_CLIENTS_H
