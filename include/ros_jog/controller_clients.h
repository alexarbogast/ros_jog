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
#include <sensor_msgs/JointState.h>
#include <string>
#include <vector>

namespace ros_jog
{

class ControllerClient
{
public:
  ControllerClient();

  
  void publishSetpoint(const taskspace_control_msgs::PoseTwistSetpoint& setpoint);
  bool getPose(geometry_msgs::Pose& pose);
  void updateDevice(const std::string& device_name);
  std::vector<std::string> getPotentialConrollers();
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  std::vector<double> getJointPositions();

private:
  std::vector<std::string> GetPoseControllers();

  std::vector<std::string> potential_controllers_;
  std::string name_;
  std::string ns_;
  std::vector<double> joint_positions_;
  ros::Publisher setpoint_pub_;
  ros::ServiceClient pose_client_;
  ros::Subscriber joint_state_sub_;
};

}  // namespace ros_jog

#endif  // CONTROLLER_CLIENTS_H
