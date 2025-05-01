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
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <vector>
#include <mutex>

using namespace std;

namespace ros_jog
{

class ControllerBase
{
public:
  ControllerBase(string controller_service_name);

  virtual void updateDevice(const string& device_name);
  vector<string> getControllers();
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  vector<double> getJointPositions();

protected:
  string name_;
  string ns_;
  string controller_service_name_;
  vector<double> joint_positions_;
  ros::Subscriber joint_state_sub_;
  bool connected_ = false;
  ros::NodeHandle nh_;
  std::mutex joint_positions_mutex_;
};

class PoseClient : public ControllerBase
{
public:
  PoseClient(const std::string& service = "/query_pose");

  void updateDevice(const string& device_name) override;
  void
  publishSetpoint(const taskspace_control_msgs::PoseTwistSetpoint& setpoint);
  bool getPose(geometry_msgs::Pose& pose);

private:
  ros::Publisher setpoint_pub_;
  ros::ServiceClient pose_client_;
};

class TrajClient : public ControllerBase
{
public:
  TrajClient();

  void updateDevice(const string& device_name) override;
  void publishJoints(double value, int index);

private:
  std::unique_ptr<
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>
      ac_;
  std::vector<std::string> joint_names_;
};

}  // namespace ros_jog

#endif  // CONTROLLER_CLIENTS_H
