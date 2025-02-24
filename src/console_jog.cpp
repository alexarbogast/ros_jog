// Copyright 2025 Alex Arbogast
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

#include <termios.h>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <controller_manager_msgs/SwitchController.h>
#include <taskspace_control_msgs/PoseTwistSetpoint.h>
#include <taskspace_control_msgs/QueryPose.h>
#include <unistd.h>
#include <vector>

// parameters
const static std::string SETPOINT_TOPIC = "setpoint";

const static std::string JOINT_CONTROLLER_NAME = "joint_position_controller";
const static std::string POSE_CONTROLLER_NAME = "pose_controller";

#define LOAD_ROS_PARAM(nh, param_name, variable)                               \
  if (!nh.getParam(param_name, variable))                                      \
  {                                                                            \
    ROS_ERROR_STREAM("Failed to load parameter '"                              \
                     << param_name << "' from the parameter server.");         \
    return false;                                                              \
  }

#define KEYCODE_right 0x27
#define KEYCODE_a 0x61
#define KEYCODE_b 0x62
#define KEYCODE_c 0x63
#define KEYCODE_d 0x64
#define KEYCODE_e 0x65
#define KEYCODE_f 0x66
#define KEYCODE_g 0x67
#define KEYCODE_h 0x68
#define KEYCODE_i 0x69
#define KEYCODE_j 0x6a
#define KEYCODE_k 0x6b
#define KEYCODE_l 0x6c
#define KEYCODE_m 0x6d
#define KEYCODE_n 0x6e
#define KEYCODE_o 0x6f
#define KEYCODE_p 0x70
#define KEYCODE_q 0x71
#define KEYCODE_r 0x72
#define KEYCODE_s 0x73
#define KEYCODE_t 0x74
#define KEYCODE_u 0x75
#define KEYCODE_v 0x76
#define KEYCODE_w 0x77
#define KEYCODE_x 0x78
#define KEYCODE_y 0x79
#define KEYCODE_z 0x7a
#define KEYCODE_esc 0x1B

enum class JogType
{
  JOINT,
  CARTESIAN
};


enum class JogMode
{
  STEP,
  CONTINUOUS
};


class AxisMotionGenerator
{
public:
  AxisMotionGenerator(double vel, double acc, double jerk)
    : vel_(vel), acc_(acc), jerk_(jerk)
  {
  }

private:
  double vel_, acc_, jerk_;
};


class JointJogger
{
public:
  JointJogger() = default;

  bool init(ros::NodeHandle& nh, const std::string& controller_name)
  {
    nh_ = nh;
    controller_name_ = controller_name;

    LOAD_ROS_PARAM(nh, controller_name_ + "/joints", joint_names);
    synchronizeJointState();

    setpoint_pub_ = nh.advertise<std_msgs::Float64MultiArray>(
        controller_name + "/command", 1);
    return true;
  }

  inline const std::string& getControllerName() const
  {
    return controller_name_;
  }

  void setPosition(std::size_t joint_idx, double position)
  {
    state_.position[joint_idx] = position;
    std_msgs::Float64MultiArray msg;
    msg.data = state_.position;
    setpoint_pub_.publish(msg);
  }

private:
  void synchronizeJointState()
  {
    sensor_msgs::JointState::ConstPtr msg =
        ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states",
                                                            nh_);
    if (msg)
    {
      state_ = *msg;
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher setpoint_pub_;

  std::string controller_name_;
  std::vector<std::string> joint_names;
  sensor_msgs::JointState state_;
};


class CartesianJogger
{
public:
  CartesianJogger() = default;

  bool init(ros::NodeHandle& nh, const std::string& controller_name)
  {
    controller_name_ = controller_name;

    // Query pose
    std::string query_pose_service_name = controller_name_ + "/query_pose";
    query_pose_client_ = nh.serviceClient<taskspace_control_msgs::QueryPose>(
        query_pose_service_name);

    ROS_INFO_STREAM("Waiting for service: " << query_pose_service_name);
    query_pose_client_.waitForExistence();

    setpoint_pub_ =
        nh.advertise<taskspace_control_msgs::PoseTwistSetpoint>("setpoint", 1);

    if (!getPose(setpoint_.pose))
    {
      ROS_ERROR("Failed to get pose from controller.");
      return false;
    }

    return true;
  }

  inline const std::string& getControllerName() const
  {
    return controller_name_;
  }

private:
  bool getPose(geometry_msgs::Pose& pose)
  {
    taskspace_control_msgs::QueryPose srv;
    bool succeeded = query_pose_client_.call(srv);
    pose = srv.response.pose;
    return succeeded;
  }

  ros::Publisher setpoint_pub_;
  ros::ServiceClient query_pose_client_;

  std::string controller_name_;
  taskspace_control_msgs::PoseTwistSetpoint setpoint_;
};


class JoggingInterface
{
public:
  JoggingInterface() = default;

  bool init(ros::NodeHandle& nh)
  {
    // Controller manager
    ROS_INFO("Waiting for /controller_manager/switch_controller");
    switch_client_ =
        nh.serviceClient<controller_manager_msgs::SwitchController>(
            "controller_manager/switch_controller");
    switch_client_.waitForExistence();

    if (!joint_jogger_.init(nh, JOINT_CONTROLLER_NAME))
    {
      ROS_ERROR("Failed to initialize joint jogger");
      return false;
    }
    if (!cartesian_jogger_.init(nh, POSE_CONTROLLER_NAME))
    {
      ROS_ERROR("Failed to initialize cartesian jogger");
      return false;
    }
    return true;
  }

  void jogJoint()
  {
    startJointControl();
    joint_jogger_.setPosition(0, 1.0);
  }

private:
  void startJointControl()
  {
    if (type_ == JogType::JOINT)
      return;

    if (switchController(joint_jogger_.getControllerName()))
    {
      type_ = JogType::JOINT;
    }
    else
    {
      ROS_ERROR("Failed to start joint control");
    }
  }

  void startCartesianControl()
  {
    if (type_ == JogType::CARTESIAN)
      return;


    if (switchController(cartesian_jogger_.getControllerName()))
    {
      type_ = JogType::CARTESIAN;
    }
    else
    {
      ROS_ERROR("Failed to start Cartesian control");
    }
  }

  bool switchController(const std::string& controller)
  {
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = { controller };
    srv.request.stop_controllers = { current_controller_ };
    srv.request.strictness =
        controller_manager_msgs::SwitchController::Request::STRICT;

    if (!switch_client_.call(srv))
    {
      return false;
    }
    current_controller_ = controller;
    return true;
  }

  // controller manager client
  ros::ServiceClient switch_client_;

  JointJogger joint_jogger_;
  CartesianJogger cartesian_jogger_;

  // current state
  JogMode mode_ = JogMode::CONTINUOUS;
  JogType type_ = JogType::JOINT;
  std::string current_controller_;
};

// Configure terminal in raw mode
void enable_raw_mode(struct termios* original_termios)
{
  struct termios raw;
  tcgetattr(STDIN_FILENO, original_termios);

  raw = *original_termios;
  raw.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
  raw.c_cc[VEOL] = 1;               // New line
  raw.c_cc[VEOF] = 2;               // End of file

  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

void disable_raw_mode(struct termios* original_termios)
{
  tcsetattr(STDIN_FILENO, TCSAFLUSH, original_termios);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "console_jog");
  ros::NodeHandle nh;

  JoggingInterface jogger;
  if (!jogger.init(nh))
  {
    ROS_ERROR("Failed to initialize jogger");
    return 1;
  }

  struct termios original_termios;
  char c;

  enable_raw_mode(&original_termios);

  while (true)
  {
    if (read(STDIN_FILENO, &c, 1) < 0)
    {
      disable_raw_mode(&original_termios);
      exit(-1);
    }

    switch (c)
    {
      case KEYCODE_right:

      case KEYCODE_esc:
        std::cout << "Exiting" << std::endl;
        disable_raw_mode(&original_termios);
        exit(0);
      default:
        std::cout << c << std::endl;
    }
  }

  return 0;
}
