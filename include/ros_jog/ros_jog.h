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

#ifndef ROS_JOG_H
#define ROS_JOG_H

#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include <QWidget>
#include <QPushButton>
#include <QKeyEvent>
#include <QSlider>
#include <QLabel>
#include <QTimer>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <taskspace_control_msgs/PoseTwistSetpoint.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "ros_jog/controller_clients.h"  
#include "ros_jog/controller_manager_client.h"

using namespace std;

namespace ros_jog {

class RosJog : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  RosJog();
  ~RosJog();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected:
  bool eventFilter(QObject* obj, QEvent* event);

private slots:
  // Step size controls
  void onStepSliderChanged(int value);
  void setStepSize(double value);
  void on01mmStepClicked();
  void on1mmStepClicked();
  void on1cmStepClicked();
  void on10cmStepClicked();

private:
  // Motion planning functions
  void movel(const Eigen::Vector3d& target_position, 
             const Eigen::Quaterniond& target_orientation,
             double duration);
  void executeLinearPath(const Eigen::Vector3d& start_position,
                        const Eigen::Vector3d& end_position,
                        const Eigen::Quaterniond& start_orientation,
                        const Eigen::Quaterniond& end_orientation,
                        double duration);
  void executePath(const std::vector<Eigen::Vector3d>& positions,
                  const std::vector<Eigen::Vector3d>& velocities,
                  const std::vector<Eigen::Quaterniond>& orientations);
  
  // Helper functions
  Eigen::Quaterniond quaternionMsgToEigen(const geometry_msgs::Quaternion& msg);
  geometry_msgs::Quaternion quaternionEigenToMsg(const Eigen::Quaterniond& q);

  std::pair<std::function<Eigen::Vector3d(double)>, 
            std::function<Eigen::Vector3d(double)>> 
  linearTrajectory(const Eigen::Vector3d& start, 
                  const Eigen::Vector3d& end, 
                  double duration);
  std::pair<std::function<Eigen::Quaterniond(double)>, 
            std::function<Eigen::Vector3d(double)>> 
  slerpTrajectory(const Eigen::Quaterniond& start, 
                 const Eigen::Quaterniond& end, 
                 double duration);

  bool getPose();
  void handleButtonClick(double x, double y, double z);
  void setupAxisControl(QPushButton* plusBtn, QPushButton* minusBtn, 
                       const QString& plusText, const QString& minusText,
                       QWidget* container);
  void styleButton(QPushButton* button, const QString& style);
  QWidget* createStepSizeControl();
  void handleKeyPress(int key);
  void handleKeyRelease(int key);
  void startContinuousMove();
  void stopContinuousMove();
  double sliderToStepSize(int sliderValue);
  int stepSizeToSlider(double stepSize);
  QString formatStepSize(double stepSize);
  
  QWidget* widget_;
  // XYZ buttons
  QPushButton* x_plus_btn_;
  QPushButton* x_minus_btn_;
  QPushButton* y_plus_btn_;
  QPushButton* y_minus_btn_;
  QPushButton* z_plus_btn_;
  QPushButton* z_minus_btn_;
  // ABC buttons
  QPushButton* a_plus_btn_;
  QPushButton* a_minus_btn_;
  QPushButton* b_plus_btn_;
  QPushButton* b_minus_btn_;
  QPushButton* c_plus_btn_;
  QPushButton* c_minus_btn_;
  
  // Step size controls
  QSlider* step_slider_;
  QLabel* step_size_label_;
  QPushButton* mm01_step_btn_;
  QPushButton* mm1_step_btn_;
  QPushButton* cm1_step_btn_;
  QPushButton* cm10_step_btn_;
  double step_size_;
  
  // Movement control
  bool keys_pressed_[6];  // Track state of arrow keys and page up/down
  double current_x_;
  double current_y_;
  double current_z_;
  
  ros::Publisher point_pub_;
  ros::NodeHandle nh_;
  std::string controller_name_;
  std::string joint_controller_name_; 
  ControllerClient* controller_client_;
  JointControllerClient* joint_controller_client_;
  ControllerManagerClient* controller_manager_client_;

  // Motion planning parameters
  double hz_;  // Control frequency
  geometry_msgs::Pose current_pose_;
  geometry_msgs::Pose target_pose_;  // Target pose for incremental movements
  bool is_first_movement_;  // Flag to track if this is the first movement

  // Constants for logarithmic slider
  static constexpr double MIN_STEP_SIZE = 0.0001;  // 0.1mm
  static constexpr double MAX_STEP_SIZE = 0.1;     // 100mm
  static constexpr int SLIDER_RESOLUTION = 1000;    // Number of slider steps
};

} // namespace ros_jog

#endif // ROS_JOG_H 
