#include "ros_jog/ros_jog.h"
#include <pluginlib/class_list_macros.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QFrame>
#include <QTimer>
#include <QSlider>
#include <cmath>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>

namespace ros_jog
{

RosJog::RosJog()
  : rqt_gui_cpp::Plugin()
  , widget_(nullptr)
  , step_size_(0.1)  // Default step size
  , current_x_(0.0)
  , current_y_(0.0)
  , current_z_(0.0)
  , nh_(ros::NodeHandle()) // Initialize nh_
  , is_first_movement_(true)
  , controller_client_()
  , current_controllers_()
{
  setObjectName("RosJog");

  double update_freq = 10.0;  // Hz
  double update_interval = 1.0 / update_freq;

  // Create a timer to periodically update the controller manager list
  update_cc_list_timer_ = nh_.createTimer(ros::Duration(update_interval),
                                          &RosJog::updateCCList, this);

}

RosJog::~RosJog()
{
}

void RosJog::styleButton(QPushButton* button, const QString& style)
{
  button->setMinimumSize(40, 40);
  button->setStyleSheet("QPushButton {" + style +
                        "background-color: #404040;"
                        "border: none;"
                        "color: white;"
                        "padding: 8px;"
                        "border-radius: 5px;}"
                        "QPushButton:pressed {"
                        "background-color: #606060;}");
}

void RosJog::setupAxisControl(QPushButton* plusBtn, QPushButton* minusBtn,
                              const QString& plusText, const QString& minusText,
                              QWidget* container)
{
  QVBoxLayout* layout = new QVBoxLayout(container);
  layout->setSpacing(2);
  layout->setContentsMargins(2, 2, 2, 2);

  plusBtn->setText(plusText);
  minusBtn->setText(minusText);

  styleButton(plusBtn, "");
  styleButton(minusBtn, "");

  layout->addWidget(plusBtn);
  layout->addWidget(minusBtn);
}

double RosJog::sliderToStepSize(int sliderValue)
{
  // Convert slider value (0-1000) to step size (0.1mm - 100mm) logarithmically
  double t = static_cast<double>(sliderValue) / SLIDER_RESOLUTION;
  return MIN_STEP_SIZE * std::pow(MAX_STEP_SIZE / MIN_STEP_SIZE, t);
}

int RosJog::stepSizeToSlider(double stepSize)
{
  // Convert step size to slider value (inverse of sliderToStepSize)
  double t = std::log(stepSize / MIN_STEP_SIZE) /
             std::log(MAX_STEP_SIZE / MIN_STEP_SIZE);
  return static_cast<int>(t * SLIDER_RESOLUTION);
}

QString RosJog::formatStepSize(double stepSize)
{
  // Format step size in appropriate units (mm or cm)
  if (stepSize < 0.001)
  {
    return QString("%1 mm").arg(stepSize * 1000, 0, 'f', 2);
  }
  else if (stepSize < 0.01)
  {
    return QString("%1 mm").arg(stepSize * 1000, 0, 'f', 1);
  }
  else
  {
    return QString("%1 cm").arg(stepSize * 100, 0, 'f', 1);
  }
}

QWidget* RosJog::createStepSizeControl()
{
  QFrame* frame = new QFrame();
  frame->setFrameStyle(QFrame::Panel | QFrame::Sunken);

  QVBoxLayout* layout = new QVBoxLayout(frame);
  layout->setSpacing(4);
  layout->setContentsMargins(4, 4, 4, 4);

  // Create header with label
  QHBoxLayout* header_layout = new QHBoxLayout();
  QLabel* title_label = new QLabel("Step Size:");
  step_size_label_ = new QLabel(formatStepSize(step_size_));
  step_size_label_->setMinimumWidth(80);
  step_size_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  header_layout->addWidget(title_label);
  header_layout->addWidget(step_size_label_);

  // Create slider
  step_slider_ = new QSlider(Qt::Horizontal);
  step_slider_->setMinimum(0);
  step_slider_->setMaximum(SLIDER_RESOLUTION);
  step_slider_->setValue(stepSizeToSlider(step_size_));
  step_slider_->setStyleSheet(
      "QSlider::groove:horizontal {"
      "  background: #404040;"
      "  height: 6px;"
      "  border-radius: 3px;}"
      "QSlider::handle:horizontal {"
      "  background: #808080;"
      "  width: 18px;"
      "  margin: -6px 0;"
      "  border-radius: 9px;}");

  // Create preset buttons
  QHBoxLayout* button_layout = new QHBoxLayout();
  mm01_step_btn_ = new QPushButton("0.1mm");
  mm1_step_btn_ = new QPushButton("1mm");
  cm1_step_btn_ = new QPushButton("1cm");
  cm10_step_btn_ = new QPushButton("10cm");

  styleButton(mm01_step_btn_, "min-width: 50px;");
  styleButton(mm1_step_btn_, "min-width: 50px;");
  styleButton(cm1_step_btn_, "min-width: 50px;");
  styleButton(cm10_step_btn_, "min-width: 50px;");

  button_layout->addWidget(mm01_step_btn_);
  button_layout->addWidget(mm1_step_btn_);
  button_layout->addWidget(cm1_step_btn_);
  button_layout->addWidget(cm10_step_btn_);

  layout->addLayout(header_layout);
  layout->addWidget(step_slider_);
  layout->addLayout(button_layout);

  // Connect signals
  connect(step_slider_, &QSlider::valueChanged, this,
          &RosJog::onStepSliderChanged);
  connect(mm01_step_btn_, &QPushButton::clicked, this,
          &RosJog::on01mmStepClicked);
  connect(mm1_step_btn_, &QPushButton::clicked, this,
          &RosJog::on1mmStepClicked);
  connect(cm1_step_btn_, &QPushButton::clicked, this,
          &RosJog::on1cmStepClicked);
  connect(cm10_step_btn_, &QPushButton::clicked, this,
          &RosJog::on10cmStepClicked);

  return frame;
}

void RosJog::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  widget_->setWindowTitle("Move Tool");
  widget_->setStyleSheet("background-color: #303030; color: white;");

  // Create main layout
  QVBoxLayout* main_layout = new QVBoxLayout(widget_);
  main_layout->setSpacing(10);

  // Add a dropdown to the widget
  controller_manager_dropdown_ = new QComboBox(widget_);
  controller_manager_dropdown_->setToolTip("Select Controller Manager");
  main_layout->addWidget(controller_manager_dropdown_);

  // Connect the dropdown's signal to the slot
  connect(controller_manager_dropdown_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &RosJog::onControllerManagerDropdownChanged);

  // Add step size control at the top
  main_layout->addWidget(createStepSizeControl());

  // Create XY control pad
  QFrame* xy_pad = new QFrame();
  QGridLayout* xy_layout = new QGridLayout(xy_pad);
  xy_layout->setSpacing(2);

  x_plus_btn_ = new QPushButton("X+");
  x_minus_btn_ = new QPushButton("X-");
  y_plus_btn_ = new QPushButton("Y+");
  y_minus_btn_ = new QPushButton("Y-");

  styleButton(x_plus_btn_, "");
  styleButton(x_minus_btn_, "");
  styleButton(y_plus_btn_, "");
  styleButton(y_minus_btn_, "");

  xy_layout->addWidget(y_plus_btn_, 0, 1);
  xy_layout->addWidget(x_minus_btn_, 1, 0);
  xy_layout->addWidget(x_plus_btn_, 1, 2);
  xy_layout->addWidget(y_minus_btn_, 2, 1);

  // Create Z control
  QFrame* z_frame = new QFrame();
  z_plus_btn_ = new QPushButton("Z+");
  z_minus_btn_ = new QPushButton("Z-");
  setupAxisControl(z_plus_btn_, z_minus_btn_, "Z+", "Z-", z_frame);

  // Create bottom row for XY pad and Z control
  QHBoxLayout* bottom_row = new QHBoxLayout();
  bottom_row->addWidget(xy_pad);
  bottom_row->addWidget(z_frame);

  main_layout->addLayout(bottom_row);

  // Add Save Joint States button at the bottom
  save_joints_btn_ = new QPushButton("Save Joint States");
  styleButton(save_joints_btn_, "min-width: 120px;");
  main_layout->addWidget(save_joints_btn_);

  // Connect signals and slots using lambda functions
  connect(x_plus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(1.0, 0.0, 0.0); });
  connect(x_minus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(-1.0, 0.0, 0.0); });
  connect(y_plus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(0.0, 1.0, 0.0); });
  connect(y_minus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(0.0, -1.0, 0.0); });
  connect(z_plus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(0.0, 0.0, 1.0); });
  connect(z_minus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(0.0, 0.0, -1.0); });
  connect(save_joints_btn_, &QPushButton::clicked, this, &RosJog::handleSaveJointStates);
  
  // Install event filter for keyboard events
  widget_->installEventFilter(this);
  widget_->setFocusPolicy(Qt::StrongFocus);

  context.addWidget(widget_);
}

void RosJog::handleKeyRelease(int key) {
  switch (key) {
    case Qt::Key_Right:
      handleMovelClick(1.0, 0.0, 0.0);
      break;
    case Qt::Key_Left:
      handleMovelClick(-1.0, 0.0, 0.0);
      break;
    case Qt::Key_Up:
      handleMovelClick(0.0, 1.0, 0.0);
      break;
    case Qt::Key_Down:
      handleMovelClick(0.0, -1.0, 0.0);
      break;
    case Qt::Key_PageUp:
      handleMovelClick(0.0, 0.0, 1.0);
      break;
    case Qt::Key_PageDown:
      handleMovelClick(0.0, 0.0, -1.0);
      break;
  }
}

bool RosJog::eventFilter(QObject* obj, QEvent* event)
{
  if (event->type() == QEvent::KeyRelease) {
    QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
    if (!keyEvent->isAutoRepeat()) {  // Ignore auto-repeat key releases
      handleKeyRelease(keyEvent->key());
    }
    return true;
  }
  return QObject::eventFilter(obj, event);
}

void RosJog::shutdownPlugin()
{

}

void RosJog::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                          qt_gui_cpp::Settings& instance_settings) const
{
  // No settings to save
}

void RosJog::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                             const qt_gui_cpp::Settings& instance_settings)
{
  // No settings to restore
}

bool RosJog::getPose()
{
  if (!controller_client_.getPose(current_pose_))
  {
    ROS_ERROR("Failed to get current pose from controller");
    return false;
  }
  return true;
}

void RosJog::handleMovelClick(double x, double y, double z)
{
  // Get current pose if this is the first movement
  if (is_first_movement_)
  {
    if (!getPose())
    {
      ROS_ERROR("Failed to get initial pose");
      return;
    }
    target_pose_ = current_pose_;
    is_first_movement_ = false;
  }

  // Update target pose with the movement
  target_pose_.position.x += x * step_size_;
  target_pose_.position.y += y * step_size_;
  target_pose_.position.z += z * step_size_;

  // Convert target position to Eigen vector
  Eigen::Vector3d target_position(target_pose_.position.x,
                                  target_pose_.position.y,
                                  target_pose_.position.z);

  // Convert current orientation to Eigen quaternion
  Eigen::Quaterniond target_orientation =
      quaternionMsgToEigen(target_pose_.orientation);

  // Move to the target position with current orientation
  movel(target_position, target_orientation, 0.5);  // 1 second duration

  // Log the movement
  ROS_INFO_STREAM("Moving to position: (" << target_position.x() << ", " 
                  << target_position.y() << ", " << target_position.z() << ")");
}

// Step size control slots
void RosJog::onStepSliderChanged(int value)
{
  step_size_ = sliderToStepSize(value);
  step_size_label_->setText(formatStepSize(step_size_));
}

void RosJog::setStepSize(double value)
{
  step_slider_->setValue(stepSizeToSlider(value));
}

void RosJog::on01mmStepClicked()
{
  setStepSize(0.0001);  // 0.1 mm
}

void RosJog::on1mmStepClicked()
{
  setStepSize(0.001);  // 1 mm
}

void RosJog::on1cmStepClicked()
{
  setStepSize(0.01);  // 1 cm
}

void RosJog::on10cmStepClicked()
{
  setStepSize(0.1);  // 10 cm
}

// Motion planning functions implementation
void RosJog::movel(const Eigen::Vector3d& target_position,
                   const Eigen::Quaterniond& target_orientation,
                   double duration)
{
  geometry_msgs::Pose current_pose;
  if (!controller_client_.getPose(current_pose))
  {
    ROS_ERROR("Failed to retrieve current pose");
    return;
  }

  Eigen::Vector3d current_position(current_pose.position.x,
                                   current_pose.position.y,
                                   current_pose.position.z);
  Eigen::Quaterniond current_orientation =
      quaternionMsgToEigen(current_pose.orientation);

  executeLinearPath(current_position, target_position, current_orientation,
                    target_orientation, duration);
}

void RosJog::executeLinearPath(const Eigen::Vector3d& start_position,
                               const Eigen::Vector3d& end_position,
                               const Eigen::Quaterniond& start_orientation,
                               const Eigen::Quaterniond& end_orientation,
                               double duration)
{
  int num_points = static_cast<int>(hz_ * duration);
  std::vector<double> time_points(num_points);
  for (int i = 0; i < num_points; ++i)
  {
    time_points[i] = i * duration / (num_points - 1);
  }

  auto [position_func, velocity_func] =
      linearTrajectory(start_position, end_position, duration);
  auto [orientation_func, _] =
      slerpTrajectory(start_orientation, end_orientation, duration);

  std::vector<Eigen::Vector3d> positions;
  std::vector<Eigen::Vector3d> velocities;
  std::vector<Eigen::Quaterniond> orientations;

  for (double t : time_points)
  {
    positions.push_back(position_func(t));
    velocities.push_back(velocity_func(t));
    orientations.push_back(orientation_func(t));
  }

  executePath(positions, velocities, orientations);
}

void RosJog::executePath(const std::vector<Eigen::Vector3d>& positions,
                         const std::vector<Eigen::Vector3d>& velocities,
                         const std::vector<Eigen::Quaterniond>& orientations)
{
  ros::Rate rate(hz_);
  taskspace_control_msgs::PoseTwistSetpoint setpoint;

  for (size_t i = 0; i < positions.size(); ++i)
  {
    setpoint.pose.position.x = positions[i].x();
    setpoint.pose.position.y = positions[i].y();
    setpoint.pose.position.z = positions[i].z();

    setpoint.pose.orientation = quaternionEigenToMsg(orientations[i]);

    setpoint.twist.linear.x = velocities[i].x();
    setpoint.twist.linear.y = velocities[i].y();
    setpoint.twist.linear.z = velocities[i].z();

    controller_client_.publishSetpoint(setpoint);
    rate.sleep();
  }
}

// Helper functions implementation
Eigen::Quaterniond
RosJog::quaternionMsgToEigen(const geometry_msgs::Quaternion& msg)
{
  return Eigen::Quaterniond(msg.w, msg.x, msg.y, msg.z);
}

geometry_msgs::Quaternion
RosJog::quaternionEigenToMsg(const Eigen::Quaterniond& q)
{
  geometry_msgs::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

std::pair<std::function<Eigen::Vector3d(double)>,
          std::function<Eigen::Vector3d(double)>>
RosJog::linearTrajectory(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end, double duration)
{
  Eigen::Vector3d delta = end - start;

  auto position_func = [start, delta, duration](double t) {
    double s = t / duration;
    return start + s * delta;
  };

  auto velocity_func = [delta, duration](double t) { return delta / duration; };

  return { position_func, velocity_func };
}

std::pair<std::function<Eigen::Quaterniond(double)>,
          std::function<Eigen::Vector3d(double)>>
RosJog::slerpTrajectory(const Eigen::Quaterniond& start,
                        const Eigen::Quaterniond& end, double duration)
{
  auto orientation_func = [start, end, duration](double t) {
    double s = t / duration;
    return start.slerp(s, end);
  };

  // For simplicity, we return zero angular velocity
  auto velocity_func = [](double t) { return Eigen::Vector3d::Zero(); };

  return { orientation_func, velocity_func };
}

void RosJog::populateControllerManagerDropdown(const std::vector<std::string>& controllers) {
    controller_manager_dropdown_->clear();
    for (const auto& controller : controllers) {
        controller_manager_dropdown_->addItem(QString::fromStdString(controller));
    }
}

void RosJog::updateCCList(const ros::TimerEvent& event) {
    std::vector<std::string> controllers = controller_client_.getPotentialConrollers();

    // Check if the list has changed
    if (controllers != current_controllers_) {
        current_controllers_ = controllers; // Update the current list
        populateControllerManagerDropdown(controllers); // Update the dropdown
    }
}

void RosJog::onControllerManagerDropdownChanged(int index) {
    QString selected_controller = controller_manager_dropdown_->itemText(index);
    ROS_INFO_STREAM("Selected controller: " << selected_controller.toStdString());

    // Perform actions based on the selected controller
    controller_client_.updateDevice(selected_controller.toStdString());
    is_first_movement_ = true; // Reset the first movement flag
}

void RosJog::handleSaveJointStates() 
{
    std::vector<double> joint_positions = controller_client_.getJointPositions();
    
// Get home directory path
    const char* home_dir = getenv("HOME");
    if (!home_dir) {
        ROS_ERROR("Could not get home directory path");
        return;
    }
    
    std::string filepath = std::string(home_dir) + "/workspaces/joint_states.csv";
    std::ofstream file;
    file.open(filepath, std::ios::app); // Open in append mode
    
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file for writing: %s", filepath.c_str());
        return;
    }
    
    // Write joint positions as CSV with 6 decimal places
    file << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        file << joint_positions[i];
        if (i < joint_positions.size() - 1) {
            file << ",";
        }
    }
    file << "\n";
    file.close();
    
    ROS_INFO("Joint states saved to: %s", filepath.c_str());
}

}  // namespace ros_jog

PLUGINLIB_EXPORT_CLASS(ros_jog::RosJog, rqt_gui_cpp::Plugin)
