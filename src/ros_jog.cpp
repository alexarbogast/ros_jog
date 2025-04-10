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

namespace ros_jog
{

RosJog::RosJog()
  : rqt_gui_cpp::Plugin()
  , widget_(nullptr)
  , step_size_(0.1)  // Default step size
  , current_x_(0.0)
  , current_y_(0.0)
  , current_z_(0.0)
  , is_first_movement_(true)
  , controller_name_("pose_controller")
  , joint_controller_name_("joint_position_controller")
  , controller_client_(nullptr)
  , joint_controller_client_(nullptr)
  , controller_manager_client_(nullptr)
{
  setObjectName("RosJog");

  // ros::NodeHandle private_nh("~");  // "~" means private namespace
  // if (!private_nh.getParam("pose_controller", controller_name_)) {
  //     ROS_ERROR("Failed to get 'controller' parameter");
  //     return;
  // }

  // if (!private_nh.getParam("joint_controller", joint_controller_name_)) {
  //     ROS_ERROR("Failed to get 'joint_controller' parameter");
  //     return;
  // }

  // Initialize controller clients
  controller_client_ = new ControllerClient(controller_name_);
  joint_controller_client_ = new JointControllerClient(joint_controller_name_);
  controller_manager_client_ = new ControllerManagerClient();

  // Initialize key states
  for (int i = 0; i < 6; ++i)
  {
    keys_pressed_[i] = false;
  }
}

RosJog::~RosJog()
{
  // Clean up controller clients
  delete controller_client_;
  delete joint_controller_client_;
  delete controller_manager_client_;
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

  // Add step size control at the top
  main_layout->addWidget(createStepSizeControl());

  // Create top row with ABC controls
  QHBoxLayout* top_row = new QHBoxLayout();

  // Create ABC buttons
  QFrame* a_frame = new QFrame();
  QFrame* b_frame = new QFrame();
  QFrame* c_frame = new QFrame();

  a_plus_btn_ = new QPushButton();
  a_minus_btn_ = new QPushButton();
  b_plus_btn_ = new QPushButton();
  b_minus_btn_ = new QPushButton();
  c_plus_btn_ = new QPushButton();
  c_minus_btn_ = new QPushButton();

  setupAxisControl(a_plus_btn_, a_minus_btn_, "A+", "A-", a_frame);
  setupAxisControl(b_plus_btn_, b_minus_btn_, "B+", "B-", b_frame);
  setupAxisControl(c_plus_btn_, c_minus_btn_, "C+", "C-", c_frame);

  top_row->addWidget(a_frame);
  top_row->addWidget(b_frame);
  top_row->addWidget(c_frame);

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

  main_layout->addLayout(top_row);
  main_layout->addLayout(bottom_row);

  // Connect signals and slots using lambda functions
  connect(x_plus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(1.0, 0.0, 0.0); });
  connect(x_minus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(-1.0, 0.0, 0.0); });
  connect(y_plus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(0.0, 1.0, 0.0); });
  connect(y_minus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(0.0, -1.0, 0.0); });
  connect(z_plus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(0.0, 0.0, 1.0); });
  connect(z_minus_btn_, &QPushButton::clicked, [this]() { handleMovelClick(0.0, 0.0, -1.0); });
  
  connect(a_plus_btn_, &QPushButton::clicked, [this]() { handleRotationClick(1.0, 0.0, 0.0); });
  connect(a_minus_btn_, &QPushButton::clicked, [this]() { handleRotationClick(-1.0, 0.0, 0.0); });
  connect(b_plus_btn_, &QPushButton::clicked, [this]() { handleRotationClick(0.0, 1.0, 0.0); });
  connect(b_minus_btn_, &QPushButton::clicked, [this]() { handleRotationClick(0.0, -1.0, 0.0); });
  connect(c_plus_btn_, &QPushButton::clicked, [this]() { handleRotationClick(0.0, 0.0, 1.0); });
  connect(c_minus_btn_, &QPushButton::clicked, [this]() { handleRotationClick(0.0, 0.0, -1.0); });

  // Swap to the taskspace pose controller
  controller_manager_client_->switchController({controller_name_}, {joint_controller_name_});

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
  controller_manager_client_->switchController({ joint_controller_name_ },
                                               { controller_name_ });
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
  if (!controller_client_->getPose(current_pose_))
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


void RosJog::handleRotationClick(double roll, double pitch, double yaw)
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
  return;
  // Convert current orientation to Eigen quaternion
  Eigen::Quaterniond current_orientation = quaternionMsgToEigen(target_pose_.orientation);

  double step_size = 0.17;  // Adjust step size for rotation

  // Create incremental rotations for roll, pitch, and yaw
  Eigen::AngleAxisd roll_rotation(roll * step_size, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_rotation(pitch * step_size, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_rotation(yaw * step_size, Eigen::Vector3d::UnitZ());

  // Apply the rotations to the current orientation
  Eigen::Quaterniond new_orientation = yaw_rotation * pitch_rotation * roll_rotation * current_orientation;

  // Update the target pose orientation
  target_pose_.orientation = quaternionEigenToMsg(new_orientation);

  // Move to the target position with the updated orientation
  movel(Eigen::Vector3d(target_pose_.position.x, target_pose_.position.y, target_pose_.position.z),
        new_orientation, 0.5);  // 1 second duration

  // Log the movement
  ROS_INFO_STREAM("Adjusting orientation: roll=" << roll * step_size_ << ", pitch=" << pitch * step_size_
                  << ", yaw=" << yaw * step_size_);
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
  if (!controller_client_->getPose(current_pose))
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

    controller_client_->publishSetpoint(setpoint);
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

}  // namespace ros_jog

PLUGINLIB_EXPORT_CLASS(ros_jog::RosJog, rqt_gui_cpp::Plugin)
