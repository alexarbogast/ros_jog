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

//--------------------------------------------------------
// Joint Trajectory Controller
//--------------------------------------------------------

JointJog::JointJog()
  : rqt_gui_cpp::Plugin()
  , widget_(nullptr)
  , joint_parent_(nullptr)
  , joint_labels_()
  , nh_(ros::NodeHandle())
  , controller_client_()
  , current_controllers_()
{
  setObjectName("JointJog");

  // Create a timer to periodically update the controller manager list
  update_cc_list_timer_ =
      nh_.createTimer(ros::Duration(2.0), &JointJog::updateCCList, this);
}

//--------------------------------------------------------

void JointJog::updateJsVals()
{
  std::vector<double> vals = controller_client_.getJointPositions();

  if (vals.size() != joint_labels_.size())
  {
    rebuildWidget(vals.size());
  }

  for (size_t i = 0; i < vals.size(); i++)
  {
    if (i < joint_labels_.size())
    {
      joint_labels_[i]->setText(QString::number(vals[i], 'f', 6));
    }
  }
}

//--------------------------------------------------------

void JointJog::rebuildWidget(const int count)
{
  if (joint_parent_ == nullptr)
  {
    return;
  }

  // Remove extra joints
  QLayoutItem* child;
  while (joint_parent_->count() > count)
  {
    QLayoutItem* child = joint_parent_->takeAt(joint_parent_->count() - 1);
    delete child->widget();
    delete child;
    joint_labels_.pop_back();
  }

  // Create x number of spinner-like widgets
  for (int i = joint_labels_.size(); i < count; ++i)
  {
    ROS_INFO("Building spinner for joint %d", i);
    QHBoxLayout* spinner_layout = new QHBoxLayout();

    // Create left arrow button
    QPushButton* left_arrow = new QPushButton("<");
    left_arrow->setProperty("joint", i);
    left_arrow->setFixedSize(30, 30);
    spinner_layout->addWidget(left_arrow);

    // Create label to display the value
    QLabel* value_label = new QLabel("0.0");
    value_label->setAlignment(Qt::AlignCenter);
    value_label->setFixedSize(150, 30);
    spinner_layout->addWidget(value_label);

    // Create right arrow button
    QPushButton* right_arrow = new QPushButton(">");
    right_arrow->setProperty("joint", i);
    right_arrow->setFixedSize(30, 30);
    spinner_layout->addWidget(right_arrow);

    // Connect buttons to update the label value
    connect(left_arrow, &QPushButton::clicked, [value_label, i, this]() {
      double value = value_label->text().toDouble();
      value -= 0.1;  // Decrement value This is probably jank and need to fix
                     // with a step size
      this->controller_client_.publishJoints(value, i);
    });

    connect(right_arrow, &QPushButton::clicked, [value_label, i, this]() {
      double value = value_label->text().toDouble();
      value += 0.1;  // Increment value Same with this
      this->controller_client_.publishJoints(value, i);
    });

    // Add the spinner layout to the parent layout
    joint_parent_->addLayout(spinner_layout);
    joint_labels_.push_back(value_label);
  }
}

//--------------------------------------------------------

void JointJog::shutdownPlugin() {}

//--------------------------------------------------------

void JointJog::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const
{
  // No settings to save
}

//--------------------------------------------------------

void JointJog::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings)
{
  // No settings to restore
}

//--------------------------------------------------------

void JointJog::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();

  widget_->setWindowTitle("Pose Jogging Tool");
  widget_->setStyleSheet("background-color: #303030; color: white;");

  // Create main layout
  QVBoxLayout* main_layout = new QVBoxLayout(widget_);
  main_layout->setSpacing(10);

  // Add a dropdown to the widget
  controller_manager_dropdown_ = new QComboBox(widget_);
  controller_manager_dropdown_->setToolTip("Select Controller Manager");
  main_layout->addWidget(controller_manager_dropdown_);

  // Connect the dropdown's signal to the slot
  connect(controller_manager_dropdown_,
          QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &JointJog::onControllerManagerDropdownChanged);

  joint_parent_ = new QVBoxLayout(widget_);
  joint_parent_->setSpacing(10);
  main_layout->addLayout(joint_parent_);

  QTimer* ui_timer = new QTimer(widget_);
  connect(ui_timer, &QTimer::timeout, this, &JointJog::updateJsVals);
  ui_timer->start(100);

  widget_->setFocusPolicy(Qt::StrongFocus);
  context.addWidget(widget_);
}

//--------------------------------------------------------

void JointJog::populateControllerManagerDropdown(
    const std::vector<std::string>& controllers)
{
  controller_manager_dropdown_->clear();
  for (const auto& controller : controllers)
  {
    controller_manager_dropdown_->addItem(QString::fromStdString(controller));
  }
}

//--------------------------------------------------------

void JointJog::updateCCList(const ros::TimerEvent& event)
{
  std::vector<std::string> controllers = controller_client_.getControllers();

  // Check if the list has changed
  if (controllers != current_controllers_)
  {
    current_controllers_ = controllers;              // Update the current list
    populateControllerManagerDropdown(controllers);  // Update the dropdown
  }
}

//--------------------------------------------------------

void JointJog::onControllerManagerDropdownChanged(int index)
{
  QString selected_controller = controller_manager_dropdown_->itemText(index);
  ROS_INFO_STREAM("Selected controller: " << selected_controller.toStdString());

  // Perform actions based on the selected controller
  controller_client_.updateDevice(selected_controller.toStdString());
}

//--------------------------------------------------------

}  // namespace ros_jog

PLUGINLIB_EXPORT_CLASS(ros_jog::JointJog, rqt_gui_cpp::Plugin)