#include "ros_jogger/controller_manager_client.h"
#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>

namespace ros_jogger {

ControllerManagerClient::ControllerManagerClient() {
  ros::NodeHandle nh;
  switch_controller_client_ = nh.serviceClient<controller_manager_msgs::SwitchController>(
    "/controller_manager/switch_controller");
  
  // Wait for the service to be available
  if (!switch_controller_client_.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR("Failed to connect to controller_manager/switch_controller service");
  }
}

bool ControllerManagerClient::switchController(const std::vector<std::string>& start_controllers,
                                            const std::vector<std::string>& stop_controllers) {
  controller_manager_msgs::SwitchController srv;
  srv.request.start_controllers = start_controllers;
  srv.request.stop_controllers = stop_controllers;
  srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;
  
  if (!switch_controller_client_.call(srv)) {
    ROS_ERROR("Failed to call switch_controller service");
    return false;
  }
  
  if (!srv.response.ok) {
    ROS_ERROR("Controller switch failed");
    return false;
  }
  
  return true;
}

} // namespace ros_jogger 