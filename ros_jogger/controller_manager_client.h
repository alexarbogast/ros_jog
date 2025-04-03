// Copyright 2025 Rowan Ramamurthy
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


#ifndef CONTROLLER_MANAGER_CLIENT_H
#define CONTROLLER_MANAGER_CLIENT_H

#include <ros/ros.h>
#include <controller_manager_msgs/SwitchController.h>

namespace ros_jogger {

class ControllerManagerClient {
public:
  ControllerManagerClient();
  
  bool switchController(const std::vector<std::string>& start_controllers,
                        const std::vector<std::string>& stop_controllers);

private:
  ros::ServiceClient switch_controller_client_;
};

} // namespace ros_jogger

#endif // CONTROLLER_MANAGER_CLIENT_H 