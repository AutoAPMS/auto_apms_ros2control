// Copyright 2026 Robin Müller
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

#include "auto_apms_behavior_tree_core/node.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"

#define INPUT_KEY_CONTROLLER_MANAGER "controller_manager"
#define OUTPUT_KEY_CONTROLLERS "controllers"
#define OUTPUT_KEY_ACTIVE "active_controllers"
#define OUTPUT_KEY_INACTIVE "inactive_controllers"

namespace auto_apms_ros2control
{

/**
 * @brief Queries the names and states of all controllers loaded by the controller manager.
 *
 * The targeted controller manager's service name is derived from the 'controller_manager' input port via the
 * NodeRegistrationOptions::topic pattern "(input:controller_manager)/list_controllers".
 */
class ListControllers
: public auto_apms_behavior_tree::core::RosServiceNode<controller_manager_msgs::srv::ListControllers>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::OutputPort<std::vector<std::string>>(OUTPUT_KEY_CONTROLLERS, "Names of all loaded controllers."),
      BT::OutputPort<std::vector<std::string>>(OUTPUT_KEY_ACTIVE, "Names of all active controllers."),
      BT::OutputPort<std::vector<std::string>>(OUTPUT_KEY_INACTIVE, "Names of all inactive controllers."),
    };
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    std::vector<std::string> all;
    std::vector<std::string> active;
    std::vector<std::string> inactive;
    for (const controller_manager_msgs::msg::ControllerState & controller : response->controller) {
      all.push_back(controller.name);
      if (controller.state == "active") active.push_back(controller.name);
      if (controller.state == "inactive") inactive.push_back(controller.name);
    }
    setOutput(OUTPUT_KEY_CONTROLLERS, all);
    setOutput(OUTPUT_KEY_ACTIVE, active);
    setOutput(OUTPUT_KEY_INACTIVE, inactive);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::ListControllers)
