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
#include "controller_manager_msgs/srv/reload_controller_libraries.hpp"

#define INPUT_KEY_CONTROLLER_MANAGER "controller_manager"
#define INPUT_KEY_FORCE_KILL "force_kill"

namespace auto_apms_ros2control
{

/**
 * @brief Reloads all available controller libraries of the controller manager.
 *
 * This is only possible when no controllers are loaded, unless 'force_kill' is set to true, in which case all
 * loaded controllers are unloaded first.
 *
 * The targeted controller manager's service name is derived from the 'controller_manager' input port via the
 * NodeRegistrationOptions::topic pattern "(input:controller_manager)/reload_controller_libraries".
 */
class ReloadControllerLibraries
: public auto_apms_behavior_tree::core::RosServiceNode<controller_manager_msgs::srv::ReloadControllerLibraries>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::InputPort<bool>(
        INPUT_KEY_FORCE_KILL, false, "Unload all loaded controllers before reloading the libraries."),
    };
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    const BT::Expected<bool> expected_force_kill = getInput<bool>(INPUT_KEY_FORCE_KILL);
    if (!expected_force_kill) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_force_kill.error().c_str());
      return false;
    }
    request->force_kill = expected_force_kill.value();
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    if (!response->ok) {
      RCLCPP_ERROR(
        logger_, "%s - Failed to reload controller libraries via service '%s'.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), getServiceName().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::ReloadControllerLibraries)
