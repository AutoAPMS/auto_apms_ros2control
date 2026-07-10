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
#include "controller_manager_msgs/srv/list_hardware_interfaces.hpp"

#define INPUT_KEY_CONTROLLER_MANAGER "controller_manager"
#define OUTPUT_KEY_COMMAND_INTERFACES "command_interfaces"
#define OUTPUT_KEY_STATE_INTERFACES "state_interfaces"
#define OUTPUT_KEY_AVAILABLE_COMMAND_INTERFACES "available_command_interfaces"
#define OUTPUT_KEY_CLAIMED_COMMAND_INTERFACES "claimed_command_interfaces"

namespace auto_apms_ros2control
{

/**
 * @brief Queries the names of all command and state interfaces registered with the controller manager.
 *
 * The targeted controller manager's service name is derived from the 'controller_manager' input port via the
 * NodeRegistrationOptions::topic pattern "(input:controller_manager)/list_hardware_interfaces".
 */
class ListHardwareInterfaces
: public auto_apms_behavior_tree::core::RosServiceNode<controller_manager_msgs::srv::ListHardwareInterfaces>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::OutputPort<std::vector<std::string>>(OUTPUT_KEY_COMMAND_INTERFACES, "Names of all command interfaces."),
      BT::OutputPort<std::vector<std::string>>(OUTPUT_KEY_STATE_INTERFACES, "Names of all state interfaces."),
      BT::OutputPort<std::vector<std::string>>(
        OUTPUT_KEY_AVAILABLE_COMMAND_INTERFACES, "Names of all currently available command interfaces."),
      BT::OutputPort<std::vector<std::string>>(
        OUTPUT_KEY_CLAIMED_COMMAND_INTERFACES, "Names of all currently claimed command interfaces."),
    };
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    std::vector<std::string> command_interfaces;
    std::vector<std::string> available;
    std::vector<std::string> claimed;
    for (const controller_manager_msgs::msg::HardwareInterface & interface : response->command_interfaces) {
      command_interfaces.push_back(interface.name);
      if (interface.is_available) available.push_back(interface.name);
      if (interface.is_claimed) claimed.push_back(interface.name);
    }
    std::vector<std::string> state_interfaces;
    for (const controller_manager_msgs::msg::HardwareInterface & interface : response->state_interfaces) {
      state_interfaces.push_back(interface.name);
    }
    setOutput(OUTPUT_KEY_COMMAND_INTERFACES, command_interfaces);
    setOutput(OUTPUT_KEY_STATE_INTERFACES, state_interfaces);
    setOutput(OUTPUT_KEY_AVAILABLE_COMMAND_INTERFACES, available);
    setOutput(OUTPUT_KEY_CLAIMED_COMMAND_INTERFACES, claimed);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::ListHardwareInterfaces)
