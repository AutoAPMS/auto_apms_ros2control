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
#include "controller_manager_msgs/srv/list_hardware_components.hpp"

#define INPUT_KEY_CONTROLLER_MANAGER "controller_manager"
#define INPUT_KEY_HARDWARE "hardware"
#define INPUT_KEY_STATE "state"
#define OUTPUT_KEY_STATE "current_state"

namespace auto_apms_ros2control
{

/**
 * @brief Checks the current lifecycle state of a single hardware component loaded by the controller manager.
 *
 * The controller manager offers no service for querying an individual hardware component, so this node calls the
 * 'list_hardware_components' service and looks up the component named by the 'hardware' input port. It returns FAILURE
 * if no component with that name is loaded. If the 'state' input port is set, the node returns SUCCESS only when the
 * component's current lifecycle state matches it (FAILURE otherwise); if it is left empty, the node returns SUCCESS as
 * long as the component exists. The component's current state label is always written to the 'current_state' output
 * port.
 *
 * The targeted controller manager's service name is derived from the 'controller_manager' input port via the
 * NodeRegistrationOptions::topic pattern "(input:controller_manager)/list_hardware_components".
 */
class CheckHardwareState
: public auto_apms_behavior_tree::core::RosServiceNode<controller_manager_msgs::srv::ListHardwareComponents>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::InputPort<std::string>(INPUT_KEY_HARDWARE, "Name of the hardware component to check."),
      BT::InputPort<std::string>(
        INPUT_KEY_STATE, "",
        "Expected lifecycle state of the hardware component (e.g. active, inactive, unconfigured, finalized). If empty, "
        "only the existence of the component is checked."),
      BT::OutputPort<std::string>(OUTPUT_KEY_STATE, "Current lifecycle state label of the hardware component."),
    };
  }

  bool setRequest(Request::SharedPtr & /*request*/) override final
  {
    const BT::Expected<std::string> expected_hardware = getInput<std::string>(INPUT_KEY_HARDWARE);
    if (!expected_hardware || expected_hardware.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_hardware ? "Hardware component name must not be empty." : expected_hardware.error().c_str());
      return false;
    }
    hardware_name_ = expected_hardware.value();
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    for (const controller_manager_msgs::msg::HardwareComponentState & component : response->component) {
      if (component.name != hardware_name_) continue;
      setOutput(OUTPUT_KEY_STATE, component.state.label);

      const std::string expected_state = getInput<std::string>(INPUT_KEY_STATE).value_or("");
      if (!expected_state.empty() && component.state.label != expected_state) {
        RCLCPP_DEBUG(
          logger_, "%s - Hardware component '%s' is in state '%s', expected '%s'.",
          context_.getFullyQualifiedTreeNodeName(this).c_str(), hardware_name_.c_str(),
          component.state.label.c_str(), expected_state.c_str());
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(
      logger_, "%s - No hardware component named '%s' is loaded by the controller manager.",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), hardware_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string hardware_name_;
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::CheckHardwareState)
