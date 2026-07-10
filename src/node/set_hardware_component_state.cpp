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
#include "controller_manager_msgs/srv/set_hardware_component_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#define INPUT_KEY_CONTROLLER_MANAGER "controller_manager"
#define INPUT_KEY_COMPONENT "component"
#define INPUT_KEY_TARGET_STATE "target_state"
#define OUTPUT_KEY_STATE "state"

namespace auto_apms_ros2control
{

/**
 * @brief Sets the lifecycle state of a hardware component loaded by the controller manager.
 *
 * The targeted controller manager's service name is derived from the 'controller_manager' input port via the
 * NodeRegistrationOptions::topic pattern "(input:controller_manager)/set_hardware_component_state".
 */
class SetHardwareComponentState
: public auto_apms_behavior_tree::core::RosServiceNode<controller_manager_msgs::srv::SetHardwareComponentState>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::InputPort<std::string>(INPUT_KEY_COMPONENT, "Name of the targeted hardware component."),
      BT::InputPort<std::string>(
        INPUT_KEY_TARGET_STATE, "Target lifecycle state. One of: unconfigured, inactive, active, finalized."),
      BT::OutputPort<std::string>(OUTPUT_KEY_STATE, "Label of the hardware component's resulting lifecycle state."),
    };
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    const BT::Expected<std::string> expected_component = getInput<std::string>(INPUT_KEY_COMPONENT);
    if (!expected_component || expected_component.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_component ? "Hardware component name must not be empty." : expected_component.error().c_str());
      return false;
    }
    request->name = expected_component.value();

    const BT::Expected<std::string> expected_state = getInput<std::string>(INPUT_KEY_TARGET_STATE);
    if (!expected_state) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected_state.error().c_str());
      return false;
    }
    const std::string & label = expected_state.value();
    if (label == "unconfigured") {
      request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
    } else if (label == "inactive") {
      request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    } else if (label == "active") {
      request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    } else if (label == "finalized") {
      request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
    } else {
      RCLCPP_ERROR(
        logger_, "%s - Invalid target state '%s'. Must be one of: unconfigured, inactive, active, finalized.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), label.c_str());
      return false;
    }
    request->target_state.label = label;
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    setOutput(OUTPUT_KEY_STATE, response->state.label);
    if (!response->ok) {
      RCLCPP_ERROR(
        logger_, "%s - Failed to set hardware component state via service '%s'. Current state: '%s'.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), getServiceName().c_str(),
        response->state.label.c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::SetHardwareComponentState)
