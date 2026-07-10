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
#include "controller_manager_msgs/srv/switch_controller.hpp"

#define INPUT_KEY_CONTROLLER_MANAGER "controller_manager"
#define INPUT_KEY_ACTIVATE "activate_controllers"
#define INPUT_KEY_DEACTIVATE "deactivate_controllers"
#define INPUT_KEY_STRICTNESS "strictness"
#define INPUT_KEY_ACTIVATE_ASAP "activate_asap"
#define INPUT_KEY_TIMEOUT "timeout"
#define OUTPUT_KEY_MESSAGE "message"

namespace auto_apms_ros2control
{

/**
 * @brief Activates and/or deactivates one or more ros2_control controllers within a single update cycle of the
 * controller manager.
 *
 * The targeted controller manager's service name is derived from the 'controller_manager' input port via the
 * NodeRegistrationOptions::topic pattern "(input:controller_manager)/switch_controller". Multiple controller names
 * are given as a ';'-separated list.
 */
class SwitchController
: public auto_apms_behavior_tree::core::RosServiceNode<controller_manager_msgs::srv::SwitchController>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    // We do not use the default 'topic' port — the service name is derived from the 'controller_manager' port
    // via the NodeRegistrationOptions::topic pattern "(input:controller_manager)/switch_controller".
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::InputPort<std::vector<std::string>>(
        INPUT_KEY_ACTIVATE, {}, "';'-separated names of the controllers to activate."),
      BT::InputPort<std::vector<std::string>>(
        INPUT_KEY_DEACTIVATE, {}, "';'-separated names of the controllers to deactivate."),
      BT::InputPort<std::string>(
        INPUT_KEY_STRICTNESS, "STRICT",
        "Switching behavior on error. One of: BEST_EFFORT, STRICT, AUTO, FORCE_AUTO."),
      BT::InputPort<bool>(
        INPUT_KEY_ACTIVATE_ASAP, false,
        "Activate the controllers as soon as their hardware dependencies are ready instead of waiting for all "
        "interfaces to become available."),
      BT::InputPort<double>(INPUT_KEY_TIMEOUT, 0.0, "Timeout [s] before aborting pending controllers (0 = infinite)."),
      BT::OutputPort<std::string>(OUTPUT_KEY_MESSAGE, "Service response: informational message."),
    };
  }

  bool setRequest(Request::SharedPtr & request) override final
  {
    const BT::Expected<std::vector<std::string>> expected_activate =
      getInput<std::vector<std::string>>(INPUT_KEY_ACTIVATE);
    const BT::Expected<std::vector<std::string>> expected_deactivate =
      getInput<std::vector<std::string>>(INPUT_KEY_DEACTIVATE);
    if (!expected_activate || !expected_deactivate) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        (!expected_activate ? expected_activate.error() : expected_deactivate.error()).c_str());
      return false;
    }
    request->activate_controllers = expected_activate.value();
    request->deactivate_controllers = expected_deactivate.value();

    const BT::Expected<std::string> expected_strictness = getInput<std::string>(INPUT_KEY_STRICTNESS);
    if (!expected_strictness) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_strictness.error().c_str());
      return false;
    }
    const std::string & strictness = expected_strictness.value();
    if (strictness == "BEST_EFFORT") {
      request->strictness = Request::BEST_EFFORT;
    } else if (strictness == "STRICT") {
      request->strictness = Request::STRICT;
    } else if (strictness == "AUTO") {
      request->strictness = Request::AUTO;
    } else if (strictness == "FORCE_AUTO") {
      request->strictness = Request::FORCE_AUTO;
    } else {
      RCLCPP_ERROR(
        logger_, "%s - Invalid strictness value '%s'. Must be one of: BEST_EFFORT, STRICT, AUTO, FORCE_AUTO.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), strictness.c_str());
      return false;
    }

    const BT::Expected<bool> expected_activate_asap = getInput<bool>(INPUT_KEY_ACTIVATE_ASAP);
    if (!expected_activate_asap) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_activate_asap.error().c_str());
      return false;
    }
    request->activate_asap = expected_activate_asap.value();

    const BT::Expected<double> expected_timeout = getInput<double>(INPUT_KEY_TIMEOUT);
    if (!expected_timeout) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(), expected_timeout.error().c_str());
      return false;
    }
    request->timeout = rclcpp::Duration::from_seconds(expected_timeout.value());

    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    setOutput(OUTPUT_KEY_MESSAGE, response->message);
    return response->ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::SwitchController)
