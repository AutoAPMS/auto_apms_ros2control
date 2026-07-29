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
#define INPUT_KEY_CONTROLLER "controller"
#define INPUT_KEY_STATE "state"
#define OUTPUT_KEY_STATE "current_state"

namespace auto_apms_ros2control
{

/**
 * @brief Checks the current state of a single controller loaded by the controller manager.
 *
 * The controller manager offers no service for querying an individual controller, so this node calls the
 * 'list_controllers' service and looks up the controller named by the 'controller' input port. It returns FAILURE if
 * no controller with that name is loaded. If the 'state' input port is set, the node returns SUCCESS only when the
 * controller's current state matches it (FAILURE otherwise); if it is left empty, the node returns SUCCESS as long as
 * the controller exists. The controller's current state is always written to the 'current_state' output port.
 *
 * The targeted controller manager's service name is derived from the 'controller_manager' input port via the
 * NodeRegistrationOptions::topic pattern "(input:controller_manager)/list_controllers".
 */
class CheckControllerState
: public auto_apms_behavior_tree::core::RosServiceNode<controller_manager_msgs::srv::ListControllers>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::InputPort<std::string>(INPUT_KEY_CONTROLLER, "Name of the controller to check."),
      BT::InputPort<std::string>(
        INPUT_KEY_STATE, "",
        "Expected state of the controller (e.g. active, inactive, unconfigured). If empty, only the existence of the "
        "controller is checked."),
      BT::OutputPort<std::string>(OUTPUT_KEY_STATE, "Current state of the controller."),
    };
  }

  bool setRequest(Request::SharedPtr & /*request*/) override final
  {
    const BT::Expected<std::string> expected_controller = getInput<std::string>(INPUT_KEY_CONTROLLER);
    if (!expected_controller || expected_controller.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_controller ? "Controller name must not be empty." : expected_controller.error().c_str());
      return false;
    }
    controller_name_ = expected_controller.value();
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    for (const controller_manager_msgs::msg::ControllerState & controller : response->controller) {
      if (controller.name != controller_name_) continue;
      setOutput(OUTPUT_KEY_STATE, controller.state);

      const std::string expected_state = getInput<std::string>(INPUT_KEY_STATE).value_or("");
      if (!expected_state.empty() && controller.state != expected_state) {
        RCLCPP_DEBUG(
          logger_, "%s - Controller '%s' is in state '%s', expected '%s'.",
          context_.getFullyQualifiedTreeNodeName(this).c_str(), controller_name_.c_str(), controller.state.c_str(),
          expected_state.c_str());
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN(
      logger_, "%s - No controller named '%s' is loaded by the controller manager.",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), controller_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

private:
  std::string controller_name_;
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::CheckControllerState)
