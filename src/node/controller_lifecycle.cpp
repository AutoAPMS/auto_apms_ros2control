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
#include "controller_manager_msgs/srv/cleanup_controller.hpp"
#include "controller_manager_msgs/srv/configure_controller.hpp"
#include "controller_manager_msgs/srv/load_controller.hpp"
#include "controller_manager_msgs/srv/unload_controller.hpp"

#define INPUT_KEY_CONTROLLER_MANAGER "controller_manager"
#define INPUT_KEY_CONTROLLER "controller"

namespace auto_apms_ros2control
{

/**
 * @brief Template for controller manager services that operate on a single controller by name.
 *
 * Covers the LoadController, ConfigureController, CleanupController and UnloadController services, which all share
 * the same request (a controller name) and response (a success flag) layout.
 *
 * The targeted controller manager's service name is derived from the 'controller_manager' input port via the
 * NodeRegistrationOptions::topic pattern "(input:controller_manager)/<service>".
 *
 * @tparam ServiceT One of the single-controller controller_manager_msgs services.
 */
template <class ServiceT>
class ControllerRequestTemplate : public auto_apms_behavior_tree::core::RosServiceNode<ServiceT>
{
public:
  using auto_apms_behavior_tree::core::RosServiceNode<ServiceT>::RosServiceNode;
  using typename auto_apms_behavior_tree::core::RosServiceNode<ServiceT>::Request;
  using typename auto_apms_behavior_tree::core::RosServiceNode<ServiceT>::Response;

  static BT::PortsList providedPorts()
  {
    // We do not use the default 'topic' port — the service name is derived from the 'controller_manager' port
    // via the NodeRegistrationOptions::topic pattern "(input:controller_manager)/<service>".
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::InputPort<std::string>(INPUT_KEY_CONTROLLER, "Name of the targeted controller."),
    };
  }

  bool setRequest(typename Request::SharedPtr & request) override final
  {
    const BT::Expected<std::string> expected_controller = this->template getInput<std::string>(INPUT_KEY_CONTROLLER);
    if (!expected_controller || expected_controller.value().empty()) {
      RCLCPP_ERROR(
        this->logger_, "%s - %s", this->context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_controller ? "Controller name must not be empty." : expected_controller.error().c_str());
      return false;
    }
    request->name = expected_controller.value();
    return true;
  }

  BT::NodeStatus onResponseReceived(const typename Response::SharedPtr & response) override final
  {
    if (!response->ok) {
      RCLCPP_ERROR(
        this->logger_, "%s - Request failed for service '%s'.",
        this->context_.getFullyQualifiedTreeNodeName(this).c_str(), this->getServiceName().c_str());
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

/// @brief Loads a controller into the controller manager.
class LoadController : public ControllerRequestTemplate<controller_manager_msgs::srv::LoadController>
{
public:
  using ControllerRequestTemplate::ControllerRequestTemplate;
};

/// @brief Configures a loaded controller (transition to the 'inactive' state).
class ConfigureController : public ControllerRequestTemplate<controller_manager_msgs::srv::ConfigureController>
{
public:
  using ControllerRequestTemplate::ControllerRequestTemplate;
};

/// @brief Cleans up an inactive controller (transition back to the 'unconfigured' state).
class CleanupController : public ControllerRequestTemplate<controller_manager_msgs::srv::CleanupController>
{
public:
  using ControllerRequestTemplate::ControllerRequestTemplate;
};

/// @brief Unloads a controller from the controller manager.
class UnloadController : public ControllerRequestTemplate<controller_manager_msgs::srv::UnloadController>
{
public:
  using ControllerRequestTemplate::ControllerRequestTemplate;
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::LoadController)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::ConfigureController)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::CleanupController)
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::UnloadController)
