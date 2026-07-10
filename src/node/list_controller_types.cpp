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
#include "controller_manager_msgs/srv/list_controller_types.hpp"

#define INPUT_KEY_CONTROLLER_MANAGER "controller_manager"
#define OUTPUT_KEY_TYPES "types"
#define OUTPUT_KEY_BASE_CLASSES "base_classes"

namespace auto_apms_ros2control
{

/**
 * @brief Queries all controller types the controller manager is able to load.
 *
 * The targeted controller manager's service name is derived from the 'controller_manager' input port via the
 * NodeRegistrationOptions::topic pattern "(input:controller_manager)/list_controller_types".
 */
class ListControllerTypes
: public auto_apms_behavior_tree::core::RosServiceNode<controller_manager_msgs::srv::ListControllerTypes>
{
public:
  using RosServiceNode::RosServiceNode;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::OutputPort<std::vector<std::string>>(OUTPUT_KEY_TYPES, "Names of all available controller types."),
      BT::OutputPort<std::vector<std::string>>(
        OUTPUT_KEY_BASE_CLASSES, "Base classes of the available controller types (aligned with 'types')."),
    };
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr & response) override final
  {
    setOutput(OUTPUT_KEY_TYPES, response->types);
    setOutput(OUTPUT_KEY_BASE_CLASSES, response->base_classes);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::ListControllerTypes)
