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

#include <limits>

#include "auto_apms_behavior_tree_core/node.hpp"
#include "pal_statistics_msgs/msg/statistics.hpp"

#define INPUT_KEY_CONTROLLER_MANAGER "controller_manager"
#define INPUT_KEY_INTERFACE "interface"
#define INPUT_KEY_MIN "min"
#define INPUT_KEY_MAX "max"
#define INPUT_KEY_TIMEOUT "timeout"
#define OUTPUT_KEY_VALUE "value"

namespace auto_apms_ros2control
{

/**
 * @brief Checks the current value of a state, command or reference interface of the controller manager.
 *
 * The controller manager publishes the values of all its interfaces on 'introspection_data/full', named
 * "state_interface.<name>", "command_interface.<name>" and so on. This node looks up the entry named by the 'interface'
 * input port in the most recent message and returns SUCCESS if its value lies within ['min', 'max'] (both included,
 * unbounded by default). It returns FAILURE if the value is out of range, if the interface is not part of the
 * introspection, or if no message was received within the last 'timeout' seconds, so that a controller manager that
 * went silent never passes the check. The value is written to the 'value' output port whenever it is known.
 *
 * The topic is derived from the 'controller_manager' input port via the NodeRegistrationOptions::topic pattern
 * "(input:controller_manager)/introspection_data/full".
 */
class CheckInterfaceValue : public auto_apms_behavior_tree::core::RosSubscriberNode<pal_statistics_msgs::msg::Statistics>
{
public:
  CheckInterfaceValue(
    const std::string & instance_name, const BT::NodeConfig & config,
    const auto_apms_behavior_tree::core::RosNodeContext & context)
  : RosSubscriberNode{instance_name, config, context, rclcpp::SensorDataQoS{}}
  {
  }

  static BT::PortsList providedPorts()
  {
    // We do not use the default 'topic' port — the topic is derived from the 'controller_manager' port via the
    // NodeRegistrationOptions::topic pattern "(input:controller_manager)/introspection_data/full".
    return {
      BT::InputPort<std::string>(
        INPUT_KEY_CONTROLLER_MANAGER, "controller_manager", "Name of the targeted controller manager."),
      BT::InputPort<std::string>(
        INPUT_KEY_INTERFACE,
        "Name of the interface as published by the introspection of the controller manager, e.g. "
        "'state_interface.joint1/position'."),
      BT::InputPort<double>(
        INPUT_KEY_MIN, -std::numeric_limits<double>::infinity(), "Smallest value that passes the check."),
      BT::InputPort<double>(
        INPUT_KEY_MAX, std::numeric_limits<double>::infinity(), "Largest value that passes the check."),
      BT::InputPort<double>(
        INPUT_KEY_TIMEOUT, 0.5, "Age [s] of the most recent introspection message above which the check fails."),
      BT::OutputPort<double>(OUTPUT_KEY_VALUE, "Current value of the interface."),
    };
  }

  BT::NodeStatus onTick(const std::shared_ptr<pal_statistics_msgs::msg::Statistics> & last_msg_ptr) override final
  {
    const rclcpp::Time now = context_.getCurrentTime();
    if (last_msg_ptr) {
      last_msg_ = last_msg_ptr;
      last_msg_time_ = now;
    }

    const BT::Expected<std::string> expected_interface = getInput<std::string>(INPUT_KEY_INTERFACE);
    if (!expected_interface || expected_interface.value().empty()) {
      RCLCPP_ERROR(
        logger_, "%s - %s", context_.getFullyQualifiedTreeNodeName(this).c_str(),
        expected_interface ? "Interface name must not be empty." : expected_interface.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    const std::string & interface = expected_interface.value();

    if (!last_msg_) {
      RCLCPP_DEBUG(
        logger_, "%s - No introspection data received yet.", context_.getFullyQualifiedTreeNodeName(this).c_str());
      return BT::NodeStatus::FAILURE;
    }
    const double timeout = getInput<double>(INPUT_KEY_TIMEOUT).value_or(0.5);
    if ((now - last_msg_time_).seconds() > timeout) {
      RCLCPP_DEBUG(
        logger_, "%s - The last introspection data is older than %.2f s.",
        context_.getFullyQualifiedTreeNodeName(this).c_str(), timeout);
      return BT::NodeStatus::FAILURE;
    }

    for (const pal_statistics_msgs::msg::Statistic & statistic : last_msg_->statistics) {
      if (statistic.name != interface) continue;
      setOutput(OUTPUT_KEY_VALUE, statistic.value);
      const double min = getInput<double>(INPUT_KEY_MIN).value_or(-std::numeric_limits<double>::infinity());
      const double max = getInput<double>(INPUT_KEY_MAX).value_or(std::numeric_limits<double>::infinity());
      if (statistic.value < min || statistic.value > max) {
        RCLCPP_DEBUG(
          logger_, "%s - Interface '%s' has the value %g, outside of [%g, %g].",
          context_.getFullyQualifiedTreeNodeName(this).c_str(), interface.c_str(), statistic.value, min, max);
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_WARN_THROTTLE(
      logger_, *context_.getRosNode()->get_clock(), 5000, "%s - Interface '%s' is not part of the introspection.",
      context_.getFullyQualifiedTreeNodeName(this).c_str(), interface.c_str());
    return BT::NodeStatus::FAILURE;
  }

private:
  std::shared_ptr<pal_statistics_msgs::msg::Statistics> last_msg_;
  rclcpp::Time last_msg_time_;
};

}  // namespace auto_apms_ros2control

AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(auto_apms_ros2control::CheckInterfaceValue)
