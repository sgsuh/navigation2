// Copyright (c) 2024 Open Navigation LLC
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__STOP_SLOWLY_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__STOP_SLOWLY_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/stop_slowly.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps
 * nav2_msgs::action::StopSlowly
 */
class StopSlowlyAction : public BtActionNode<nav2_msgs::action::StopSlowly>
{
  using Action = nav2_msgs::action::StopSlowly;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::StopSlowlyAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  StopSlowlyAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("max_distance", 1.5, "Maximum distance to allow during deceleration"),
        BT::InputPort<double>("desired_deceleration", 0.15, "Desired deceleration"),
        BT::InputPort<bool>("consider_collision", true, "Whether to lookout for potential collision"),
        BT::InputPort<bool>("is_recovery", false, "True if recovery"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The stop slowly behavior server error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The stop slowly behavior server error msg"),
      });
  }

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancellation of the action
   */
  BT::NodeStatus on_cancelled() override;

  /**
   * @brief Function to perform work in a BT Node when the action server times out
   */
  void on_timeout() override;

private:
  bool is_recovery_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__STOP_SLOWLY_ACTION_HPP_
