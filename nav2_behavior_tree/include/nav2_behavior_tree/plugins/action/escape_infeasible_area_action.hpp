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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ESCAPE_INFEASIBLE_AREA_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ESCAPE_INFEASIBLE_AREA_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/escape_infeasible_area.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps
 * nav2_msgs::action::EscapeInfeasibleArea
 */
class EscapeInfeasibleAreaAction : public BtActionNode<nav2_msgs::action::EscapeInfeasibleArea>
{
  using Action = nav2_msgs::action::EscapeInfeasibleArea;
  using ActionResult = Action::Result;

public:
  /**
   * @brief A constructor for nav2_behavior_tree::EscapeInfeasibleAreaAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  EscapeInfeasibleAreaAction(
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
        BT::InputPort<double>("min_distance", 0.2, "Minimum distance to move"),
        BT::InputPort<double>("max_distance", 1.0, "Maximum distance to move"),
        BT::InputPort<double>("speed", 0.1, "Desired speed"),
        BT::InputPort<bool>(
          "prioritize_backup", true, "Prioritize reverse motion when both ways are feasible"),
        BT::InputPort<bool>("avoid_collision", true, "Whether to avoid collision while moving"),
        BT::InputPort<bool>(
          "collision_avoidance_is_failure", true,
          "Whether avoiding collision should be treated as failure or not"),
        BT::InputPort<double>("time_allowance", 15.0, "Allowed time for behavior completion"),
        BT::InputPort<bool>("is_recovery", true, "True if recovery"),
        BT::OutputPort<ActionResult::_error_code_type>(
          "error_code_id", "The escape infeasible area behavior server error code"),
        BT::OutputPort<std::string>(
          "error_msg", "The escape infeasible area behavior server error msg"),
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

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ESCAPE_INFEASIBLE_AREA_ACTION_HPP_
