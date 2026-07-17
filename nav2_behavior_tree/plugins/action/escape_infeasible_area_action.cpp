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

#include <string>
#include <memory>

#include "nav2_behavior_tree/plugins/action/escape_infeasible_area_action.hpp"

namespace nav2_behavior_tree
{

EscapeInfeasibleAreaAction::EscapeInfeasibleAreaAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::EscapeInfeasibleArea>(xml_tag_name, action_name, conf),
  is_recovery_(true)
{
}

void EscapeInfeasibleAreaAction::initialize()
{
  double min_dist;
  getInput("min_distance", min_dist);
  double max_dist;
  getInput("max_distance", max_dist);
  double speed;
  getInput("speed", speed);
  bool prioritize_backup;
  getInput("prioritize_backup", prioritize_backup);
  bool avoid_collision;
  getInput("avoid_collision", avoid_collision);
  bool collision_avoidance_is_failure;
  getInput("collision_avoidance_is_failure", collision_avoidance_is_failure);
  double time_allowance;
  getInput("time_allowance", time_allowance);
  getInput("is_recovery", is_recovery_);

  goal_.min_distance = min_dist;
  goal_.max_distance = max_dist;
  goal_.speed = speed;
  goal_.prioritize_backup.data = prioritize_backup;
  goal_.avoid_collision.data = avoid_collision;
  goal_.collision_avoidance_is_failure.data = collision_avoidance_is_failure;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

void EscapeInfeasibleAreaAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (is_recovery_) {
    increment_recovery_count();
  }
}

BT::NodeStatus EscapeInfeasibleAreaAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus EscapeInfeasibleAreaAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus EscapeInfeasibleAreaAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void EscapeInfeasibleAreaAction::on_timeout()
{
  setOutput("error_code_id", ActionResult::UNKNOWN);
  setOutput("error_msg", "Behavior Tree action client timed out waiting.");
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::EscapeInfeasibleAreaAction>(
        name, "escape_infeasible_area", config);
    };

  factory.registerBuilder<nav2_behavior_tree::EscapeInfeasibleAreaAction>(
    "EscapeInfeasibleArea", builder);
}
