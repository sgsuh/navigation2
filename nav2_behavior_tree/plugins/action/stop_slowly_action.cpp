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

#include "nav2_behavior_tree/plugins/action/stop_slowly_action.hpp"

namespace nav2_behavior_tree
{

StopSlowlyAction::StopSlowlyAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<nav2_msgs::action::StopSlowly>(xml_tag_name, action_name, conf),
  is_recovery_(false)
{
}

void StopSlowlyAction::initialize()
{
  double max_dist;
  getInput("max_distance", max_dist);
  double desired_decel;
  getInput("desired_deceleration", desired_decel);
  bool consider_collision;
  getInput("consider_collision", consider_collision);
  getInput("is_recovery", is_recovery_);

  goal_.max_distance = max_dist;
  goal_.desired_deceleration = desired_decel;
  goal_.consider_collision.data = consider_collision;
}

void StopSlowlyAction::on_tick()
{
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  if (is_recovery_) {
    increment_recovery_count();
  }
}

BT::NodeStatus StopSlowlyAction::on_success()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StopSlowlyAction::on_aborted()
{
  setOutput("error_code_id", result_.result->error_code);
  setOutput("error_msg", result_.result->error_msg);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus StopSlowlyAction::on_cancelled()
{
  setOutput("error_code_id", ActionResult::NONE);
  setOutput("error_msg", "");
  return BT::NodeStatus::SUCCESS;
}

void StopSlowlyAction::on_timeout()
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
      return std::make_unique<nav2_behavior_tree::StopSlowlyAction>(
        name, "stop_slowly", config);
    };

  factory.registerBuilder<nav2_behavior_tree::StopSlowlyAction>("StopSlowly", builder);
}
