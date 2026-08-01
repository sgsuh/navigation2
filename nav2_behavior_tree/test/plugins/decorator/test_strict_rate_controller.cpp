// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <set>
#include <string>

#include "utils/test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/decorator/strict_rate_controller.hpp"

using namespace std::chrono;  // NOLINT
using namespace std::chrono_literals;  // NOLINT

namespace nav2_behavior_tree
{

/**
 * @brief A DummyNode that also records how many times it was ticked.
 *
 * The tick count is what separates StrictRateController from RateController: the
 * difference between the two is not the status they return but how often the child
 * is actually executed.
 */
class CountingDummyNode : public DummyNode
{
public:
  CountingDummyNode()
  : DummyNode() {}

  BT::NodeStatus tick() override
  {
    tick_count_++;
    return DummyNode::tick();
  }

  int tickCount() const {return tick_count_;}

  void resetTickCount() {tick_count_ = 0;}

private:
  int tick_count_{0};
};

}  // namespace nav2_behavior_tree

class StrictRateControllerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    // 10 Hz -> a 100 ms period, so the test spends ~1 s in total.
    // The port map is <string, string>: assigning a numeric literal here would bind
    // std::string::operator=(char), leaving an unparseable value and silently falling
    // back to the 1 Hz default.
    config_->input_ports["hz"] = "10.0";
    bt_node_ = std::make_shared<nav2_behavior_tree::StrictRateController>(
      "strict_rate_controller", *config_);
    dummy_node_ = std::make_shared<nav2_behavior_tree::CountingDummyNode>();
    bt_node_->setChild(dummy_node_.get());
  }

  void TearDown()
  {
    dummy_node_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::StrictRateController> bt_node_;
  static std::shared_ptr<nav2_behavior_tree::CountingDummyNode> dummy_node_;
};

std::shared_ptr<nav2_behavior_tree::StrictRateController>
StrictRateControllerTestFixture::bt_node_ = nullptr;
std::shared_ptr<nav2_behavior_tree::CountingDummyNode>
StrictRateControllerTestFixture::dummy_node_ = nullptr;

/**
 * The child is ticked on the first tick and then once per period, and the status the
 * child returns is passed straight through.
 */
TEST_F(StrictRateControllerTestFixture, test_ticks_child_once_per_period)
{
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  // First tick always reaches the child
  dummy_node_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(dummy_node_->tickCount(), 1);

  // Within the period the child is not ticked again. The decorator does not repeat the
  // child's last status either: tick() sets its own status to RUNNING before the period
  // check, so a gated tick always reports RUNNING. The child's SUCCESS above is seen by
  // the parent on that one tick only.
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  }
  EXPECT_EQ(dummy_node_->tickCount(), 1);

  // Once the period elapses the child is ticked exactly once more
  std::this_thread::sleep_for(150ms);
  dummy_node_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(dummy_node_->tickCount(), 2);
}

/**
 * The decisive difference from RateController: a RUNNING child is NOT re-ticked
 * between periods.
 *
 * This is the property the path-patience gate in
 * navigate_to_pose_w_path_patience_and_graceful_recovery.xml relies on. Because an
 * asynchronous child (ValidatePath is a BtServiceNode) is only re-ticked on a period
 * boundary, its result cannot be harvested before then, which is what turns
 * RetryUntilSuccessful's `num_attempts` into a wall-clock duration of
 * num_attempts / hz seconds instead of a count of service round trips.
 */
TEST_F(StrictRateControllerTestFixture, test_does_not_retick_running_child)
{
  dummy_node_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->tickCount(), 1);

  // Tick hard for well under one period. RateController would tick the child on every
  // one of these, because its rule is "keep ticking a RUNNING child til completion".
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_EQ(dummy_node_->tickCount(), 1);

  // The child is only reached again on the next period boundary
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->tickCount(), 2);
}

/**
 * Halting resets the decorator, so the next tick starts a fresh period and reaches the
 * child immediately. RetryUntilSuccessful halts its child after every failed attempt,
 * so this is what makes each attempt start its own period rather than inherit a
 * partially elapsed one.
 */
TEST_F(StrictRateControllerTestFixture, test_reinitializes_after_halt)
{
  dummy_node_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->tickCount(), 1);

  // Still inside the period: without the halt the child would not be reached
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->tickCount(), 1);

  bt_node_->halt();
  EXPECT_EQ(bt_node_->status(), BT::NodeStatus::IDLE);

  dummy_node_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(dummy_node_->tickCount(), 2);
}

/**
 * The period comes from the `hz` port, read on initialize().
 */
TEST_F(StrictRateControllerTestFixture, test_honours_hz_port)
{
  config_->input_ports["hz"] = "2.0";
  auto slow_node = std::make_shared<nav2_behavior_tree::StrictRateController>(
    "slow_strict_rate_controller", *config_);
  auto child = std::make_shared<nav2_behavior_tree::CountingDummyNode>();
  slow_node->setChild(child.get());

  child->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(slow_node->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child->tickCount(), 1);

  // 150 ms would already be a full period at 10 Hz, but is well short of one at 2 Hz
  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(slow_node->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child->tickCount(), 1);

  std::this_thread::sleep_for(400ms);
  EXPECT_EQ(slow_node->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child->tickCount(), 2);

  // Leave the shared fixture config as the other tests expect it
  config_->input_ports["hz"] = "10.0";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  bool all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
