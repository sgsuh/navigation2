// Copyright (c) 2026 Open Navigation LLC
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
#include <string>
#include <thread>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;  // NOLINT

// Both managers in launch_transition_timeout_test.py use transition_timeout: 1.0.
//
// A node that sits in on_configure serves as a stand-in for the failure these tests
// exist for: a change_state whose reply does not come back in time. In the field the
// cause is the reply being dropped outright -- rmw_fastrtps gives up when the server
// answers before the client's response reader has matched, logging "failed to send
// response to <node>/change_state (timeout): client will not receive response" --
// which is not reproducible on demand, whereas blocking is.
//
// The distinction the two durations draw is whether the node manages to reach the
// target state while the manager is still looking.
constexpr auto kUnrecoverableConfigure = 15s;  // outlasts the state check as well
constexpr auto kRecoverableConfigure = 3s;     // finishes while the state check waits

class SlowLifecycleNode : public nav2::LifecycleNode
{
public:
  SlowLifecycleNode(const std::string & name, std::chrono::seconds configure_duration)
  : nav2::LifecycleNode(name), configure_duration_(configure_duration) {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Configuring slowly, on purpose...");
    std::this_thread::sleep_for(configure_duration_);
    return CallbackReturn::SUCCESS;
  }

private:
  std::chrono::seconds configure_duration_;
};

/**
 * A transition that is never confirmed must be reported, not waited on forever.
 *
 * Before this was bounded, lifecycle_manager passed milliseconds(-1) as the
 * transition timeout, which routes LifecycleServiceClient::change_state to the
 * untimed ServiceClient::invoke overload -- spin_until_complete with no deadline.
 * `service_timeout` does not help: it bounds only wait_for_service, not the reply.
 * A dropped response therefore wedged bringup permanently, on the manager's own
 * thread, and the process then had to be SIGKILLed.
 *
 * This node stays busy long enough that neither the transition reply nor the
 * follow-up state read can confirm anything, so retrying cannot help either and the
 * manager has to give up. Note the assertion is that startup() returns *false*: with
 * the old unbounded wait it returned true after kUnrecoverableConfigure, so this
 * fails on the old behaviour rather than hanging.
 */
TEST(TransitionTimeoutTest, UnconfirmableTransitionIsReportedNotAwaited)
{
  auto slow_node = std::make_shared<SlowLifecycleNode>(
    "slow_lifecycle_node", kUnrecoverableConfigure);
  auto slow_thread = std::make_unique<nav2::NodeThread>(slow_node->get_node_base_interface());

  auto node = std::make_shared<rclcpp::Node>("transition_timeout_test_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_slow_test", node);

  const auto start = std::chrono::steady_clock::now();
  const bool brought_up = client.startup();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(brought_up) << "bringup should be reported as failed once the transition "
                              "can be neither acknowledged nor confirmed";

  EXPECT_LT(elapsed, kUnrecoverableConfigure - 3s)
    << "took " << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
    << "s; the wait was not bounded";
}

/**
 * A transition that completes without being acknowledged must be recovered from.
 *
 * This is the shape of the real failure: the node does everything asked of it and
 * only the reply goes missing. Bounding the wait alone turned that into a clean
 * abort but still lost the run, so the manager now treats the node's own state as
 * the authority and carries on when it finds the node already where it should be.
 *
 * Here the reply misses the 1 s bound but the node reaches `inactive` while the
 * follow-up get_state is still waiting on it, so bringup must succeed -- and must do
 * so without re-issuing the transition, which would be rejected as invalid from the
 * state the node is now in.
 */
TEST(TransitionTimeoutTest, UnacknowledgedButCompletedTransitionRecovers)
{
  auto node_under_test = std::make_shared<SlowLifecycleNode>(
    "recovering_lifecycle_node", kRecoverableConfigure);
  auto thread = std::make_unique<nav2::NodeThread>(node_under_test->get_node_base_interface());

  auto node = std::make_shared<rclcpp::Node>("transition_recovery_test_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_recover_test", node);

  const auto start = std::chrono::steady_clock::now();
  const bool brought_up = client.startup();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_TRUE(brought_up) << "the node did reach the target state, so a lost "
                             "acknowledgement must not fail the bringup";

  // Recovery comes from reading the node's state, not from waiting out retries.
  EXPECT_LT(elapsed, 10s)
    << "took " << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() << "s";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
