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
#include <thread>

#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_thread.hpp"
#include "rclcpp/rclcpp.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;  // NOLINT

// Must comfortably exceed the manager's transition_timeout, set to 1 s by
// launch_transition_timeout_test.py.
constexpr auto kConfigureDuration = 8s;

/**
 * @brief A lifecycle node whose configure transition outlasts the manager's patience.
 *
 * This stands in for the failure this test exists for: a change_state call whose
 * response does not come back in time. In the field the cause is the response being
 * dropped entirely -- rmw_fastrtps gives up when the server answers before the
 * client's response reader has matched, logging "failed to send response to
 * <node>/change_state (timeout): client will not receive response" -- which is
 * unrecoverable, whereas simply blocking here is reproducible.
 *
 * Either way the manager is waiting on a reply it will not get in time, and the
 * behaviour under test is that it gives up and says so.
 */
class SlowLifecycleNode : public nav2::LifecycleNode
{
public:
  SlowLifecycleNode()
  : nav2::LifecycleNode("slow_lifecycle_node") {}

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_INFO(get_logger(), "Configuring slowly, on purpose...");
    std::this_thread::sleep_for(kConfigureDuration);
    return CallbackReturn::SUCCESS;
  }
};

/**
 * A transition that outruns `transition_timeout` must be reported as a failure.
 *
 * Before this was bounded, lifecycle_manager passed milliseconds(-1) as the
 * transition timeout, which routes LifecycleServiceClient::change_state to the
 * untimed ServiceClient::invoke overload -- spin_until_complete with no deadline.
 * `service_timeout` does not help: it only bounds wait_for_service, not the reply.
 * A dropped response therefore wedged bringup permanently, on the manager's own
 * thread, and the process then had to be SIGKILLed.
 *
 * Note this asserts startup() returns *false*, not merely that it returns. With the
 * old unbounded wait this node's transition does eventually succeed, so startup()
 * came back true after kConfigureDuration -- the test fails on the old behaviour
 * rather than hanging, which keeps it quick and unambiguous.
 */
TEST(TransitionTimeoutTest, SlowTransitionIsReportedNotAwaited)
{
  auto slow_node = std::make_shared<SlowLifecycleNode>();
  auto slow_thread = std::make_unique<nav2::NodeThread>(slow_node->get_node_base_interface());

  auto node = std::make_shared<rclcpp::Node>("transition_timeout_test_client");
  nav2_lifecycle_manager::LifecycleManagerClient client("lifecycle_manager_slow_test", node);

  const auto start = std::chrono::steady_clock::now();
  const bool brought_up = client.startup();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(brought_up) << "bringup should fail once the transition outruns "
                              "transition_timeout, not wait for it to finish";

  // The manager gives up after ~1 s. Anything close to kConfigureDuration means it
  // sat on the reply instead of bounding the wait.
  EXPECT_LT(elapsed, kConfigureDuration - 2s)
    << "took " << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
    << "s; the wait was not bounded";
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return all_successful;
}
