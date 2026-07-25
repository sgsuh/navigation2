// Copyright (c) 2021 Samsung Research America
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/feasible_path_handler.hpp"
#include "nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class RotationShimShim : public nav2_rotation_shim_controller::RotationShimController
{
public:
  RotationShimShim()
  : nav2_rotation_shim_controller::RotationShimController()
  {
  }

  nav2_core::Controller::Ptr getPrimaryController()
  {
    return primary_controller_;
  }

  bool isPathUpdated()
  {
    return path_updated_;
  }

  geometry_msgs::msg::PoseStamped getSampledPathPtWrapper(
    const geometry_msgs::msg::PoseStamped & global_goal)
  {
    return getSampledPathPt(global_goal);
  }

  geometry_msgs::msg::Pose transformPoseToBaseFrameWrapper(geometry_msgs::msg::PoseStamped pt)
  {
    return transformPoseToBaseFrame(pt);
  }

  geometry_msgs::msg::TwistStamped
  computeRotateToHeadingCommandWrapper(
    const double & param,
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity)
  {
    return computeRotateToHeadingCommand(param, pose, velocity);
  }

  // getSampledPathPt reads the protected current_path_/current_pose_ members, which
  // are otherwise only set from inside computeVelocityCommands. Setting them directly
  // lets the ported path-deviation check be exercised without routing through the
  // primary controller (whose own rotate-to-heading would otherwise mask the effect).
  void setPathAndPoseForTest(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::PoseStamped & pose)
  {
    current_path_ = path;
    current_pose_ = pose;
  }
};

TEST(RotationShimControllerTest, lifecycleTransitions)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  // Should not populate primary controller, does not exist
  EXPECT_THROW(ctrl->configure(node, name, tf, costmap), std::runtime_error);
  EXPECT_EQ(ctrl->getPrimaryController(), nullptr);

  // Add a controller to the setup
  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());
  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter(
        "PathFollower.primary_controller.plugin",
        std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"))});
  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  ctrl->configure(node, name, tf, costmap);
  EXPECT_NE(ctrl->getPrimaryController(), nullptr);

  ctrl->activate();

  ctrl->setSpeedLimit(50.0, true);

  ctrl->deactivate();
  ctrl->cleanup();
}

TEST(RotationShimControllerTest, setPlanAndSampledPointsTests)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  // disable collision check
  node->declare_parameter("PathFollower.simulate_ahead_time", 0.0);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(10);
  path.poses[1].pose.position.x = 0.1;
  path.poses[1].pose.position.y = 0.1;
  path.poses[2].pose.position.x = 1.0;
  path.poses[2].pose.position.y = 1.0;
  path.poses[3].pose.position.x = 10.0;
  path.poses[3].pose.position.y = 10.0;
  geometry_msgs::msg::PoseStamped robot_pose;
  nav2_controller::SimpleGoalChecker checker;
  geometry_msgs::msg::Twist velocity;
  auto goal = path.poses.back();
  EXPECT_EQ(controller->isPathUpdated(), false);
  controller->computeVelocityCommands(robot_pose, velocity, &checker, path, goal);
  controller->newPathReceived(path);
  EXPECT_EQ(controller->isPathUpdated(), true);

  // Test getting a sampled point
  geometry_msgs::msg::PoseStamped global_goal;
  auto pose = controller->getSampledPathPtWrapper(global_goal);
  EXPECT_EQ(pose.pose.position.x, 1.0);  // default forward sampling is 0.5
  EXPECT_EQ(pose.pose.position.y, 1.0);
}

// Ported feature: max_dist_from_path. When > 0, getSampledPathPt rejects the path with
// nav2_core::InvalidPath if the robot has strayed farther than the tolerance from the
// nearest path point; when the robot is on the path it samples normally. (<= 0 disables
// the check, which is covered by every other test here since the default is 0.0.)
TEST(RotationShimControllerTest, pathDeviationCheck)
{
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  node->declare_parameter(
    "PathFollower.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter("PathFollower.simulate_ahead_time", 0.0);
  // Enable the ported path-deviation check with a 0.5 m tolerance.
  node->declare_parameter("PathFollower.max_dist_from_path", 0.5);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  // A short path along +x near the origin, expressed in base_link so the pose->path-frame
  // transform inside getSampledPathPt is identity (no TF server required).
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(6);
  for (unsigned int i = 0; i < path.poses.size(); ++i) {
    path.poses[i].header.frame_id = "base_link";
    path.poses[i].pose.position.x = 0.2 * i;  // (0,0) .. (1.0, 0)
    path.poses[i].pose.position.y = 0.0;
  }
  geometry_msgs::msg::PoseStamped goal = path.poses.back();

  // On the path (distance 0 < 0.5 m): the deviation check passes and a point is sampled.
  geometry_msgs::msg::PoseStamped on_path;
  on_path.header.frame_id = "base_link";
  controller->setPathAndPoseForTest(path, on_path);
  EXPECT_NO_THROW(controller->getSampledPathPtWrapper(goal));

  // 2 m off the path (nearest point 2.0 m away, > 0.5 m): rejected as InvalidPath.
  geometry_msgs::msg::PoseStamped off_path;
  off_path.header.frame_id = "base_link";
  off_path.pose.position.y = 2.0;
  controller->setPathAndPoseForTest(path, off_path);
  EXPECT_THROW(controller->getSampledPathPtWrapper(goal), nav2_core::InvalidPath);
}

// Ported feature: avoid_overshoot. When enabled, computeRotateToHeadingCommand ramps the
// angular speed linearly from min_angular_vel to rotate_to_heading_angular_vel as the
// remaining heading error sweeps from stop_slowdown_threshold to start_slowdown_threshold,
// holding min_angular_vel below the stop threshold (instead of the default sqrt decel curve).
TEST(RotationShimControllerTest, avoidOvershootRamp)
{
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap", "/", false);
  costmap->configure();

  node->declare_parameter(
    "PathFollower.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter("controller_frequency", 1.0);  // dt = 1 s -> generous accel headroom
  node->declare_parameter("PathFollower.simulate_ahead_time", 0.0);  // skip the collision check
  node->declare_parameter("PathFollower.avoid_overshoot", true);
  node->declare_parameter("PathFollower.min_angular_vel", 0.1);
  node->declare_parameter("PathFollower.start_slowdown_threshold", 0.7);
  node->declare_parameter("PathFollower.stop_slowdown_threshold", 0.2);
  // rotate_to_heading_angular_vel defaults to 1.8; max_angular_accel to 3.2.

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  const geometry_msgs::msg::Twist zero_vel;  // current angular velocity 0
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";

  // >= start_slowdown_threshold (0.7): full speed, 0.1 + 1.0 * (1.8 - 0.1) = 1.8
  EXPECT_NEAR(
    controller->computeRotateToHeadingCommandWrapper(0.7, pose, zero_vel).twist.angular.z,
    1.8, 1e-3);
  // midpoint (0.45): factor 0.5, 0.1 + 0.5 * (1.8 - 0.1) = 0.95
  EXPECT_NEAR(
    controller->computeRotateToHeadingCommandWrapper(0.45, pose, zero_vel).twist.angular.z,
    0.95, 1e-3);
  // at stop_slowdown_threshold (0.2): factor 0, floored at min_angular_vel 0.1
  EXPECT_NEAR(
    controller->computeRotateToHeadingCommandWrapper(0.2, pose, zero_vel).twist.angular.z,
    0.1, 1e-3);
  // below the stop threshold: still floored at min_angular_vel, sign follows the error
  EXPECT_NEAR(
    controller->computeRotateToHeadingCommandWrapper(-0.1, pose, zero_vel).twist.angular.z,
    -0.1, 1e-3);
}

TEST(RotationShimControllerTest, rotationAndTransformTests)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap", "/", false);
  costmap->configure();

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));

  node->declare_parameter("controller_frequency", 1.0);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "fake_frame";
  path.poses.resize(10);
  path.poses[1].pose.position.x = 0.15;
  path.poses[1].pose.position.y = 0.15;
  path.poses[2].pose.position.x = 1.0;
  path.poses[2].pose.position.y = 1.0;
  path.poses[3].pose.position.x = 10.0;
  path.poses[3].pose.position.y = 10.0;
  controller->newPathReceived(path);

  const geometry_msgs::msg::Twist velocity;
  EXPECT_EQ(
    controller->computeRotateToHeadingCommandWrapper(
      0.7, path.poses[1], velocity).twist.angular.z, 1.8);
  EXPECT_EQ(
    controller->computeRotateToHeadingCommandWrapper(
      -0.7, path.poses[1], velocity).twist.angular.z, -1.8);

  EXPECT_EQ(
    controller->computeRotateToHeadingCommandWrapper(
      0.87, path.poses[1], velocity).twist.angular.z, 1.8);

  // in base_link, so should pass through values without issue
  geometry_msgs::msg::PoseStamped pt;
  pt.pose.position.x = 100.0;
  pt.header.frame_id = "base_link";
  pt.header.stamp = rclcpp::Time();
  auto rtn = controller->transformPoseToBaseFrameWrapper(pt);
  EXPECT_EQ(rtn.position.x, 100.0);

  // in frame that doesn't exist, shouldn't throw, but should fail
  geometry_msgs::msg::PoseStamped pt2;
  pt.pose.position.x = 100.0;
  pt.header.frame_id = "fake_frame2";
  pt.header.stamp = rclcpp::Time();
  EXPECT_THROW(controller->transformPoseToBaseFrameWrapper(pt2), std::runtime_error);
}

TEST(RotationShimControllerTest, computeVelocityTests)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  costmap->declare_parameter("origin_x", 0.0);
  costmap->declare_parameter("origin_y", 0.0);
  costmap->configure();
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter("controller_frequency", 1.0);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap, tf);

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.poses.resize(10);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  geometry_msgs::msg::Twist velocity;
  nav2_controller::SimpleGoalChecker checker;
  checker.initialize(node, "checker", costmap);


  path.header.frame_id = "base_link";
  path.poses[1].pose.position.x = 0.1;
  path.poses[1].pose.position.y = 0.1;
  path.poses[2].pose.position.x = -1.0;
  path.poses[2].pose.position.y = -1.0;
  path.poses[2].header.frame_id = "base_link";
  path.poses[3].pose.position.x = 10.0;
  path.poses[3].pose.position.y = 10.0;

  // this should allow it to find the sampled point, then transform to base_link
  // validly because we setup the TF for it. The -1.0 should be selected since default min
  // is 0.5 and that should cause a rotation in place
  controller->newPathReceived(path);
  path_handler.setPlan(path);
  tf_broadcaster->sendTransform(transform);
  geometry_msgs::msg::PoseStamped goal = path.poses.back();
  auto effort = controller->computeVelocityCommands(pose, velocity, &checker,
    path, goal);
  EXPECT_EQ(fabs(effort.twist.angular.z), 1.8);
}

TEST(RotationShimControllerTest, openLoopRotationTests) {
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter(
    "controller_frequency",
    20.0);
  node->declare_parameter(
    "PathFollower.rotate_to_goal_heading",
    true);
  node->declare_parameter(
    "PathFollower.closed_loop",
    false);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap, tf);

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(4);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  geometry_msgs::msg::Twist velocity;
  nav2_controller::SimpleGoalChecker checker;
  node->declare_parameter(
    "checker.xy_goal_tolerance",
    1.0);
  checker.initialize(node, "checker", costmap);

  path.header.frame_id = "base_link";
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.05;
  path.poses[1].pose.position.y = 0.05;
  path.poses[2].pose.position.x = 0.10;
  path.poses[2].pose.position.y = 0.10;
  // goal position within checker xy_goal_tolerance
  path.poses[3].pose.position.x = 0.20;
  path.poses[3].pose.position.y = 0.20;
  // goal heading 45 degrees to the left
  path.poses[3].pose.orientation.z = -0.3826834;
  path.poses[3].pose.orientation.w = 0.9238795;
  path.poses[3].header.frame_id = "base_link";

  // Calculate first velocity command
  controller->newPathReceived(path);
  path_handler.setPlan(path);
  auto [closest_point, pruned_plan_end] = path_handler.findPlanSegment(pose);
  nav_msgs::msg::Path transformed_global_plan = path_handler.transformLocalPlan(closest_point,
    pruned_plan_end);
  geometry_msgs::msg::PoseStamped goal = path.poses.back();
  auto cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker,
    transformed_global_plan, goal);
  EXPECT_NEAR(cmd_vel.twist.angular.z, -0.16, 1e-4);

  // Test second velocity command with wrong odometry
  velocity.angular.z = 1.8;
  cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker, transformed_global_plan,
    goal);
  EXPECT_NEAR(cmd_vel.twist.angular.z, -0.32, 1e-4);
}

TEST(RotationShimControllerTest, computeVelocityGoalRotationTests) {
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter(
    "PathFollower.rotate_to_goal_heading",
    true);
  node->declare_parameter("controller_frequency", 1.0);
  node->declare_parameter("PathFollower.primary_controller.use_collision_detection", false);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap, tf);

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(4);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  geometry_msgs::msg::Twist velocity;
  nav2_controller::SimpleGoalChecker checker;
  node->declare_parameter(
    "checker.xy_goal_tolerance",
    1.0);
  checker.initialize(node, "checker", costmap);

  path.header.frame_id = "base_link";
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.05;
  path.poses[1].pose.position.y = 0.05;
  path.poses[2].pose.position.x = 0.10;
  path.poses[2].pose.position.y = 0.10;
  // goal position within checker xy_goal_tolerance
  path.poses[3].pose.position.x = 0.20;
  path.poses[3].pose.position.y = 0.20;
  // goal heading 45 degrees to the left
  path.poses[3].pose.orientation.z = -0.3826834;
  path.poses[3].pose.orientation.w = 0.9238795;
  path.poses[3].header.frame_id = "base_link";

  controller->newPathReceived(path);
  path_handler.setPlan(path);
  geometry_msgs::msg::PoseStamped goal = path.poses.back();
  auto cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker,
    path, goal);
  EXPECT_EQ(cmd_vel.twist.angular.z, -1.8);

  // goal heading 45 degrees to the right
  path.poses[3].pose.orientation.z = 0.3826834;
  path.poses[3].pose.orientation.w = 0.9238795;
  controller->newPathReceived(path);
  path_handler.setPlan(path);
  goal = path.poses.back();
  cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker, path,
    goal);
  EXPECT_EQ(cmd_vel.twist.angular.z, 1.8);
}

TEST(RotationShimControllerTest, accelerationTests) {
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter(
    "controller_frequency",
    20.0);
  node->declare_parameter(
    "PathFollower.rotate_to_goal_heading",
    true);
  node->declare_parameter(
    "PathFollower.max_angular_accel",
    0.5);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();
  nav2_controller::FeasiblePathHandler path_handler;
  path_handler.initialize(node, node->get_logger(), "path_handler", costmap, tf);

  // Test state update and path setting
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(4);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  geometry_msgs::msg::Twist velocity;
  nav2_controller::SimpleGoalChecker checker;
  node->declare_parameter(
    "checker.xy_goal_tolerance",
    1.0);
  checker.initialize(node, "checker", costmap);

  path.header.frame_id = "base_link";
  path.poses[0].pose.position.x = 0.0;
  path.poses[0].pose.position.y = 0.0;
  path.poses[1].pose.position.x = 0.05;
  path.poses[1].pose.position.y = 0.05;
  path.poses[2].pose.position.x = 0.10;
  path.poses[2].pose.position.y = 0.10;
  // goal position within checker xy_goal_tolerance
  path.poses[3].pose.position.x = 0.20;
  path.poses[3].pose.position.y = 0.20;
  // goal heading 45 degrees to the left
  path.poses[3].pose.orientation.z = -0.3826834;
  path.poses[3].pose.orientation.w = 0.9238795;
  path.poses[3].header.frame_id = "base_link";

  // Test acceleration limits
  controller->newPathReceived(path);
  path_handler.setPlan(path);
  auto [closest_point, pruned_plan_end] = path_handler.findPlanSegment(pose);
  nav_msgs::msg::Path transformed_global_plan = path_handler.transformLocalPlan(closest_point,
    pruned_plan_end);
  geometry_msgs::msg::PoseStamped goal = path.poses.back();
  auto cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker,
    transformed_global_plan, goal);
  EXPECT_EQ(cmd_vel.twist.angular.z, -0.025);

  // Test slowing down to avoid overshooting
  velocity.angular.z = -1.8;
  cmd_vel = controller->computeVelocityCommands(pose, velocity, &checker, transformed_global_plan,
    goal);
  EXPECT_NEAR(cmd_vel.twist.angular.z, -std::sqrt(2 * 0.5 * M_PI / 4), 1e-4);
}

TEST(RotationShimControllerTest, isGoalChangedTest)
{
  auto ctrl = std::make_shared<RotationShimShim>();
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "PathFollower";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*tf, node, true);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  geometry_msgs::msg::PoseStamped robot_pose;
  nav2_controller::SimpleGoalChecker checker;
  geometry_msgs::msg::Twist velocity;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "odom";
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster->sendTransform(transform);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "PathFollower.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter(
    "PathFollower.rotate_to_heading_once",
    true);
  // disable collision check
  node->declare_parameter("PathFollower.simulate_ahead_time", 0.0);

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(2);
  path.poses.back().pose.position.x = 2.0;
  path.poses.back().pose.position.y = 2.0;
  auto goal = path.poses.back();

  // Test: Current path is empty, should return false
  EXPECT_EQ(controller->isPathUpdated(), false);

  // Test: Last pose of the current path is the same, should return true
  controller->newPathReceived(path);
  controller->computeVelocityCommands(robot_pose, velocity, &checker, path, goal);
  EXPECT_EQ(controller->isPathUpdated(), true);

  // Test: Last pose of the current path differs, should return true
  path.poses.back().pose.position.x = 3.0;
  goal = path.poses.back();
  controller->computeVelocityCommands(robot_pose, velocity, &checker, path, goal);
  EXPECT_EQ(controller->isPathUpdated(), true);
}

TEST(RotationShimControllerTest, testDynamicParameter)
{
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  std::string name = "test";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  rclcpp_lifecycle::State state;
  costmap->on_configure(state);

  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "test.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));

  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    node->get_node_base_interface(), node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.angular_dist_threshold", 7.0),
      rclcpp::Parameter("test.forward_sampling_distance", 7.0),
      rclcpp::Parameter("test.rotate_to_heading_angular_vel", 7.0),
      rclcpp::Parameter("test.max_angular_accel", 7.0),
      rclcpp::Parameter("test.max_cost_threshold",
      static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)),
      rclcpp::Parameter("test.simulate_ahead_time", 7.0),
      rclcpp::Parameter("test.primary_controller.plugin", std::string("HI")),
      rclcpp::Parameter("test.rotate_to_goal_heading", true),
      rclcpp::Parameter("test.rotate_to_heading_once", true),
      rclcpp::Parameter("test.closed_loop", false),
      rclcpp::Parameter("test.use_path_orientations", true)});

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.angular_dist_threshold").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.forward_sampling_distance").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_angular_vel").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.max_angular_accel").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.max_cost_threshold").as_double(),
    static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE));
  EXPECT_EQ(node->get_parameter("test.simulate_ahead_time").as_double(), 7.0);
  EXPECT_EQ(node->get_parameter("test.rotate_to_goal_heading").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.rotate_to_heading_once").as_bool(), true);
  EXPECT_EQ(node->get_parameter("test.closed_loop").as_bool(), false);
  EXPECT_EQ(node->get_parameter("test.use_path_orientations").as_bool(), true);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.angular_dist_threshold", -1.0)}
  );

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.angular_dist_threshold").as_double(), 7.0);

  results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.simulate_ahead_time", -0.1)}
  );

  rclcpp::spin_until_future_complete(
    node->get_node_base_interface(),
    results);

  EXPECT_EQ(node->get_parameter("test.simulate_ahead_time").as_double(), 7.0);
}

TEST(RotationShimControllerTest, maxCostThresholdTest)
{
  auto node = std::make_shared<nav2::LifecycleNode>("ShimControllerTest");
  std::string name = "test";
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("fake_costmap");
  costmap->configure();
  // set a valid primary controller so we can do lifecycle
  node->declare_parameter(
    "test.primary_controller.plugin",
    std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"));
  node->declare_parameter("controller_frequency", 20.0);
  node->declare_parameter("test.max_cost_threshold",
    static_cast<double>(nav2_costmap_2d::FREE_SPACE));
  auto controller = std::make_shared<RotationShimShim>();
  controller->configure(node, name, tf, costmap);
  controller->activate();
  nav_msgs::msg::Path path;
  path.header.frame_id = "base_link";
  path.poses.resize(4);
  path.poses[1].pose.position.x = 0.15;
  path.poses[1].pose.position.y = 0.15;
  path.poses[2].pose.position.x = 1.0;
  path.poses[2].pose.position.y = 1.0;
  controller->newPathReceived(path);
  const geometry_msgs::msg::Twist velocity;
  // With max_cost_threshold=0, even FREE_SPACE cells should trigger collision
  EXPECT_THROW(
    controller->computeRotateToHeadingCommandWrapper(1.5, path.poses[1], velocity),
    nav2_core::NoValidControl);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
