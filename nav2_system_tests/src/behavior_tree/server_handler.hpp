// Copyright (c) 2020 Vinny Ruia
// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2018 Intel Corporation
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
// limitations under the License. Reserved.

#ifndef BEHAVIOR_TREE__SERVER_HANDLER_HPP_
#define BEHAVIOR_TREE__SERVER_HANDLER_HPP_

#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_costmap_around_pose.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_route.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav2_msgs/action/stop_slowly.hpp"
#include "nav2_msgs/action/escape_infeasible_area.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

#include "dummy_action_server.hpp"
#include "dummy_service.hpp"

namespace nav2_system_tests
{

class DummyComputePathToPoseActionServer
  : public DummyActionServer<nav2_msgs::action::ComputePathToPose>
{
public:
  explicit DummyComputePathToPoseActionServer(const rclcpp::Node::SharedPtr & node)
  : DummyActionServer(node, "compute_path_to_pose")
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node->get_clock()->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    result_->path.header.stamp = node->now();
    result_->path.header.frame_id = pose.header.frame_id;
    for (int i = 0; i < 6; ++i) {
      result_->path.poses.push_back(pose);
    }
  }

  /**
   * @brief Choose which error code a failed goal reports.
   *
   * Which one it is decides whether the failure reaches a tree's recovery branch at
   * all: WouldAPlannerRecoveryHelp accepts only UNKNOWN, NO_VALID_PATH and TIMEOUT, so
   * START_OCCUPIED has to be selected explicitly to exercise a tree that handles it.
   */
  void setFailureErrorCode(uint16_t error_code, const std::string & error_msg)
  {
    failure_error_code_ = error_code;
    failure_error_msg_ = error_msg;
  }

  void reset() override
  {
    failure_error_code_ = nav2_msgs::action::ComputePathToPose::Result::TIMEOUT;
    failure_error_msg_ = "Timeout";
    DummyActionServer::reset();
  }

protected:
  void updateResultForFailure(
    std::shared_ptr<nav2_msgs::action::ComputePathToPose::Result>
    & result) override
  {
    result->error_code = failure_error_code_;
    result->error_msg = failure_error_msg_;
  }

  uint16_t failure_error_code_{nav2_msgs::action::ComputePathToPose::Result::TIMEOUT};
  std::string failure_error_msg_{"Timeout"};
};

class DummyFollowPathActionServer : public DummyActionServer<nav2_msgs::action::FollowPath>
{
public:
  explicit DummyFollowPathActionServer(const rclcpp::Node::SharedPtr & node)
  : DummyActionServer(node, "follow_path") {}

protected:
  void updateResultForFailure(
    std::shared_ptr<nav2_msgs::action::FollowPath::Result>
    & result) override
  {
    result->error_code = nav2_msgs::action::FollowPath::Result::NO_VALID_CONTROL;
    result->error_msg = "No valid control";
  }
};

class DummyStopSlowlyActionServer : public DummyActionServer<nav2_msgs::action::StopSlowly>
{
public:
  explicit DummyStopSlowlyActionServer(const rclcpp::Node::SharedPtr & node)
  : DummyActionServer(node, "stop_slowly") {}

protected:
  void updateResultForFailure(
    std::shared_ptr<nav2_msgs::action::StopSlowly::Result>
    & result) override
  {
    result->error_code = nav2_msgs::action::StopSlowly::Result::TF_ERROR;
    result->error_msg = "TF error";
  }
};

class DummyEscapeInfeasibleAreaActionServer
  : public DummyActionServer<nav2_msgs::action::EscapeInfeasibleArea>
{
public:
  explicit DummyEscapeInfeasibleAreaActionServer(const rclcpp::Node::SharedPtr & node)
  : DummyActionServer(node, "escape_infeasible_area") {}

protected:
  void updateResultForFailure(
    std::shared_ptr<nav2_msgs::action::EscapeInfeasibleArea::Result>
    & result) override
  {
    result->error_code = nav2_msgs::action::EscapeInfeasibleArea::Result::NO_ESCAPE_ROUTE;
    result->error_msg = "No escape route";
  }
};

class DummyClearEntireCostmapService : public DummyService<nav2_msgs::srv::ClearEntireCostmap>
{
public:
  explicit DummyClearEntireCostmapService(
    const rclcpp::Node::SharedPtr & node,
    std::string service_name)
  : DummyService(node, service_name) {}

protected:
  void fillResponse(
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Request>/*request*/,
    const std::shared_ptr<nav2_msgs::srv::ClearEntireCostmap::Response> response) override
  {
    response->success = true;
  }
};

class DummyClearCostmapAroundRobotService
  : public DummyService<nav2_msgs::srv::ClearCostmapAroundRobot>
{
public:
  explicit DummyClearCostmapAroundRobotService(
    const rclcpp::Node::SharedPtr & node,
    std::string service_name)
  : DummyService(node, service_name) {}

protected:
  void fillResponse(
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Request>/*request*/,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundRobot::Response> response) override
  {
    response->success = true;
  }
};

class DummyClearCostmapExceptRegionService
  : public DummyService<nav2_msgs::srv::ClearCostmapExceptRegion>
{
public:
  explicit DummyClearCostmapExceptRegionService(
    const rclcpp::Node::SharedPtr & node,
    std::string service_name)
  : DummyService(node, service_name) {}

protected:
  void fillResponse(
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Request>/*request*/,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapExceptRegion::Response> response) override
  {
    response->success = true;
  }
};

class DummyClearCostmapAroundPoseService
  : public DummyService<nav2_msgs::srv::ClearCostmapAroundPose>
{
public:
  explicit DummyClearCostmapAroundPoseService(
    const rclcpp::Node::SharedPtr & node,
    std::string service_name)
  : DummyService(node, service_name) {}

protected:
  void fillResponse(
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundPose::Request>/*request*/,
    const std::shared_ptr<nav2_msgs::srv::ClearCostmapAroundPose::Response> response) override
  {
    response->success = true;
  }
};

class ServerHandler
{
public:
  ServerHandler();
  ~ServerHandler();

  void activate();

  void deactivate();

  bool isActive() const
  {
    return is_active_;
  }

  void reset() const;

public:
  std::unique_ptr<DummyClearEntireCostmapService> clear_local_costmap_server;
  std::unique_ptr<DummyClearEntireCostmapService> clear_global_costmap_server;
  std::unique_ptr<DummyClearCostmapAroundRobotService> clear_costmap_around_robot_server;
  std::unique_ptr<DummyClearCostmapExceptRegionService> clear_costmap_except_region_server;
  std::unique_ptr<DummyClearCostmapAroundPoseService> clear_costmap_around_pose_server;
  std::unique_ptr<DummyService<nav2_msgs::srv::IsPathValid>> validate_path_server;
  std::unique_ptr<DummyComputePathToPoseActionServer> compute_path_to_pose_server;
  std::unique_ptr<DummyFollowPathActionServer> follow_path_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::Spin>> spin_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::Wait>> wait_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::BackUp>> backup_server;
  std::unique_ptr<DummyStopSlowlyActionServer> stop_slowly_server;
  std::unique_ptr<DummyEscapeInfeasibleAreaActionServer> escape_infeasible_area_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::ComputeRoute>> compute_route_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::SmoothPath>> smoother_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::DriveOnHeading>> drive_on_heading_server;
  std::unique_ptr<DummyActionServer<nav2_msgs::action::ComputePathThroughPoses>> ntp_server;

private:
  void spinThread();

  bool is_active_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<std::thread> server_thread_;
};

}  // namespace nav2_system_tests

#endif  //  BEHAVIOR_TREE__SERVER_HANDLER_HPP_
