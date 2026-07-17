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

#ifndef NAV2_BEHAVIORS__PLUGINS__ESCAPE_INFEASIBLE_AREA_HPP_
#define NAV2_BEHAVIORS__PLUGINS__ESCAPE_INFEASIBLE_AREA_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/escape_infeasible_area.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_behaviors
{

/**
 * @class nav2_behaviors::EscapeInfeasibleArea
 * @brief An action server behavior that drives the robot straight forward or
 * backward out of an infeasible (in-collision / inflated) area, choosing the
 * direction that has a feasible escape route.
 */
template<typename ActionT = nav2_msgs::action::EscapeInfeasibleArea>
class EscapeInfeasibleArea : public TimedBehavior<ActionT>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  EscapeInfeasibleArea()
  : TimedBehavior<ActionT>(),
    feedback_(std::make_shared<typename ActionT::Feedback>()),
    started_(false),
    ongoing_(false),
    move_forward_(false),
    move_dist_(0.0),
    min_dist_(0.0),
    max_dist_(0.0),
    speed_(0.0),
    prioritize_backup_(true),
    avoid_collision_(true),
    collision_avoidance_is_failure_(false)
  {
  }

  ~EscapeInfeasibleArea() = default;

  ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
  {
    if (command->min_distance < 0.0 || command->max_distance < 0.0 ||
      command->min_distance > command->max_distance)
    {
      std::string error_msg =
        "Parameters min/max_distance must be greater than 0, and max_distance should be greater "
        "than min_distance.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT, error_msg};
    }

    if (command->speed < 0.0) {
      std::string error_msg = "Speed must be greater than 0";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT, error_msg};
    }

    min_dist_ = command->min_distance;
    max_dist_ = command->max_distance;
    speed_ = command->speed;
    prioritize_backup_ = command->prioritize_backup.data;
    avoid_collision_ = command->avoid_collision.data;
    collision_avoidance_is_failure_ = command->collision_avoidance_is_failure.data;
    time_allowance_ = command->time_allowance;

    end_time_ = this->clock_->now() + time_allowance_;

    // reset states
    started_ = false;
    if (!nav2_util::getCurrentPose(
        initial_pose_map_, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      std::string error_msg = "Initial robot pose is not available.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR, error_msg};
    }
    if (!nav2_util::getCurrentPose(
        initial_pose_odom_, *this->tf_, this->local_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      std::string error_msg = "Initial robot pose is not available.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR, error_msg};
    }

    return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE, ""};
  }

  ResultStatus onCycleUpdate() override
  {
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && time_allowance_.seconds() > 0.0) {
      this->stopRobot();
      std::string error_msg = "Exceeded time allowance: Exiting escape motion.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::TIMEOUT, error_msg};
    }

    geometry_msgs::msg::PoseStamped current_pose_map;
    if (!nav2_util::getCurrentPose(
        current_pose_map, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      this->stopRobot();
      std::string error_msg = "Current robot pose is not available.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR, error_msg};
    }

    geometry_msgs::msg::PoseStamped current_pose_odom;
    if (!nav2_util::getCurrentPose(
        current_pose_odom, *this->tf_, this->local_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      this->stopRobot();
      std::string error_msg = "Current robot pose is not available.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR, error_msg};
    }

    if (!started_) {
      const double move_forward_dist = simulatePossibleMovement(true);
      const double move_backward_dist = simulatePossibleMovement(false);
      if (move_forward_dist < 0.0 && move_backward_dist < 0.0) {
        this->stopRobot();
        std::string error_msg = "Cannot escape infeasible area: no feasible movement!";
        RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
        return ResultStatus{Status::FAILED, ActionT::Result::NO_ESCAPE_ROUTE, error_msg};
      } else if (move_forward_dist >= 0.0 && move_backward_dist <= 0.0) {
        started_ = true;
        move_forward_ = true;
        move_dist_ = move_forward_dist;
        RCLCPP_INFO(
          this->logger_, "Found possible escape route: moving forward %f meters.", move_dist_);
      } else if (move_forward_dist <= 0.0 && move_backward_dist >= 0.0) {
        started_ = true;
        move_forward_ = false;
        move_dist_ = move_backward_dist;
        RCLCPP_INFO(
          this->logger_, "Found possible escape route: moving backward %f meters.", move_dist_);
      } else {
        started_ = true;
        if (ongoing_) {
          RCLCPP_INFO(this->logger_, "Ongoing escapement: continuing in previous direction...");
          move_dist_ = move_forward_ ? move_forward_dist : move_backward_dist;
        } else {
          move_forward_ = prioritize_backup_ ? false : true;
          move_dist_ = prioritize_backup_ ? move_backward_dist : move_forward_dist;
        }
        RCLCPP_INFO(
          this->logger_, "Found possible escape route: moving %s %f meters.",
          move_forward_ ? "forward" : "backward", move_dist_);
      }
    }
    ongoing_ = true;

    double diff_x = initial_pose_map_.pose.position.x - current_pose_map.pose.position.x;
    double diff_y = initial_pose_map_.pose.position.y - current_pose_map.pose.position.y;
    double distance = std::hypot(diff_x, diff_y);

    feedback_->velocity_command = move_forward_ ? speed_ : -speed_;
    feedback_->distance_traveled = distance;
    this->action_server_->publish_feedback(feedback_);

    auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd_vel->header.stamp = this->clock_->now();
    cmd_vel->header.frame_id = this->robot_base_frame_;
    cmd_vel->twist.linear.y = 0.0;
    cmd_vel->twist.angular.z = 0.0;
    cmd_vel->twist.linear.x = move_forward_ ? speed_ : -speed_;

    if (avoid_collision_ && distance >= min_dist_) {
      // check if edges of footprint are in contact with an obstacle;
      // test is passed otherwise.
      if (!this->local_collision_checker_->isCollisionFree(current_pose_odom.pose, true) ||
        !this->global_collision_checker_->isCollisionFree(current_pose_map.pose, true))
      {
        this->stopRobot();
        std::string error_msg = "Collision detected: Exiting escape behavior!";
        RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
        auto result = collision_avoidance_is_failure_ ? Status::FAILED : Status::SUCCEEDED;
        return ResultStatus{result, ActionT::Result::COLLISION_AHEAD, error_msg};
      }
    }

    // 0.05 is to sufficiently move out of the infeasible area
    if (distance >= std::fabs(move_dist_ + 0.05)) {
      // 1. check if edges of footprint are in contact with an obstacle
      // 2. check if base pose of robot is in obstacle or inflation area
      // if the test passes, planning is now possible (robot escaped to free space)
      if (this->local_collision_checker_->isCollisionFree(current_pose_odom.pose, true) &&
        this->global_collision_checker_->isCollisionFree(current_pose_map.pose, true) &&
        !this->local_collision_checker_->isBasePoseIllegal(current_pose_odom.pose, true) &&
        !this->global_collision_checker_->isBasePoseIllegal(current_pose_map.pose, true))
      {
        this->stopRobot();
        RCLCPP_INFO(this->logger_, "Robot out of infeasible area!");
        ongoing_ = false;
        return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE, ""};
      }
    }

    if (distance >= std::fabs(max_dist_)) {
      this->stopRobot();
      std::string error_msg =
        "Escape behavior performed but robot did not move out of infeasible area!";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::NONE, error_msg};
    }

    this->vel_pub_->publish(std::move(cmd_vel));

    return ResultStatus{Status::RUNNING, ActionT::Result::NONE, ""};
  }

  CostmapInfoType getResourceInfo() override {return CostmapInfoType::BOTH;}

protected:
  /**
   * @brief Build a geometry_msgs::msg::Pose from a 2D position and yaw.
   */
  static geometry_msgs::msg::Pose poseFromXYYaw(double x, double y, double yaw)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.orientation = tf2::toMsg(q);
    return pose;
  }

  /**
   * @brief Forward-simulate straight-line movement in the given direction and
   * return the distance at which the robot first reaches a feasible pose, or
   * -1.0 if no feasible pose is found within [min_dist_, max_dist_].
   */
  double simulatePossibleMovement(const bool forward)
  {
    const double init_map_x = initial_pose_map_.pose.position.x;
    const double init_map_y = initial_pose_map_.pose.position.y;
    const double init_map_yaw = tf2::getYaw(initial_pose_map_.pose.orientation);
    const double init_odom_x = initial_pose_odom_.pose.position.x;
    const double init_odom_y = initial_pose_odom_.pose.position.y;
    const double init_odom_yaw = tf2::getYaw(initial_pose_odom_.pose.orientation);

    const double sign = forward ? 1.0 : -1.0;
    uint16_t step = 0;

    bool fetch_data = true;
    while (true) {
      double projected_dist = min_dist_ + 0.01 * step;  // simulate 1 cm at a time
      if (projected_dist > max_dist_) {
        RCLCPP_WARN(
          this->logger_, "%s movement simulation over: cannot escape infeasible area.",
          forward ? "Forward" : "Reverse");
        break;
      }
      step++;

      geometry_msgs::msg::Pose projected_pose_map = poseFromXYYaw(
        init_map_x + sign * projected_dist * std::cos(init_map_yaw),
        init_map_y + sign * projected_dist * std::sin(init_map_yaw),
        init_map_yaw);
      geometry_msgs::msg::Pose projected_pose_odom = poseFromXYYaw(
        init_odom_x + sign * projected_dist * std::cos(init_odom_yaw),
        init_odom_y + sign * projected_dist * std::sin(init_odom_yaw),
        init_odom_yaw);

      bool collision =
        this->local_collision_checker_->isBasePoseIllegal(projected_pose_odom, fetch_data) ||
        this->global_collision_checker_->isBasePoseIllegal(projected_pose_map, fetch_data) ||
        !this->local_collision_checker_->isCollisionFree(projected_pose_odom, fetch_data) ||
        !this->global_collision_checker_->isCollisionFree(projected_pose_map, fetch_data);
      if (collision) {
        continue;
      } else {
        return projected_dist;
      }
    }

    return -1.0;
  }

  void onConfigure() override
  {
    auto node = this->node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2::declare_parameter_if_not_declared(
      node, "controller_frequency", rclcpp::ParameterValue(20.0));
    node->get_parameter("controller_frequency", controller_frequency_);
  }

  typename ActionT::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_map_;
  geometry_msgs::msg::PoseStamped initial_pose_odom_;
  bool started_;
  bool ongoing_;
  bool move_forward_;
  double move_dist_;

  double min_dist_;
  double max_dist_;
  double speed_;
  bool prioritize_backup_;
  bool avoid_collision_;
  bool collision_avoidance_is_failure_;
  rclcpp::Duration time_allowance_{0, 0};
  rclcpp::Time end_time_{0, 0, RCL_ROS_TIME};
  double controller_frequency_{20.0};
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__ESCAPE_INFEASIBLE_AREA_HPP_
