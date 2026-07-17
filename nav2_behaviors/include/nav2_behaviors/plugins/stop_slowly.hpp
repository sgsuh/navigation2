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

#ifndef NAV2_BEHAVIORS__PLUGINS__STOP_SLOWLY_HPP_
#define NAV2_BEHAVIORS__PLUGINS__STOP_SLOWLY_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/stop_slowly.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_behaviors
{

/**
 * @class nav2_behaviors::StopSlowly
 * @brief An action server behavior that decelerates the robot to a stop at a
 * desired deceleration rate, optionally increasing the deceleration if a
 * collision is predicted ahead while stopping.
 */
template<typename ActionT = nav2_msgs::action::StopSlowly>
class StopSlowly : public TimedBehavior<ActionT>
{
  using CostmapInfoType = nav2_core::CostmapInfoType;

public:
  StopSlowly()
  : TimedBehavior<ActionT>(),
    feedback_(std::make_shared<typename ActionT::Feedback>()),
    started_(false),
    max_dist_(0.0),
    desired_decel_(0.0),
    consider_collision_(true),
    controller_frequency_(20.0)
  {
  }

  ~StopSlowly() = default;

  ResultStatus onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
  {
    if (command->desired_deceleration < 0.01) {
      std::string error_msg =
        "Please specify the desired deceleration parameter as a positive float number of at "
        "least 0.01.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT, error_msg};
    }

    if (command->max_distance < 0.05) {
      std::string error_msg =
        "Please specify the max distance parameter as a positive float number of at least 0.05.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::INVALID_INPUT, error_msg};
    }

    max_dist_ = command->max_distance;
    desired_decel_ = command->desired_deceleration;
    consider_collision_ = command->consider_collision.data;

    started_ = false;

    if (!nav2_util::getCurrentPose(
        initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
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
    // get current pose in the global (map) and local (odom) frames
    geometry_msgs::msg::PoseStamped current_pose_map;
    if (!nav2_util::getCurrentPose(
        current_pose_map, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      std::string error_msg = "Current robot pose is not available.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR, error_msg};
    }
    geometry_msgs::msg::PoseStamped current_pose_odom;
    if (!nav2_util::getCurrentPose(
        current_pose_odom, *this->tf_, this->local_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      std::string error_msg = "Current robot pose is not available.";
      RCLCPP_ERROR(this->logger_, "%s", error_msg.c_str());
      return ResultStatus{Status::FAILED, ActionT::Result::TF_ERROR, error_msg};
    }

    double diff_x = initial_pose_.pose.position.x - current_pose_map.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose_map.pose.position.y;
    double distance = std::hypot(diff_x, diff_y);
    feedback_->distance_traveled = distance;

    // Robot exceeded allowable max travel distance
    if (distance > max_dist_) {
      RCLCPP_WARN(this->logger_, "Stopping distance exceeded threshold. Immediate stop triggered.");
      this->stopRobot();
      return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE, ""};
    }

    // get current speed
    geometry_msgs::msg::TwistStamped twist = odom_smoother_->getTwistStamped();
    const double vel_x = twist.twist.linear.x;

    // Robot stopped
    if (std::fabs(vel_x) < 0.01) {
      RCLCPP_INFO(this->logger_, "Robot stopped.");
      this->stopRobot();
      return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE, ""};
    }

    // check collision probability to update safe deceleration rate
    feedback_->avoiding_collision.data = false;
    if (consider_collision_) {
      updateSafeDecel(current_pose_map.pose, current_pose_odom.pose, vel_x);
    }

    const double dt = 1.0 / controller_frequency_;
    const double delta_v = vel_x > 0.0 ? -desired_decel_ * dt : desired_decel_ * dt;

    // update command timestamp
    cmd_twist_.header.stamp = this->clock_->now();
    // populate initial command message
    if (!started_) {
      prev_twist_ = twist;
      cmd_twist_.header.frame_id = this->robot_base_frame_;
      cmd_twist_.twist.linear.y = 0.0;
      cmd_twist_.twist.angular.z = 0.0;
      cmd_twist_.twist.linear.x = vel_x + delta_v;
      started_ = true;
    }

    // Robot overshot zero velocity
    if (vel_x * delta_v > 0.0) {
      RCLCPP_WARN(this->logger_, "Overshoot detected. Stopping.");
      this->stopRobot();
      return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE, ""};
    }

    // compute new control command
    if ((this->clock_->now() - prev_twist_.header.stamp).seconds() > dt) {
      const double new_x_cmd = vel_x + delta_v;
      if (new_x_cmd * vel_x <= 0.0) {
        // velocity command inversion, stop
        this->stopRobot();
        return ResultStatus{Status::SUCCEEDED, ActionT::Result::NONE, ""};
      } else {
        cmd_twist_.twist.linear.x = new_x_cmd;
      }
      prev_twist_ = twist;
    }

    // publish feedback and control command
    feedback_->velocity_command = cmd_twist_.twist.linear.x;
    this->action_server_->publish_feedback(feedback_);
    auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>(cmd_twist_);
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
   * @brief Forward-simulate the deceleration and, if a collision is predicted
   * before the robot stops, increase desired_decel_ so the robot can halt in
   * time (with a safety margin).
   */
  void updateSafeDecel(
    const geometry_msgs::msg::Pose & curr_pose_map,
    const geometry_msgs::msg::Pose & curr_pose_odom,
    double vel_x)
  {
    double delta_v = -(1.0 / controller_frequency_) * desired_decel_;
    if (vel_x < 0.0) {
      delta_v *= -1.0;
    }

    const double map_yaw = tf2::getYaw(curr_pose_map.orientation);
    const double odom_yaw = tf2::getYaw(curr_pose_odom.orientation);
    uint16_t step = 0;

    // simulate forward in time to check for potential collisions
    while (true) {
      step++;
      double projected_vel = vel_x + step * delta_v;
      double dt = (1.0 / controller_frequency_) * step;

      if (projected_vel * vel_x < 0.0) {
        // robot stopped, end simulation
        RCLCPP_DEBUG(
          this->logger_, "Simulation terminated: Safely stopped after %f seconds.", dt);
        break;
      }

      double projected_dist = ((vel_x + projected_vel) / 2.0) * dt;
      geometry_msgs::msg::Pose projected_pose_map = poseFromXYYaw(
        curr_pose_map.position.x + projected_dist * std::cos(map_yaw),
        curr_pose_map.position.y + projected_dist * std::sin(map_yaw),
        map_yaw);
      geometry_msgs::msg::Pose projected_pose_odom = poseFromXYYaw(
        curr_pose_odom.position.x + projected_dist * std::cos(odom_yaw),
        curr_pose_odom.position.y + projected_dist * std::sin(odom_yaw),
        odom_yaw);

      if (!this->local_collision_checker_->isCollisionFree(projected_pose_odom, true) ||
        !this->global_collision_checker_->isCollisionFree(projected_pose_map, true))
      {
        // update desired deceleration so robot can stop before collision
        double dist_to_collision = projected_dist;
        // 2as = v^2 - v0^2
        double feasible_decel =
          std::fabs((0.0 - std::pow(vel_x, 2)) / (2.0 * dist_to_collision));
        // update with safety margin
        desired_decel_ = std::max(desired_decel_, 1.5 * feasible_decel);
        feedback_->avoiding_collision.data = true;
        RCLCPP_DEBUG(
          this->logger_, "Collision estimated %fm, %fs ahead!", dist_to_collision, dt);
        RCLCPP_DEBUG(this->logger_, "Current speed %f", vel_x);
        RCLCPP_DEBUG(this->logger_, "Increasing deceleration rate to %f.", desired_decel_);
        break;
      }
    }
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

    std::string odom_topic;
    nav2::declare_parameter_if_not_declared(
      node, "odom_topic", rclcpp::ParameterValue(std::string("odom")));
    node->get_parameter("odom_topic", odom_topic);
    odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node, 0.3, odom_topic);
  }

  typename ActionT::Feedback::SharedPtr feedback_;

  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  geometry_msgs::msg::TwistStamped cmd_twist_;
  geometry_msgs::msg::TwistStamped prev_twist_;
  bool started_;

  double max_dist_;
  double desired_decel_;
  bool consider_collision_;
  double controller_frequency_;
};

}  // namespace nav2_behaviors

#endif  // NAV2_BEHAVIORS__PLUGINS__STOP_SLOWLY_HPP_
