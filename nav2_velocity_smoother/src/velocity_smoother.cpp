// Copyright (c) 2022 Samsung Research
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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_velocity_smoother/velocity_smoother.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace nav2_velocity_smoother
{

VelocitySmoother::VelocitySmoother(const rclcpp::NodeOptions & options)
: LifecycleNode("velocity_smoother", "", options),
  last_command_time_{0, 0, get_clock()->get_clock_type()}
{
}

VelocitySmoother::~VelocitySmoother()
{
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
}

nav2::CallbackReturn
VelocitySmoother::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Configuring velocity smoother");
  auto node = shared_from_this();

  // Smoothing metadata
  smoothing_frequency_ = node->declare_or_get_parameter(
    "smoothing_frequency", 20.0);
  std::string feedback_type = node->declare_or_get_parameter(
    "feedback", std::string("OPEN_LOOP"));
  scale_velocities_ = node->declare_or_get_parameter("scale_velocities", false);
  force_commands_above_deadband_ =
    node->declare_or_get_parameter("force_commands_above_deadband", false);

  // Kinematics
  max_velocities_ = node->declare_or_get_parameter(
    "max_velocity", std::vector<double>{0.50, 0.0, 2.5});
  min_velocities_ = node->declare_or_get_parameter(
    "min_velocity", std::vector<double>{-0.50, 0.0, -2.5});
  max_accels_ = node->declare_or_get_parameter(
    "max_accel", std::vector<double>{2.5, 0.0, 3.2});
  max_decels_ = node->declare_or_get_parameter(
    "max_decel", std::vector<double>{-2.5, 0.0, -3.2});
  accel_jerks_ = node->declare_or_get_parameter(
    "accel_jerks", std::vector<double>{0.5, 0.0, 0.5});
  decel_jerks_ = node->declare_or_get_parameter(
    "decel_jerks", std::vector<double>{-0.5, 0.0, -0.5});

  // Get feature parameters
  odom_topic_ = node->declare_or_get_parameter("odom_topic", std::string("odom"));
  odom_duration_ = node->declare_or_get_parameter("odom_duration", 0.1);
  odom_smoother_ = std::make_unique<nav2_util::OdomSmoother>(node, odom_duration_, odom_topic_);
  deadband_velocities_ = node->declare_or_get_parameter(
    "deadband_velocity", std::vector<double>{0.0, 0.0, 0.0});
  double velocity_timeout_dbl = node->declare_or_get_parameter("velocity_timeout", 1.0);
  velocity_timeout_ = rclcpp::Duration::from_seconds(velocity_timeout_dbl);

  // Check if parameters are properly set
  size_t size = max_velocities_.size();
  is_6dof_ = (size == 6);

  if ((size != 3 && size != 6) ||
    min_velocities_.size() != size ||
    max_accels_.size() != size ||
    max_decels_.size() != size ||
    accel_jerks_.size() != size ||
    decel_jerks_.size() != size ||
    deadband_velocities_.size() != size)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid setting of kinematic, jerk and/or deadband limits!"
      " All limits must be size of 3 (x, y, theta) or 6 (x, y, z, r, p, y)");
    on_cleanup(state);
    return nav2::CallbackReturn::FAILURE;
  }

  // Current per-step velocity change (acceleration proxy), one per axis
  current_accel_.assign(size, 0.0);

  for (unsigned int i = 0; i != size; i++) {
    if (accel_jerks_[i] < 0.0) {
      RCLCPP_ERROR(get_logger(), "Accel jerk values should be positive.");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (decel_jerks_[i] > 0.0) {
      RCLCPP_ERROR(get_logger(), "Decel jerk values should be negative.");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (max_decels_[i] > 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "Positive values set of deceleration! These should be negative to slow down!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (max_accels_[i] < 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "Negative values set of acceleration! These should be positive to speed up!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (min_velocities_[i] > 0.0) {
      RCLCPP_ERROR(
        get_logger(), "Positive values set of min_velocities! These should be negative!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (max_velocities_[i] < 0.0) {
      RCLCPP_ERROR(
        get_logger(), "Negative values set of max_velocities! These should be positive!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
    if (min_velocities_[i] > max_velocities_[i]) {
      RCLCPP_ERROR(get_logger(), "Min velocities are higher than max velocities!");
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  // Get control type
  if (feedback_type == "OPEN_LOOP") {
    open_loop_ = true;
  } else if (feedback_type == "CLOSED_LOOP") {
    open_loop_ = false;
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.");
    on_cleanup(state);
    return nav2::CallbackReturn::FAILURE;
  }

  // Setup inputs / outputs
  smoothed_cmd_pub_ = std::make_unique<nav2_util::TwistPublisher>(node, "cmd_vel_smoothed");
  cmd_sub_ = std::make_unique<nav2_util::TwistSubscriber>(
    node,
    "cmd_vel",
    std::bind(&VelocitySmoother::inputCommandCallback, this, std::placeholders::_1),
    std::bind(&VelocitySmoother::inputCommandStampedCallback, this, std::placeholders::_1));

  bool use_realtime_priority = node->declare_or_get_parameter("use_realtime_priority", false);
  if (use_realtime_priority) {
    try {
      nav2::setSoftRealTimePriority();
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      on_cleanup(state);
      return nav2::CallbackReturn::FAILURE;
    }
  }

  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
VelocitySmoother::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  smoothed_cmd_pub_->on_activate();
  double timer_duration_ms = 1000.0 / smoothing_frequency_;
  timer_ = this->create_timer(
    std::chrono::milliseconds(static_cast<int>(timer_duration_ms)),
    std::bind(&VelocitySmoother::smootherTimer, this));

  // Add callback for dynamic parameters
  auto node = shared_from_this();
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &VelocitySmoother::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &VelocitySmoother::validateParameterUpdatesCallback,
      this, std::placeholders::_1));

  // create bond connection
  createBond();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
VelocitySmoother::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  if (timer_) {
    timer_->cancel();
    timer_.reset();
  }
  smoothed_cmd_pub_->on_deactivate();

  auto node = shared_from_this();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();

  // destroy bond connection
  destroyBond();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
VelocitySmoother::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  smoothed_cmd_pub_.reset();
  odom_smoother_.reset();
  cmd_sub_.reset();
  return nav2::CallbackReturn::SUCCESS;
}

nav2::CallbackReturn
VelocitySmoother::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2::CallbackReturn::SUCCESS;
}

void VelocitySmoother::inputCommandStampedCallback(
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr & msg)
{
  // If message contains NaN or Inf, ignore
  if (!nav2_util::validateTwist(msg->twist)) {
    RCLCPP_ERROR(get_logger(), "Velocity message contains NaNs or Infs! Ignoring as invalid!");
    return;
  }

  command_ = *msg;
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    last_command_time_ = now();
  } else {
    last_command_time_ = msg->header.stamp;
  }
  received_first_command_ = true;
}

void VelocitySmoother::inputCommandCallback(
  const geometry_msgs::msg::Twist::ConstSharedPtr & msg)
{
  auto twist_stamped = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist_stamped->twist = *msg;
  inputCommandStampedCallback(twist_stamped);
}

// Compute the per-step velocity-change window [v_component_min, v_component_max]
// bounded by both the acceleration limits and, tightened further, the jerk limits
// around the current acceleration proxy accel_curr.
static void computeVelocityComponentBounds(
  const double v_curr, const double v_cmd, const double accel_curr,
  const double decel, const double accel,
  const double decel_jerk, const double accel_jerk,
  const double smoothing_frequency,
  double & v_component_max, double & v_component_min)
{
  double accel_v_component_max, jerk_v_component_max;
  double accel_v_component_min, jerk_v_component_min;

  // Accelerating if magnitude of v_cmd is above magnitude of v_curr
  // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
  // Decelerating otherwise
  if (std::abs(v_cmd) >= std::abs(v_curr) && v_curr * v_cmd >= 0.0) {
    // accelerating
    accel_v_component_max = accel / smoothing_frequency;
    jerk_v_component_max = (accel_curr + accel_jerk) / smoothing_frequency;
    accel_v_component_min = -accel / smoothing_frequency;
    jerk_v_component_min = (accel_curr - accel_jerk) / smoothing_frequency;
  } else {
    // decelerating
    accel_v_component_max = -decel / smoothing_frequency;
    jerk_v_component_max = (accel_curr - decel_jerk) / smoothing_frequency;
    accel_v_component_min = decel / smoothing_frequency;
    jerk_v_component_min = (accel_curr + decel_jerk) / smoothing_frequency;
  }

  v_component_max = std::clamp(jerk_v_component_max, accel_v_component_min, accel_v_component_max);
  v_component_min = std::clamp(jerk_v_component_min, accel_v_component_min, accel_v_component_max);
}

double VelocitySmoother::findEtaConstraint(
  const double v_curr, const double v_cmd,
  const double accel, const double decel,
  const double accel_curr, const double decel_jerk, const double accel_jerk)
{
  // Exploiting vector scaling properties
  double dv = v_cmd - v_curr;

  double v_component_max;
  double v_component_min;
  computeVelocityComponentBounds(
    v_curr, v_cmd, accel_curr, decel, accel, decel_jerk, accel_jerk,
    smoothing_frequency_, v_component_max, v_component_min);

  if (dv > v_component_max) {
    return v_component_max / dv;
  }

  if (dv < v_component_min) {
    return v_component_min / dv;
  }

  return -1.0;
}

double VelocitySmoother::applyConstraints(
  const double v_curr, const double v_cmd,
  const double accel, const double decel, const double eta,
  const double accel_curr, const double decel_jerk, const double accel_jerk)
{
  double dv = v_cmd - v_curr;

  double v_component_max;
  double v_component_min;
  computeVelocityComponentBounds(
    v_curr, v_cmd, accel_curr, decel, accel, decel_jerk, accel_jerk,
    smoothing_frequency_, v_component_max, v_component_min);

  return v_curr + std::clamp(eta * dv, v_component_min, v_component_max);
}

void VelocitySmoother::smootherTimer()
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Wait until the first command is received
  if (!received_first_command_) {
    return;
  }

  auto const delta_time_since_last_command = now() - last_command_time_;

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.frame_id = command_.header.frame_id;
  // Smooth the timestamp of the smoothed message
  // Do not keep the same timestamp of the last command; this causes jerky behavior
  // See https://github.com/ros-navigation/navigation2/issues/5857
  cmd_vel->header.stamp = command_.header.stamp + delta_time_since_last_command;

  // Check for velocity timeout. If nothing received, publish zeros to apply deceleration
  if (delta_time_since_last_command > velocity_timeout_) {
    if (last_cmd_.twist == geometry_msgs::msg::Twist() || stopped_) {
      stopped_ = true;
      return;
    }
    command_ = geometry_msgs::msg::TwistStamped();
    command_.header.stamp = now();
  }

  stopped_ = false;

  // Get current velocity based on feedback type
  if (open_loop_) {
    current_twist_ = last_cmd_;
    // In open loop, the acceleration proxy is updated at the end from the cmd delta.
  } else {
    // Update the per-axis acceleration proxy (per-step velocity delta) before velocity
    auto twist = odom_smoother_->getTwistStamped();
    if (!is_6dof_) {
      current_accel_[0] = twist.twist.linear.x - current_twist_.twist.linear.x;
      current_accel_[1] = twist.twist.linear.y - current_twist_.twist.linear.y;
      current_accel_[2] = twist.twist.angular.z - current_twist_.twist.angular.z;
    } else {
      current_accel_[0] = twist.twist.linear.x - current_twist_.twist.linear.x;
      current_accel_[1] = twist.twist.linear.y - current_twist_.twist.linear.y;
      current_accel_[2] = twist.twist.linear.z - current_twist_.twist.linear.z;
      current_accel_[3] = twist.twist.angular.x - current_twist_.twist.angular.x;
      current_accel_[4] = twist.twist.angular.y - current_twist_.twist.angular.y;
      current_accel_[5] = twist.twist.angular.z - current_twist_.twist.angular.z;
    }
    current_twist_ = twist;
  }

  // Apply absolute velocity restrictions to the command
  if(!is_6dof_) {
    command_.twist.linear.x = std::clamp(
      command_.twist.linear.x, min_velocities_[0],
      max_velocities_[0]);
    command_.twist.linear.y = std::clamp(
      command_.twist.linear.y, min_velocities_[1],
      max_velocities_[1]);
    command_.twist.angular.z = std::clamp(
      command_.twist.angular.z, min_velocities_[2],
      max_velocities_[2]);
  } else {
    command_.twist.linear.x = std::clamp(
      command_.twist.linear.x, min_velocities_[0],
      max_velocities_[0]);
    command_.twist.linear.y = std::clamp(
      command_.twist.linear.y, min_velocities_[1],
      max_velocities_[1]);
    command_.twist.linear.z = std::clamp(
      command_.twist.linear.z, min_velocities_[2],
      max_velocities_[2]);
    command_.twist.angular.x = std::clamp(
      command_.twist.angular.x, min_velocities_[3],
      max_velocities_[3]);
    command_.twist.angular.y = std::clamp(
      command_.twist.angular.y, min_velocities_[4],
      max_velocities_[4]);
    command_.twist.angular.z = std::clamp(
      command_.twist.angular.z, min_velocities_[5],
      max_velocities_[5]);
  }

  // Find if any component is not within the acceleration constraints. If so, store the most
  // significant scale factor to apply to the vector <dvx, dvy, dvw>, eta, to reduce all axes
  // proportionally to follow the same direction, within change of velocity bounds.
  // In case eta reduces another axis out of its own limit, apply accel constraint to guarantee
  // output is within limits, even if it deviates from requested command slightly.
  double eta = 1.0;
  if (scale_velocities_) {
    double curr_eta = -1.0;
    if (!is_6dof_) {
      curr_eta = findEtaConstraint(
        current_twist_.twist.linear.x, command_.twist.linear.x, max_accels_[0], max_decels_[0],
        current_accel_[0], decel_jerks_[0], accel_jerks_[0]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_twist_.twist.linear.y, command_.twist.linear.y, max_accels_[1], max_decels_[1],
        current_accel_[1], decel_jerks_[1], accel_jerks_[1]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_twist_.twist.angular.z, command_.twist.angular.z, max_accels_[2], max_decels_[2],
        current_accel_[2], decel_jerks_[2], accel_jerks_[2]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }
    } else {
      curr_eta = findEtaConstraint(
        current_twist_.twist.linear.x, command_.twist.linear.x, max_accels_[0], max_decels_[0],
        current_accel_[0], decel_jerks_[0], accel_jerks_[0]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_twist_.twist.linear.y, command_.twist.linear.y, max_accels_[1], max_decels_[1],
        current_accel_[1], decel_jerks_[1], accel_jerks_[1]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_twist_.twist.linear.z, command_.twist.linear.z, max_accels_[2], max_decels_[2],
        current_accel_[2], decel_jerks_[2], accel_jerks_[2]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_twist_.twist.angular.x, command_.twist.angular.x, max_accels_[3], max_decels_[3],
        current_accel_[3], decel_jerks_[3], accel_jerks_[3]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_twist_.twist.angular.y, command_.twist.angular.y, max_accels_[4], max_decels_[4],
        current_accel_[4], decel_jerks_[4], accel_jerks_[4]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }

      curr_eta = findEtaConstraint(
        current_twist_.twist.angular.z, command_.twist.angular.z, max_accels_[5], max_decels_[5],
        current_accel_[5], decel_jerks_[5], accel_jerks_[5]);
      if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
        eta = curr_eta;
      }
    }
  }

  if (!is_6dof_) {
    cmd_vel->twist.linear.x = applyConstraints(
      current_twist_.twist.linear.x, command_.twist.linear.x, max_accels_[0], max_decels_[0], eta,
      current_accel_[0], decel_jerks_[0], accel_jerks_[0]);
    cmd_vel->twist.linear.y = applyConstraints(
      current_twist_.twist.linear.y, command_.twist.linear.y, max_accels_[1], max_decels_[1], eta,
      current_accel_[1], decel_jerks_[1], accel_jerks_[1]);
    cmd_vel->twist.angular.z = applyConstraints(
      current_twist_.twist.angular.z, command_.twist.angular.z, max_accels_[2], max_decels_[2], eta,
      current_accel_[2], decel_jerks_[2], accel_jerks_[2]);
  } else {
    cmd_vel->twist.linear.x = applyConstraints(
      current_twist_.twist.linear.x, command_.twist.linear.x, max_accels_[0], max_decels_[0], eta,
      current_accel_[0], decel_jerks_[0], accel_jerks_[0]);
    cmd_vel->twist.linear.y = applyConstraints(
      current_twist_.twist.linear.y, command_.twist.linear.y, max_accels_[1], max_decels_[1], eta,
      current_accel_[1], decel_jerks_[1], accel_jerks_[1]);
    cmd_vel->twist.linear.z = applyConstraints(
      current_twist_.twist.linear.z, command_.twist.linear.z, max_accels_[2], max_decels_[2], eta,
      current_accel_[2], decel_jerks_[2], accel_jerks_[2]);
    cmd_vel->twist.angular.x = applyConstraints(
      current_twist_.twist.angular.x, command_.twist.angular.x, max_accels_[3], max_decels_[3], eta,
      current_accel_[3], decel_jerks_[3], accel_jerks_[3]);
    cmd_vel->twist.angular.y = applyConstraints(
      current_twist_.twist.angular.y, command_.twist.angular.y, max_accels_[4], max_decels_[4], eta,
      current_accel_[4], decel_jerks_[4], accel_jerks_[4]);
    cmd_vel->twist.angular.z = applyConstraints(
      current_twist_.twist.angular.z, command_.twist.angular.z, max_accels_[5], max_decels_[5], eta,
      current_accel_[5], decel_jerks_[5], accel_jerks_[5]);
  }


  // Apply deadband restrictions. When force_commands_above_deadband_ is set, small non-zero
  // commands are snapped up to the deadband magnitude (preserving sign) instead of zeroed,
  // so a commanded motion is not silently dropped below the actuator deadband.
  auto apply_deadband = [this](double v, double deadband) -> double {
      if (fabs(v) <= std::numeric_limits<float>::epsilon()) {
        return v;
      }
      if (fabs(v) < deadband) {
        const double sign = v > 0.0 ? 1.0 : -1.0;
        return force_commands_above_deadband_ ? sign * deadband : 0.0;
      }
      return v;
    };

  if (!is_6dof_) {
    cmd_vel->twist.linear.x = apply_deadband(cmd_vel->twist.linear.x, deadband_velocities_[0]);
    cmd_vel->twist.linear.y = apply_deadband(cmd_vel->twist.linear.y, deadband_velocities_[1]);
    cmd_vel->twist.linear.z = command_.twist.linear.z;
    cmd_vel->twist.angular.x = command_.twist.angular.x;
    cmd_vel->twist.angular.y = command_.twist.angular.y;
    cmd_vel->twist.angular.z = apply_deadband(cmd_vel->twist.angular.z, deadband_velocities_[2]);
  } else {
    cmd_vel->twist.linear.x = apply_deadband(cmd_vel->twist.linear.x, deadband_velocities_[0]);
    cmd_vel->twist.linear.y = apply_deadband(cmd_vel->twist.linear.y, deadband_velocities_[1]);
    cmd_vel->twist.linear.z = apply_deadband(cmd_vel->twist.linear.z, deadband_velocities_[2]);
    cmd_vel->twist.angular.x = apply_deadband(cmd_vel->twist.angular.x, deadband_velocities_[3]);
    cmd_vel->twist.angular.y = apply_deadband(cmd_vel->twist.angular.y, deadband_velocities_[4]);
    cmd_vel->twist.angular.z = apply_deadband(cmd_vel->twist.angular.z, deadband_velocities_[5]);
  }

  // Absolute final clamps to velocity limits, just in case
  if (!is_6dof_) {
    cmd_vel->twist.linear.x =
      std::clamp(cmd_vel->twist.linear.x, min_velocities_[0], max_velocities_[0]);
    cmd_vel->twist.linear.y =
      std::clamp(cmd_vel->twist.linear.y, min_velocities_[1], max_velocities_[1]);
    cmd_vel->twist.angular.z =
      std::clamp(cmd_vel->twist.angular.z, min_velocities_[2], max_velocities_[2]);
  } else {
    cmd_vel->twist.linear.x =
      std::clamp(cmd_vel->twist.linear.x, min_velocities_[0], max_velocities_[0]);
    cmd_vel->twist.linear.y =
      std::clamp(cmd_vel->twist.linear.y, min_velocities_[1], max_velocities_[1]);
    cmd_vel->twist.linear.z =
      std::clamp(cmd_vel->twist.linear.z, min_velocities_[2], max_velocities_[2]);
    cmd_vel->twist.angular.x =
      std::clamp(cmd_vel->twist.angular.x, min_velocities_[3], max_velocities_[3]);
    cmd_vel->twist.angular.y =
      std::clamp(cmd_vel->twist.angular.y, min_velocities_[4], max_velocities_[4]);
    cmd_vel->twist.angular.z =
      std::clamp(cmd_vel->twist.angular.z, min_velocities_[5], max_velocities_[5]);
  }

  // In open loop, update the per-axis acceleration proxy from the change in output command
  if (open_loop_) {
    if (!is_6dof_) {
      current_accel_[0] = cmd_vel->twist.linear.x - last_cmd_.twist.linear.x;
      current_accel_[1] = cmd_vel->twist.linear.y - last_cmd_.twist.linear.y;
      current_accel_[2] = cmd_vel->twist.angular.z - last_cmd_.twist.angular.z;
    } else {
      current_accel_[0] = cmd_vel->twist.linear.x - last_cmd_.twist.linear.x;
      current_accel_[1] = cmd_vel->twist.linear.y - last_cmd_.twist.linear.y;
      current_accel_[2] = cmd_vel->twist.linear.z - last_cmd_.twist.linear.z;
      current_accel_[3] = cmd_vel->twist.angular.x - last_cmd_.twist.angular.x;
      current_accel_[4] = cmd_vel->twist.angular.y - last_cmd_.twist.angular.y;
      current_accel_[5] = cmd_vel->twist.angular.z - last_cmd_.twist.angular.z;
    }
  }
  last_cmd_ = *cmd_vel;

  smoothed_cmd_pub_->publish(std::move(cmd_vel));
}

rcl_interfaces::msg::SetParametersResult VelocitySmoother::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() <= 0.0 && param_name == "smoothing_frequency") {
        RCLCPP_WARN(
          get_logger(), "The value of smoothing_frequency is incorrectly set to %f, "
          "it should be >0. Ignoring parameter update.",
          parameter.as_double());
        result.successful = false;
        break;
      } else if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
          get_logger(), "The value of parameter '%s' is incorrectly set to %f, "
          "it should be >=0. Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
        break;
      }
    } else if (param_type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
      size_t size = is_6dof_ ? 6 : 3;
      if (parameter.as_double_array().size() != size) {
        RCLCPP_WARN(
          get_logger(), "Invalid size of parameter %s. Must be size %ld",
          param_name.c_str(), size);
        result.successful = false;
        break;
      } else if (param_name == "max_velocity" || param_name == "max_accel") {
        for (auto val : parameter.as_double_array()) {
          if (val < 0.0) {
            RCLCPP_WARN(
              get_logger(), "The value of parameter '%s' is incorrectly set to %f, "
              "it should be >=0. Ignoring parameter update.",
              param_name.c_str(), val);
            result.successful = false;
            break;
          }
        }
      } else if (param_name == "min_velocity" || param_name == "max_decel") {
        for (auto val : parameter.as_double_array()) {
          if (val > 0.0) {
            RCLCPP_WARN(
              get_logger(), "The value of parameter '%s' is incorrectly set to %f, "
              "it should be <=0. Ignoring parameter update.",
              param_name.c_str(), val);
            result.successful = false;
            break;
          }
        }
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "feedback") {
        if (parameter.as_string() != "OPEN_LOOP" && parameter.as_string() != "CLOSED_LOOP") {
          RCLCPP_WARN(
            get_logger(),
            "Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP. "
            "Ignoring parameter update.");
          result.successful = false;
          break;
        }
      }
    }
  }
  return result;
}

void VelocitySmoother::updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find('.') != std::string::npos) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == "smoothing_frequency") {
        smoothing_frequency_ = parameter.as_double();
        if (timer_) {
          timer_->cancel();
          timer_.reset();
        }

        double timer_duration_ms = 1000.0 / smoothing_frequency_;
        timer_ = this->create_timer(
          std::chrono::milliseconds(static_cast<int>(timer_duration_ms)),
          std::bind(&VelocitySmoother::smootherTimer, this));
      } else if (param_name == "velocity_timeout") {
        velocity_timeout_ = rclcpp::Duration::from_seconds(parameter.as_double());
      } else if (param_name == "odom_duration") {
        odom_duration_ = parameter.as_double();
        odom_smoother_ =
          std::make_unique<nav2_util::OdomSmoother>(
          shared_from_this(), odom_duration_, odom_topic_);
      }
    } else if (param_type == ParameterType::PARAMETER_DOUBLE_ARRAY) {
      if (param_name == "max_velocity") {
        max_velocities_ = parameter.as_double_array();
      } else if (param_name == "min_velocity") {
        min_velocities_ = parameter.as_double_array();
      } else if (param_name == "max_accel") {
        max_accels_ = parameter.as_double_array();
      } else if (param_name == "max_decel") {
        max_decels_ = parameter.as_double_array();
      } else if (param_name == "deadband_velocity") {
        deadband_velocities_ = parameter.as_double_array();
      }
    } else if (param_type == ParameterType::PARAMETER_STRING) {
      if (param_name == "feedback") {
        if (parameter.as_string() == "OPEN_LOOP") {
          open_loop_ = true;
          odom_smoother_.reset();
        } else if (parameter.as_string() == "CLOSED_LOOP") {
          open_loop_ = false;
          odom_smoother_ =
            std::make_unique<nav2_util::OdomSmoother>(
            shared_from_this(), odom_duration_, odom_topic_);
        }
      }
    }
  }
}

}  // namespace nav2_velocity_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_velocity_smoother::VelocitySmoother)
