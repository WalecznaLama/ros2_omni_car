// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Enrique Fernández
 */

#include "omni_drive_controller/odometry.hpp"

namespace omni_drive_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  wheel_radius_(0.0),
  front_left_wheel_old_pos_(0.0),
  front_right_wheel_old_pos_(0.0),
  rear_left_wheel_old_pos_(0.0),
  rear_right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_x_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size) {}

void Odometry::init(const rclcpp::Time & time) {
  // Reset accumulators and timestamp:
  resetAccumulators();
  timestamp_ = time;
}

bool Odometry::update(double front_left_pos, double front_right_pos, double rear_left_pos, double rear_right_pos, const rclcpp::Time & time) {
  // We cannot estimate the speed with very small time intervals:
  const double dt = time.seconds() - timestamp_.seconds();
  if (dt < 0.0001) return false;  // Interval too small to integrate with
  
  // Get current wheel joint positions:
  const double front_left_wheel_cur_pos = front_left_pos * wheel_radius_;
  const double front_right_wheel_cur_pos = front_right_pos * wheel_radius_;
  const double rear_left_wheel_cur_pos = rear_left_pos * wheel_radius_;
  const double rear_right_wheel_cur_pos = rear_right_pos * wheel_radius_;

  // Estimate velocity of wheels using old and current position:
  const double front_left_wheel_est_vel = front_left_wheel_cur_pos - front_left_wheel_old_pos_;
  const double front_right_wheel_est_vel = front_right_wheel_cur_pos - front_right_wheel_old_pos_;
  const double rear_left_wheel_est_vel = rear_left_wheel_cur_pos - rear_left_wheel_old_pos_;
  const double rear_right_wheel_est_vel = rear_right_wheel_cur_pos - rear_right_wheel_old_pos_;

  // Update old position with current:
  front_left_wheel_old_pos_ = front_left_wheel_cur_pos;
  front_right_wheel_old_pos_ = front_right_wheel_cur_pos;
  rear_left_wheel_old_pos_ = rear_left_wheel_cur_pos;
  rear_right_wheel_old_pos_ = rear_right_wheel_cur_pos;

  updateFromVelocity(front_left_wheel_est_vel, front_right_wheel_est_vel, rear_left_wheel_est_vel, rear_right_wheel_est_vel, time);

  return true;
}

bool Odometry::updateFromVelocity(double front_left_vel, double front_right_vel, double rear_left_vel, double rear_right_vel, const rclcpp::Time & time) {
  const double dt = time.seconds() - timestamp_.seconds();

  // Compute linear and angular: // TODO
  const double linear_x = 0.;
  const double linear_y = 0.;
  const double angular = 0.;

  // Integrate odometry:
  integrateExact(linear_x, linear_y, angular);

  timestamp_ = time;

  // Estimate speeds using a rolling mean to filter them out:
  linear_x_accumulator_.accumulate(linear_x / dt);
  linear_y_accumulator_.accumulate(linear_y / dt);
  angular_accumulator_.accumulate(angular / dt);

  linear_x_ = linear_x_accumulator_.getRollingMean();
  linear_y_ = linear_y_accumulator_.getRollingMean();
  angular_ = angular_accumulator_.getRollingMean();

  return true;
}

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time) {
  /// Save last linear and angular velocity:
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_ = angular;

  /// Integrate odometry:
  const double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  integrateExact(linear_x_ * dt, linear_y_ * dt, angular * dt);
}

void Odometry::resetOdometry() {
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(double wheel_separation, double wheel_track, double wheel_radius) {
  wheel_separation_ = wheel_separation;
  wheel_track_ = wheel_track;
  wheel_radius_ = wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size) {
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

void Odometry::integrateExact(double linear_x, double linear_y, double angular) {
  heading_ += angular;
  x_ += linear_x;
  y_ += linear_y;
}

void Odometry::resetAccumulators() {
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace omni_drive_controller
