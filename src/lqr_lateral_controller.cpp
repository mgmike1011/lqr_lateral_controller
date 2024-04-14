// Copyright 2024 Milosz Gajewski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lqr_lateral_controller/lqr_lateral_controller.hpp"

#include <motion_utils/resample/resample.hpp>
#include <motion_utils/trajectory/conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <iostream>
#include <stdexcept>

namespace lqr_lateral_controller
{

LqrLateralController::LqrLateralController(rclcpp::Node & node)
: clock_(node.get_clock()), logger_(node.get_logger().get_child("lqr_lateral_controller_logger"))
{
  RCLCPP_ERROR(logger_, "LQR Lateral controller initialization.");  // TODO: Change ERROR to INFO

  // Controller
  this->lqr_ = std::make_shared<lqr_lateral_controller::LQR>();
  // Parameters
  prev_phi_des_ = 0.0;
  last_nearest_index_ = 0;
  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  param_.converged_steer_rad_ = node.declare_parameter<double>("converged_steer_rad", 0.1);

  RCLCPP_ERROR(logger_, "LQR Lateral controller initialized.");  // TODO: Change ERROR to INFO
}

AckermannLateralCommand LqrLateralController::generateOutputControlCmd(double & target_curvature)
{
  const double tmp_steering = target_curvature; //atan(param_.wheel_base * target_curvature);

  AckermannLateralCommand cmd;
  cmd.stamp = clock_->now();
  cmd.steering_tire_angle = static_cast<float>(std::min(std::max(tmp_steering, -param_.max_steering_angle), param_.max_steering_angle));

  return cmd;
}

bool LqrLateralController::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

bool LqrLateralController::calcIsSteerConverged(const AckermannLateralCommand & cmd)
{
  return std::abs(cmd.steering_tire_angle - current_steering_.steering_tire_angle) < static_cast<float>(param_.converged_steer_rad_);
}

LateralOutput LqrLateralController::run(const InputData & input_data)
{
  current_pose_ = input_data.current_odometry.pose.pose;
  current_vel_ = input_data.current_odometry.twist;
  current_odometry_ = input_data.current_odometry;
  current_steering_ = input_data.current_steering;

  auto output_tp_array_ = motion_utils::convertToTrajectoryPointArray(input_data.current_trajectory);
  const auto closest_idx_result = motion_utils::findNearestIndex(output_tp_array_, current_pose_, 3.0, M_PI_4);
  try
  {
    trajectory_ = output_tp_array_.at(*closest_idx_result);
    last_nearest_index_ = *closest_idx_result;
  }
  catch(const std::out_of_range& e)
  {
    RCLCPP_ERROR(logger_, "Found index of trajectory closest point out of range! Exception: %s", e.what());
    trajectory_ = output_tp_array_.at(last_nearest_index_);
  }

  auto phi = tf2::getYaw(current_odometry_.pose.pose.orientation);
  auto phi_des = tf2::getYaw(trajectory_.pose.orientation);

  auto rps = (phi_des - prev_phi_des_) / 0.01;
  prev_phi_des_ = phi_des;

  Eigen::Vector4d state = Eigen::Vector4d(
    trajectory_.pose.position.y - current_pose_.position.y,
    trajectory_.lateral_velocity_mps - current_vel_.twist.linear.y, 
    phi_des - phi,
    rps - current_vel_.twist.angular.z);

  double u = lqr_->calculate_control_signal(current_vel_.twist.linear.x, state);
  
  RCLCPP_ERROR(
    logger_,
    "\n --- Run --- \n"
    "- control signal u: %f \n"
    "- phi: %f \n"
    "- phi_des: %f \n"
    "- heading_rate_rps: %f \n"
    "- innex: %ld \n"
    "- rps: %f \n"
    "- Z: %f \n"
    "--- --- ---",
    u, phi, phi_des, trajectory_.heading_rate_rps, *closest_idx_result, rps, current_vel_.twist.angular.z); // TODO: Change from ERROR to INFO/DEBUG

  const auto cmd_msg = generateOutputControlCmd(u);

  LateralOutput output;
  output.control_cmd = cmd_msg;
  output.sync_data.is_steer_converged = true;
  output.sync_data.is_steer_converged = calcIsSteerConverged(cmd_msg);

  return output;
}

void LqrLateralController::testObject() const
{
  std::cout << "Hello from LQR lateral controller." << std::endl;
}

}  // namespace lqr_lateral_controller
