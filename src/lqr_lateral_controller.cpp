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

#include <vehicle_info_util/vehicle_info_util.hpp>

#include <iostream>

namespace lqr_lateral_controller
{

LqrLateralController::LqrLateralController(rclcpp::Node & node)
: clock_(node.get_clock()), 
logger_(node.get_logger().get_child("lqr_lateral_controller_logger"))
{
  RCLCPP_ERROR(logger_, "LQR Lateral controller initialization.");  // TODO: Change ERROR to INFO
  // Controller
  this->lqr_ = std::make_unique<LQR>();
 
  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;
  
  // Algorithm Parameters
  param_.ld_velocity_ratio = node.declare_parameter<double>("ld_velocity_ratio", 0.0);  // TODO: Declare more parameters

  RCLCPP_ERROR(logger_, "LQR Lateral controller initialized.");  // TODO: Change ERROR to INFO
}

AckermannLateralCommand LqrLateralController::generateOutputControlCmd(const double& target_curvature)
{
  const double tmp_steering = target_curvature;  // TODO: Change for function - convertCurvatureToSteeringAngle -> atan(param_.wheel_base, target_curvature)
  AckermannLateralCommand cmd;
  cmd.stamp = clock_->now();
  cmd.steering_tire_angle = static_cast<float>(std::min(std::max(tmp_steering, -param_.max_steering_angle), param_.max_steering_angle));

  return cmd;
}

bool LqrLateralController::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

LateralOutput LqrLateralController::run(const InputData & input_data)
{
  current_pose_ = input_data.current_odometry.pose.pose;
  trajectory_ = input_data.current_trajectory;
  current_odometry_ = input_data.current_odometry;
  current_steering_ = input_data.current_steering;
  RCLCPP_ERROR(logger_, "LQR Lateral controller::run.");  // TODO: Delete logging
  // setResampledTrajectory();
  // if (param_.enable_path_smoothing) {
  //   averageFilterTrajectory(*trajectory_resampled_);
  // }

  const auto cmd_msg = generateOutputControlCmd(0.5);

  LateralOutput output;
  output.control_cmd = cmd_msg;
  // output.sync_data.is_steer_converged = calcIsSteerConverged(cmd_msg);

  // calculate predicted trajectory with iterative calculation
  // const auto predicted_trajectory = generatePredictedTrajectory();
  // if (!predicted_trajectory) {
  //   RCLCPP_ERROR(logger_, "Failed to generate predicted trajectory.");
  // } else {
  //   pub_predicted_trajectory_->publish(*predicted_trajectory);
  // }
  return output;
}

int64_t LqrLateralController::testObject(int64_t bar) const
{
  std::cout << "Hello from LQR lateral controller, " << bar << std::endl;
  return bar;
}

}  // namespace lqr_lateral_controller
