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

namespace lqr_lateral_controller
{

LqrLateralController::LqrLateralController(rclcpp::Node & node)
: clock_(node.get_clock()), logger_(node.get_logger().get_child("lqr_lateral_controller_logger"))
{
  RCLCPP_ERROR(logger_, "LQR Lateral controller initialization.");  // TODO: Change ERROR to INFO
  // Controller
  // this->lqr_ = std::make_unique<LQR>();

  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  param_.ld_velocity_ratio =
    node.declare_parameter<double>("ld_velocity_ratio", 0.0);  // TODO: Declare more parameters

  RCLCPP_ERROR(logger_, "LQR Lateral controller initialized.");  // TODO: Change ERROR to INFO
}

AckermannLateralCommand LqrLateralController::generateOutputControlCmd(
  const double & target_curvature)
{
  const double tmp_steering =
    target_curvature;  // TODO: Change for function - convertCurvatureToSteeringAngle ->
                       // atan(param_.wheel_base, target_curvature)

  AckermannLateralCommand cmd;
  cmd.stamp = clock_->now();
  cmd.steering_tire_angle = static_cast<float>(
    std::min(std::max(tmp_steering, -param_.max_steering_angle), param_.max_steering_angle));

  return cmd;
}

bool LqrLateralController::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

LateralOutput LqrLateralController::run(const InputData & input_data)
{
  current_pose_ = input_data.current_odometry.pose.pose;  // Obecna pozycja
  current_vel_ = input_data.current_odometry.twist;
  current_odometry_ = input_data.current_odometry;
  current_steering_ = input_data.current_steering;

  // trajectory_ = input_data.current_trajectory.points[0];  // trajectory - cała trajektoria -
  // trzeba znaleźć najbliższy pkt trajectory_ : autoware_auto_planning_msgs::msg::Trajectory

  // ------------------------------

  auto output_tp_array_ =
    motion_utils::convertToTrajectoryPointArray(input_data.current_trajectory);
  const auto closest_idx_result =
    motion_utils::findNearestIndex(output_tp_array_, current_pose_, 3.0, M_PI_4);
  trajectory_ = output_tp_array_[*closest_idx_result];
  RCLCPP_ERROR(logger_, "CLOSEST index: %ld", *closest_idx_result);
  //  RCLCPP_ERROR(logger_, "X: %f", output_tp_array_[*closest_idx_result].pose.position.x);
  // --------------------------------
  // RCLCPP_ERROR(logger_, "LQR Lateral controller::run.");  // TODO: Delete logging
  // setResampledTrajectory();
  // if (param_.enable_path_smoothing) {
  //   averageFilterTrajectory(*trajectory_resampled_);
  // }

  auto phi = tf2::getYaw(
    current_odometry_.pose.pose.orientation);  // tan(current_vel_.linear.y/current_vel_.linear.x)
  auto phi_des = tf2::getYaw(output_tp_array_[*closest_idx_result].pose.orientation);

  Eigen::Vector4d state = Eigen::Vector4d(
    trajectory_.pose.position.y - current_pose_.position.y,
    trajectory_.lateral_velocity_mps - current_vel_.twist.linear.y, phi_des - phi,
    trajectory_.heading_rate_rps - current_vel_.twist.angular.y);
  double u = lqr->calculate_control_signal(current_vel_.twist.linear.x, trajectory_.heading_rate_rps, state);
  RCLCPP_ERROR(logger_, "U: %f", u);
  if (isnan(u)) 
  {
    u = 0.0;
  }
  RCLCPP_ERROR(logger_, "U: %f", u);
  // RCLCPP_ERROR(
  //   logger_,
  //   "--- Run --- \n"
  //   "- control signal u: %f \n"
  //   "- phi: %f \n"
  //   "- phi_des: %f \n"
  //   "- phi_des: %f \n"
  //   "--- --- ---",
  //   u, , phi, phi_des);  // << std::endl;

  // RCLCPP_ERROR(logger_, "- control signal u: %f", u); //<< u << std::endl;
  // RCLCPP_ERROR(logger_, "- state: %f", state[0]);//<< state[0] << " " << state[1] << " " <<
  // state[2] << " " << state[3] << std::endl; RCLCPP_ERROR(logger_, "- phi: %f", phi);//<< phi <<
  // std::endl; RCLCPP_ERROR(logger_, "- phi_des: %f", phi_des); //<< phi_des << std::endl;
  // RCLCPP_ERROR(logger_, "--- --- ---");//<< std::endl;
  const auto cmd_msg = generateOutputControlCmd(u);

  LateralOutput output;
  output.control_cmd = cmd_msg;
  output.sync_data.is_steer_converged = true;
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

void LqrLateralController::testObject() const
{
  std::cout << "Hello from LQR lateral controller--- " << std::endl;

  // Eigen::Vector4d state =Eigen::Vector4d(0.1-0.5,0.05-0.1,1-5,0.05-0.1);
  // std::cout<<"test"<<std::endl;
  // auto u = lqr_->calculate_control_signal(5.0,0.1,state);
  // std::cout<<u<<std::endl;
}

}  // namespace lqr_lateral_controller
