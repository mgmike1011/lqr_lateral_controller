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
  this->lqr = std::make_shared<lqr_lateral_controller::LQR>();
  // Previous heading
  prev_phi_des_ = 0.0;
  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  param_.ld_velocity_ratio =
    node.declare_parameter<double>("ld_velocity_ratio", 0.7);  // TODO: Declare more parameters

  RCLCPP_ERROR(logger_, "LQR Lateral controller initialized.");  // TODO: Change ERROR to INFO
}

AckermannLateralCommand LqrLateralController::generateOutputControlCmd(double & target_curvature)
{
  const double tmp_steering = target_curvature; //atan(param_.wheel_base * target_curvature);

  AckermannLateralCommand cmd;
  cmd.stamp = clock_->now();
  cmd.steering_tire_angle = static_cast<float>(tmp_steering); 
  // static_cast<float>(std::min(std::max(tmp_steering, -param_.max_steering_angle), param_.max_steering_angle));

  return cmd;
}

bool LqrLateralController::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

void LqrLateralController::interpolateOrientation(const std::vector<TrajectoryPoint> & trajectory_points,
                                                  const geometry_msgs::msg::Pose & current_pose,
                                                  std::vector<double> & interpolated_orientations)
{
  // Calculate distances to trajectory points
  std::vector<double> distances;
  distances.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points)
  {
    double dx = current_pose.position.x - point.pose.position.x;
    double dy = current_pose.position.y - point.pose.position.y;
    distances.push_back(std::sqrt(dx * dx + dy * dy));
  }

  // Find the closest point to the current position
  auto closest_point_iter = std::min_element(distances.begin(), distances.end());
  size_t closest_point_index = std::distance(distances.begin(), closest_point_iter);

  // Interpolate orientations for the 30 points "in advance"
  size_t num_points_ahead = std::min<size_t>(30, trajectory_points.size() - closest_point_index);
  interpolated_orientations.clear();
  interpolated_orientations.reserve(num_points_ahead);
  for (size_t i = 0; i < num_points_ahead; ++i)
  {
    size_t idx = closest_point_index + i;
    interpolated_orientations.push_back(tf2::getYaw(trajectory_points[idx].pose.orientation));
  }
}

LateralOutput LqrLateralController::run(const InputData & input_data)
{
  current_pose_ = input_data.current_odometry.pose.pose;
  current_vel_ = input_data.current_odometry.twist;
  current_odometry_ = input_data.current_odometry;
  current_steering_ = input_data.current_steering;

  auto output_tp_array_ = motion_utils::convertToTrajectoryPointArray(input_data.current_trajectory);
  const auto closest_idx_result = motion_utils::findNearestIndex(output_tp_array_, current_pose_, 3.0, M_PI_4);
  trajectory_ = output_tp_array_.at(*closest_idx_result);
  

  auto phi = tf2::getYaw(current_odometry_.pose.pose.orientation);
  auto phi_des = tf2::getYaw(output_tp_array_.at(*closest_idx_result).pose.orientation);

  auto rps = (phi_des - prev_phi_des_) / 0.01;
  prev_phi_des_ = phi_des;

  // //
  auto phi_des_rps = (tan(phi_des)*trajectory_.longitudinal_velocity_mps) / 0.274;//3.27; //0.274;

  double x = trajectory_.pose.position.x - current_pose_.position.x;
  double y = trajectory_.pose.position.y - current_pose_.position.y;
  double distanc = std::sqrt(x*x+y*y);

  // Eigen::Vector4d state = Eigen::Vector4d(
  //   // distanc,
  //   trajectory_.pose.position.y - current_pose_.position.y,
  //   trajectory_.lateral_velocity_mps - current_vel_.twist.linear.y, 
  //   phi_des - phi,
  //   rps - current_vel_.twist.angular.z);

  Eigen::Vector4d state = Eigen::Vector4d(
  trajectory_.pose.position.y - current_pose_.position.y,
  trajectory_.lateral_velocity_mps - current_vel_.twist.linear.y, 
  phi_des - phi,
  phi_des_rps - current_vel_.twist.angular.z);


  // heading_rate_rps - prędkość kątowa w osi z - yaw
  // v / r prędkość kątowa V - longitudal prędkośc z ref, cos(yaw) - yaw z referencyjnej, promień 0.274

  double u = lqr->calculate_control_signal(current_vel_.twist.linear.x, state);
  
  RCLCPP_INFO(
    logger_,
    "\n --- Run --- \n"
    "- control signal u: %f \n"
    "- phi: %f \n"
    "- phi_des: %f \n"
    "- heading_rate_rps: %f \n"
    "- innex: %ld \n"
    "- rps: %f \n"
    "- Z: %f \n"
    "- distanc: %f \n"
    "--- --- ---",
    u, phi, phi_des, trajectory_.heading_rate_rps, *closest_idx_result, rps, current_vel_.twist.angular.z, distanc);

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
}

}  // namespace lqr_lateral_controller
