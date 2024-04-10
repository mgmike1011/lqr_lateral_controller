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
geometry_msgs::msg::Quaternion getQuaternionFromYaw(const double _yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, _yaw);
  return tf2::toMsg(q);
}
LqrLateralController::LqrLateralController(rclcpp::Node & node)
: clock_(node.get_clock()), logger_(node.get_logger().get_child("lqr_lateral_controller_logger"))
{
  RCLCPP_ERROR(logger_, "LQR Lateral controller initialization.");  // TODO: Change ERROR to INFO
  // Controller
  this->lqr_ = std::make_shared<lqr_lateral_controller::LQR>();

  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  param_.ld_velocity_ratio = node.declare_parameter<double>("ld_velocity_ratio", 2.4);  // TODO: Declare more parameters
  param_.resampling_ds = node.declare_parameter<double>("resampling_ds", 0.1);
  param_.enable_path_smoothing = node.declare_parameter<bool>("enable_path_smoothing", false);
  param_.path_filter_moving_ave_num =node.declare_parameter<int64_t>("path_filter_moving_ave_num", 25);
  param_.ld_lateral_error_ratio = node.declare_parameter<double>("ld_lateral_error_ratio", 3.6);
  param_.ld_curvature_ratio = node.declare_parameter<double>("ld_curvature_ratio", 120.0);
  param_.long_ld_lateral_error_threshold = node.declare_parameter<double>("long_ld_lateral_error_threshold", 0.5);
  param_.min_lookahead_distance = node.declare_parameter<double>("min_lookahead_distance", 4.35);
  param_.max_lookahead_distance = node.declare_parameter<double>("max_lookahead_distance", 15.0);
  param_.reverse_min_lookahead_distance = node.declare_parameter<double>("reverse_min_lookahead_distance", 7.0);
  param_.converged_steer_rad_ = node.declare_parameter<double>("converged_steer_rad", 0.1);
  param_.prediction_ds = node.declare_parameter<double>("prediction_ds", 0.3);
  param_.prediction_distance_length = node.declare_parameter<double>("prediction_distance_length", 21.0);
  param_.curvature_calculation_distance = node.declare_parameter<double>("curvature_calculation_distance", 4.0);

  RCLCPP_ERROR(logger_, "LQR Lateral controller initialized.");  // TODO: Change ERROR to INFO
}

AckermannLateralCommand LqrLateralController::generateOutputControlCmd()
{
  const auto tmp_steering = calcTargetCurvature(true, current_odometry_.pose.pose);
  // target_curvature += 0;  // TODO: Change for function - convertCurvatureToSteeringAngle -> // atan(param_.wheel_base, target_curvature)

  AckermannLateralCommand output_cmd;
  if (tmp_steering)
  {
    output_cmd = generateCtrlCmdMsg(tmp_steering->curvature);
    prev_cmd_ = boost::optional<AckermannLateralCommand>(output_cmd);
  }
  else
  {
    if (prev_cmd_) 
    {
      output_cmd = *prev_cmd_;
    } 
    else 
    {
      output_cmd = generateCtrlCmdMsg(0.0);
    }
  }
  // cmd.stamp = clock_->now();
  // cmd.steering_tire_angle = static_cast<float>(
  // std::min(std::max(tmp_steering, -param_.max_steering_angle), param_.max_steering_angle);
  // cmd.steering_tire_angle = 1.0; // TODO: Podłączenie sterownika
  return output_cmd;
}

bool LqrLateralController::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

LateralOutput LqrLateralController::run(const InputData & input_data)
{
  current_pose_ = input_data.current_odometry.pose.pose;
  current_vel_ = input_data.current_odometry.twist;
  trajectory_ = input_data.current_trajectory;
  trajectory_point_ = input_data.current_trajectory.points[0];
  current_odometry_ = input_data.current_odometry;
  current_steering_ = input_data.current_steering;

  setResampledTrajectory();
  if (param_.enable_path_smoothing) 
  {
    averageFilterTrajectory(*trajectory_resampled_);
  }
// --- Od Natalii
  auto phi = tf2::getYaw(current_odometry_.pose.pose.orientation);  // tan(current_vel_.linear.y/current_vel_.linear.x)
  auto phi_des = tf2::getYaw(trajectory_point_.pose.orientation);

  Eigen::Vector4d state = Eigen::Vector4d(
    trajectory_point_.pose.position.y - current_pose_.position.y,
    trajectory_point_.lateral_velocity_mps - current_vel_.twist.linear.y, phi_des - phi,
    trajectory_point_.heading_rate_rps - current_vel_.twist.angular.y);

  double u = lqr_->calculate_control_signal(current_vel_.twist.linear.x, trajectory_point_.heading_rate_rps, state);
  if (isnan(u)) 
  {
    RCLCPP_ERROR(logger_, "Control u is nan: %f. Change to 0.0.", u);  // TODO: Change tform ERROR TO INFO
    u = 0.0;
  }

  RCLCPP_ERROR(logger_, "Control: %f", u);  // TODO: Delete logging
  // std::cout << "control signal" << u << std::endl;
  // --------------------------------
  const auto cmd_msg = generateOutputControlCmd();

  LateralOutput output;
  output.control_cmd = cmd_msg;
  output.sync_data.is_steer_converged = calcIsSteerConverged(cmd_msg);

  // calculate predicted trajectory with iterative calculation
  // const auto predicted_trajectory = generatePredictedTrajectory();
  // if (!predicted_trajectory) 
  // {
  //   RCLCPP_ERROR(logger_, "Failed to generate predicted trajectory.");
  // } 
  // else 
  // {
  //   pub_predicted_trajectory_->publish(*predicted_trajectory);
  // }
  return output;
}

bool LqrLateralController::calcIsSteerConverged(const AckermannLateralCommand & cmd)
{
  return std::abs(cmd.steering_tire_angle - current_steering_.steering_tire_angle) < static_cast<float>(param_.converged_steer_rad_);
}

void LqrLateralController::testObject() const
{
  std::cout << "Hello from LQR lateral controller--- " << std::endl;
}

void LqrLateralController::setResampledTrajectory()
{
  // Interpolate with constant interval distance.
  std::vector<double> out_arclength;
  const auto input_tp_array = motion_utils::convertToTrajectoryPointArray(trajectory_);
  const auto traj_length = motion_utils::calcArcLength(input_tp_array);
  for (double s = 0; s < traj_length; s += param_.resampling_ds)
  {
    out_arclength.push_back(s);
  }
  trajectory_resampled_ = std::make_shared<autoware_auto_planning_msgs::msg::Trajectory>(motion_utils::resampleTrajectory(motion_utils::convertToTrajectory(input_tp_array), out_arclength));
  trajectory_resampled_->points.back() = trajectory_.points.back();
  trajectory_resampled_->header = trajectory_.header;
  output_tp_array_ = motion_utils::convertToTrajectoryPointArray(*trajectory_resampled_);
}

AckermannLateralCommand LqrLateralController::generateCtrlCmdMsg(const double target_curvature)
{
  const double tmp_steering = atan(param_.wheel_base * target_curvature);
  AckermannLateralCommand cmd;
  cmd.stamp = clock_->now();
  cmd.steering_tire_angle = static_cast<float>(std::min(std::max(tmp_steering, -param_.max_steering_angle), param_.max_steering_angle));

  return cmd;
}

void LqrLateralController::averageFilterTrajectory(autoware_auto_planning_msgs::msg::Trajectory & u)
{
  if (static_cast<int>(u.points.size()) <= 2 * param_.path_filter_moving_ave_num)
  {
    RCLCPP_ERROR(logger_, "Cannot smooth path! Trajectory size is too low!");
    return;
  }

  autoware_auto_planning_msgs::msg::Trajectory filtered_trajectory(u);

  for (int64_t i = 0; i < static_cast<int64_t>(u.points.size()); ++i)
  {
    TrajectoryPoint tmp{};
    int64_t num_tmp = param_.path_filter_moving_ave_num;
    int64_t count = 0;
    double yaw = 0.0;
    if (i - num_tmp < 0) 
    {
      num_tmp = i;
    }
    if (i + num_tmp > static_cast<int64_t>(u.points.size()) - 1) 
    {
      num_tmp = static_cast<int64_t>(u.points.size()) - i - 1;
    }
    for (int64_t j = -num_tmp; j <= num_tmp; ++j) 
    {
      const auto & p = u.points.at(static_cast<size_t>(i + j));

      tmp.pose.position.x += p.pose.position.x;
      tmp.pose.position.y += p.pose.position.y;
      tmp.pose.position.z += p.pose.position.z;
      tmp.longitudinal_velocity_mps += p.longitudinal_velocity_mps;
      tmp.acceleration_mps2 += p.acceleration_mps2;
      tmp.front_wheel_angle_rad += p.front_wheel_angle_rad;
      tmp.heading_rate_rps += p.heading_rate_rps;
      yaw += tf2::getYaw(p.pose.orientation);
      tmp.lateral_velocity_mps += p.lateral_velocity_mps;
      tmp.rear_wheel_angle_rad += p.rear_wheel_angle_rad;
      ++count;
    }
    auto & p = filtered_trajectory.points.at(static_cast<size_t>(i));

    p.pose.position.x = tmp.pose.position.x / count;
    p.pose.position.y = tmp.pose.position.y / count;
    p.pose.position.z = tmp.pose.position.z / count;
    p.longitudinal_velocity_mps = tmp.longitudinal_velocity_mps / count;
    p.acceleration_mps2 = tmp.acceleration_mps2 / count;
    p.front_wheel_angle_rad = tmp.front_wheel_angle_rad / count;
    p.heading_rate_rps = tmp.heading_rate_rps / count;
    p.lateral_velocity_mps = tmp.lateral_velocity_mps / count;
    p.rear_wheel_angle_rad = tmp.rear_wheel_angle_rad / count;
    p.pose.orientation = getQuaternionFromYaw(yaw / count);
  }
  trajectory_resampled_ = std::make_shared<Trajectory>(filtered_trajectory);
}

double LqrLateralController::calcCurvature(const size_t closest_idx)
{
  // Calculate current curvature
  const size_t idx_dist = static_cast<size_t>(std::max(static_cast<int>((param_.curvature_calculation_distance) / param_.resampling_ds), 1));

  // Find the points in trajectory to calculate curvature
  size_t next_idx = trajectory_resampled_->points.size() - 1;
  size_t prev_idx = 0;

  if (static_cast<size_t>(closest_idx) >= idx_dist) 
  {
    prev_idx = closest_idx - idx_dist;
  }
  else
  {
    // return zero curvature when backward distance is not long enough in the trajectory
    return 0.0;
  }

  if (trajectory_resampled_->points.size() - 1 >= closest_idx + idx_dist)
  {
    next_idx = closest_idx + idx_dist;
  }
  else
  {
    // return zero curvature when forward distance is not long enough in the trajectory
    return 0.0;
  }
  // TODO(k.sugahara): shift the center point of the curvature calculation to allow sufficient
  // distance, because if sufficient distance cannot be obtained in front or behind, the curvature
  // will be zero in the current implementation.

  // Calculate curvature assuming the trajectory points interval is constant
  double current_curvature = 0.0;

  try 
  {
    current_curvature = tier4_autoware_utils::calcCurvature(
      tier4_autoware_utils::getPoint(trajectory_resampled_->points.at(prev_idx)),
      tier4_autoware_utils::getPoint(trajectory_resampled_->points.at(closest_idx)),
      tier4_autoware_utils::getPoint(trajectory_resampled_->points.at(next_idx)));
  } 
  catch (std::exception const & e)
  {
    // ...code that handles the error...
    RCLCPP_WARN(rclcpp::get_logger("lqr"), "%s", e.what());
    current_curvature = 0.0;
  }
  return current_curvature;
}

boost::optional<PpOutput> LqrLateralController::calcTargetCurvature(bool is_control_output, geometry_msgs::msg::Pose pose)
{
  // Ignore invalid trajectory
  if (trajectory_resampled_->points.size() < 3) 
  {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "received path size is < 3, ignored");
    return {};
  }

  // Calculate target point for velocity/acceleration

  const auto closest_idx_result = motion_utils::findNearestIndex(output_tp_array_, pose, 3.0, M_PI_4);
  if (!closest_idx_result) 
  {
    RCLCPP_ERROR(logger_, "cannot find closest waypoint");
    return {};
  }

  const double target_vel = trajectory_resampled_->points.at(*closest_idx_result).longitudinal_velocity_mps;

  // calculate the lateral error

  const double lateral_error = motion_utils::calcLateralOffset(trajectory_resampled_->points, pose.position);

  // calculate the current curvature

  const double current_curvature = calcCurvature(*closest_idx_result);

  // Calculate lookahead distance

  const bool is_reverse = (target_vel < 0);
  const double min_lookahead_distance = is_reverse ? param_.reverse_min_lookahead_distance : param_.min_lookahead_distance;
  double lookahead_distance = min_lookahead_distance;
  if (is_control_output) 
  {
    lookahead_distance = calcLookaheadDistance(lateral_error, current_curvature, current_odometry_.twist.twist.linear.x, min_lookahead_distance, is_control_output);
  }
  else
  {
    lookahead_distance = calcLookaheadDistance(lateral_error, current_curvature, target_vel, min_lookahead_distance, is_control_output);
  }

  // Set PurePursuit data
  pure_pursuit_->setCurrentPose(pose);  // TODO:
  pure_pursuit_->setWaypoints(planning_utils::extractPoses(*trajectory_resampled_)); //TODO: 
  pure_pursuit_->setLookaheadDistance(lookahead_distance); // TODO:

  // Run PurePursuit
  const auto pure_pursuit_result = pure_pursuit_->run();  // TODO:
  if (!pure_pursuit_result.first)  // TODO:
  {
    return {};
  }

  const auto kappa = pure_pursuit_result.second;  // TODO:

  // // Set debug data
  // if (is_control_output) 
  // {
  //   debug_data_.next_target = pure_pursuit_->getLocationOfNextTarget();  // TODO:
  // }
  PpOutput output{};
  output.curvature = kappa;
  if (!is_control_output) 
  {
    output.velocity = current_odometry_.twist.twist.linear.x;
  }
  else
  {
    output.velocity = target_vel;
  }

  return output;
}

}  // namespace lqr_lateral_controller
