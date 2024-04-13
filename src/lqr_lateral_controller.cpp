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
  RCLCPP_ERROR(logger_, "LQR Lateral controller initialization."); 
  //  RCLCPP_ERROR(logger_, "TEEEST trajectory_ %.4f",trajectory_.pose.position.y);
   // TODO: Change ERROR to INFO
  // Controller
  // this->lqr_ = std::make_unique<LQR>();
 
  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;
  subscription_ = node.create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/racing_planner/trajectory", 10, std::bind(&LqrLateralController::get_ref_trajectory, this,std::placeholders::_1));
  
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
void LqrLateralController::get_ref_trajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr  msg ){
 RCLCPP_ERROR(logger_,"Message przed przypisaniem %.4f ",trajectory_.pose.position.y);
  RCLCPP_ERROR(logger_,"Message %d ",msg->points.back().time_from_start.sec);
  RCLCPP_ERROR(logger_,"Message %.4f ",msg->points.back().pose.position.y);
  trajectory_=msg->points[0];
  RCLCPP_ERROR(logger_,"Message after przypisanie %.4f ",trajectory_.pose.position.y);
}

LateralOutput LqrLateralController::run(const InputData & input_data)
{
  current_pose_ = input_data.current_odometry.pose.pose;
  current_vel_ = input_data.current_odometry.twist;

  trajectory_ = input_data.current_trajectory.points[0];
  current_odometry_ = input_data.current_odometry;
  current_steering_ = input_data.current_steering;
  RCLCPP_ERROR(logger_, "LQR Lateral controller::run.");  // TODO: Delete logging
  // setResampledTrajectory();
  // if (param_.enable_path_smoothing) {
  //   averageFilterTrajectory(*trajectory_resampled_);
  // }


  auto phi = tf2::getYaw(current_odometry_.pose.pose.orientation);//tan(current_vel_.linear.y/current_vel_.linear.x)
  auto phi_des=tf2::getYaw(trajectory_.pose.orientation);

  Eigen::Vector4d state =Eigen::Vector4d(trajectory_.pose.position.y-current_pose_.position.y,trajectory_.lateral_velocity_mps-current_vel_.twist.linear.y,phi_des-phi,trajectory_.heading_rate_rps-current_vel_.twist.angular.z);

  RCLCPP_ERROR(logger_,"time from start, %d",trajectory_.time_from_start.sec);
  RCLCPP_ERROR(logger_,"v_x, %.4f",phi_des);
  RCLCPP_ERROR(logger_, "e1,e1_dot,e2,e2_dot: %.4f, %.4f, %.4f, %.4f",trajectory_.pose.position.y,trajectory_.lateral_velocity_mps,phi_des,trajectory_.heading_rate_rps);

 
  
  double u = lqr->calculate_control_signal(current_vel_.twist.linear.x,state); 
  RCLCPP_ERROR(logger_,"x_state, %.4f,%.4f,%.4f,%.4f",lqr->x_state_(0),lqr->x_state_(1),lqr->x_state_(2),lqr->x_state_(3));
  RCLCPP_ERROR(logger_,"K_, %.4f,%.4f,%.4f,%.4f",lqr->K_(0),lqr->K_(1),lqr->K_(2),lqr->K_(3));
  RCLCPP_ERROR(logger_,"u, %.4f",u);

  std::cout<<"control signal"<<u<<std::endl;
  const auto cmd_msg = generateOutputControlCmd(u);

  LateralOutput output;
  output.control_cmd = cmd_msg;
  output.sync_data.is_steer_converged = true;//calcIsSteerConverged(cmd_msg);

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
  std::cout << "Hello from LQR lateral controller--- "<< std::endl;

  // Eigen::Vector4d state =Eigen::Vector4d(0.1-0.5,0.05-0.1,1-5,0.05-0.1);
  // std::cout<<"test"<<std::endl;
  // auto u = lqr_->calculate_control_signal(5.0,0.1,state); 
  // std::cout<<u<<std::endl;

  

}

}  // namespace lqr_lateral_controller
