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
  time_prev_=0.0;
  last_nearest_index_ = 0;
  y_des_prev_ = 0.0;
  y_prev_ = 0.0;
  u_prev_ = 0.0;
  curvature_= 0.0;
  e_yLeI_ = 0.0;
  y_dot_des = 0.0;
  y_dot = 0.0;
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
  const auto closest_idx_result = motion_utils::findNearestIndex(output_tp_array_, current_pose_, 5.0, M_PI);

  try
  {
    trajectory_ = output_tp_array_.at(*closest_idx_result);
    if((last_nearest_index_>0) && (*closest_idx_result != last_nearest_index_)){
      // current_curvature = tier4_autoware_utils::calcCurvature(
      //   tier4_autoware_utils::getPoint(output_tp_array_.at(last_nearest_index_)),
      //   tier4_autoware_utils::getPoint(output_tp_array_.at(*closest_idx_result)),
      //   tier4_autoware_utils::getPoint(output_tp_array_.at(*closest_idx_result +1)));
    auto p1 = tier4_autoware_utils::getPoint(output_tp_array_.at(last_nearest_index_));
    auto p2 = tier4_autoware_utils::getPoint(output_tp_array_.at(*closest_idx_result));
    auto p3 = tier4_autoware_utils::getPoint(output_tp_array_.at(*closest_idx_result +1));

    const double denominator =
    tier4_autoware_utils::calcDistance2d(p1, p2) * tier4_autoware_utils::calcDistance2d(p2, p3) * tier4_autoware_utils::calcDistance2d(p3, p1);

    if (std::fabs(denominator) < 1e-10){

      curvature_=curvature_;

      }else{
    curvature_ = 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / denominator;
    
    }

    }


    last_nearest_index_ = *closest_idx_result;
  }
  catch(const std::out_of_range& e)
  {
    RCLCPP_ERROR(logger_, "Found index of trajectory closest point out of range! Exception: %s", e.what());
    trajectory_ = output_tp_array_.at(last_nearest_index_);
  }

  rclcpp::Time timestamp = input_data.current_trajectory.header.stamp;
  double time = timestamp.seconds() +timestamp.nanoseconds()/1e9;
  double tp = time-time_prev_;
  time_prev_ = time;

  auto orient_cur_message = current_odometry_.pose.pose.orientation;

  auto orient_des_message = trajectory_.pose.orientation;

  Eigen::Quaterniond orient_cur(orient_cur_message.w,orient_cur_message.x,orient_cur_message.y,orient_cur_message.z);
  Eigen::Quaterniond orient_des(orient_des_message.w,orient_des_message.x,orient_des_message.y,orient_des_message.z);

  Eigen::Quaterniond e_quat = orient_cur*orient_des.inverse();



  geometry_msgs::msg::Quaternion quaternion_msg;
  quaternion_msg.w = e_quat.w();
  quaternion_msg.x = e_quat.x();
  quaternion_msg.y = e_quat.y();
  quaternion_msg.z = e_quat.z();

  auto error_orient = tf2::getYaw(quaternion_msg);


 
  auto phi = tf2::getYaw(current_odometry_.pose.pose.orientation);
  auto phi_des = tf2::getYaw(trajectory_.pose.orientation);

  //  if (phi <-1.5) {

  //   RCLCPP_ERROR(logger_,"WARUNEK PHIII phi_diff %f ", phi);

  //     auto diff = M_PI + phi;

  //     phi = M_PI ;

  //   RCLCPP_ERROR(logger_,"AAAAAAAAAAAAAAA WARUNEK PHIII diff %f ", diff);


  // }



  // if((phi_des>2.8)&& (phi<0)){
  //   double val = M_PI- abs(phi);
  //   phi = M_PI + abs(val); 
  // }

  // if((phi_des<0.0)&& (phi>2.8)){
  //   double val = M_PI-phi;
  //   phi = -M_PI - val; 
  // }



  double vx =  trajectory_.longitudinal_velocity_mps;

  double R = 0.0;
  if(curvature_!=0.0){
    R = 1/curvature_;
  }

  double phi_des_rps = vx/R;

  double lf_ = 0.114;       // [m] front overhang length
  double lr_ = 0.11;  
  double delta_r = 0.0;
  double delta_f = u_prev_;
  double beta = atan2(lf_*tan(delta_r) + lr_ * tan(delta_f), lf_+lr_);
  double v = vx/cos(beta);
  double y_dot = v*sin(phi + beta);


  double e1 = current_pose_.position.y - trajectory_.pose.position.y;
  double e1_dot = y_dot + vx*(phi - phi_des);
  double e2 = phi - phi_des;
  double e2_dot =  current_vel_.twist.angular.z - phi_des_rps;

  // if(last_nearest_index_>105 ){
  //   e1=-e1;
  //   e1_dot = -e1_dot;
  //   e2=-e2;
  //   e2_dot=-e2_dot;
  // }

   


  Eigen::Vector<double,4> state = Eigen::Vector<double,4>(
   e1, //trajectory_.pose.position.y - current_pose_.position.y
   e1_dot, ///trajectory_.lateral_velocity_mps - current_vel_.twist.linear.y 
   e2,
   e2_dot
   );

  double u = lqr_->calculate_control_signal(vx,phi_des_rps,tp, state,R,last_nearest_index_);
  // auto time = (double)(input_data.current_trajectory.header.stamp.sec + input_data.current_trajectory.header.stamp.nanosec*1e-9);

  if(last_nearest_index_>104 ){
    u=-u;
  }



  // if(last_nearest_index_>134){
  //   u=-u;
  // }
  


  u_prev_=u;

  RCLCPP_ERROR(
    logger_,
    "\n --- Run --- \n"
    "- control signal u: %f \n"
    "- phi: %f \n"
    "- phi_des: %f \n"
    "- heading_rate_rps: %f \n"
    "- innex : %ld \n"
    "- Z_angular_vel: %f \n"
    "y_dot_res %f \n"
    " trajectory_.longitudinal_velocity_mps %f \n"
    " timestamp %f \n"
    " time ros %f \n"
    "y curr %f \n"
    "curvature %f \n"
    "actual x %f \n"
    "R %f"
    "current position %f"
    "pos des %f"
    "error yaw %f"
  
    "--- --- ---",
    u*180/M_PI, phi, phi_des, phi_des_rps, *closest_idx_result,current_vel_.twist.angular.z, y_dot_des,trajectory_.longitudinal_velocity_mps,time,tp,y_dot,curvature_,vx,R,current_pose_.position.y,trajectory_.pose.position.y,error_orient); // TODO: Change from ERROR to INFO/DEBU

  time_prev_=time;


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
