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
<<<<<<< Updated upstream
  // this->lqr_ = std::make_unique<LQR>();
 
=======
  this->lqr_ = std::make_shared<lqr_lateral_controller::LQR>();
  // Parameters
  // prev_phi_des_ = 0.0;
  last_nearest_index_ = 0;
  // i_=0;
  last_phi_=0.0;
  // last_time_=0.1;
>>>>>>> Stashed changes
  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;
  subscription_ = node.create_subscription<autoware_auto_planning_msgs::msg::Trajectory>("/planning/racing_planner/trajectory", 10, std::bind(&LqrLateralController::get_ref_trajectory, this,std::placeholders::_1));
  
  // Algorithm Parameters
<<<<<<< Updated upstream
  param_.ld_velocity_ratio = node.declare_parameter<double>("ld_velocity_ratio", 0.0);  // TODO: Declare more parameters
=======
  param_.converged_steer_rad_ = node.declare_parameter<double>("converged_steer_rad", 0.1);
  // csv_file_.open("data.csv");
  // csv_file_ << "header1,header2,header3\n";

  // if (!csv_file_.is_open()) {
  //   RCLCPP_ERROR(logger_, "file didnt initialized.");

  // }


  // file_.open(filename_, std::ios::app);
  //   if (!file_.is_open()) {
  //       std::cerr << "Error opening file: " << std::endl;
  //   }

  // else{
  //   file_<<"input_data.header.stamp.seq,u,phi.phi_des,phi_des_rps,*closest_idx_result,current_vel_.twist.linear.x,current_vel_.twist.angular.z,trajectory_.longitudinal_velocity_mps \n";
  // }


>>>>>>> Stashed changes

  RCLCPP_ERROR(logger_, "LQR Lateral controller initialized.");  // TODO: Change ERROR to INFO
}

<<<<<<< Updated upstream
AckermannLateralCommand LqrLateralController::generateOutputControlCmd(const double& target_curvature)
=======

// LqrLateralController:: ~LqrLateralController(){

//    if (csv_file_.is_open())
//     {
//         csv_file_.close();
//     }

// }

AckermannLateralCommand LqrLateralController::generateOutputControlCmd(double & target_curvature)
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
void LqrLateralController::get_ref_trajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr  msg ){
 RCLCPP_ERROR(logger_,"Message przed przypisaniem %.4f ",trajectory_.pose.position.y);
  RCLCPP_ERROR(logger_,"Message %d ",msg->points.back().time_from_start.sec);
  RCLCPP_ERROR(logger_,"Message %.4f ",msg->points.back().pose.position.y);
  trajectory_=msg->points[0];
  RCLCPP_ERROR(logger_,"Message after przypisanie %.4f ",trajectory_.pose.position.y);
=======


double LqrLateralController::normalizeEulerAngle(const double euler)
{
  double res = euler;
  while (res > M_PI) {
    res -= (2 * M_PI);
  }
  while (res < -M_PI) {
    res += 2 * M_PI;
  }

  return res;
}



// std::pair<bool, int32_t> LqrLateralController::findClosestIdxWithDistAngThr(
//   const std::vector<geometry_msgs::msg::Pose> & poses,
//   const geometry_msgs::msg::Pose & current_pose, double th_dist, double th_yaw)
// {
//   double dist_squared_min = std::numeric_limits<double>::max();
//   int32_t idx_min = -1;

//   for (size_t i = 0; i < poses.size(); ++i) {
//     const double ds = calcDistSquared2D(poses.at(i).position, current_pose.position);
//     if (ds > th_dist * th_dist) {
//       continue;
//     }

//     const double yaw_pose = tf2::getYaw(current_pose.orientation);
//     const double yaw_ps = tf2::getYaw(poses.at(i).orientation);
//     const double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_ps);
//     if (fabs(yaw_diff) > th_yaw) {
//       continue;
//     }

//     if (ds < dist_squared_min) {
//       dist_squared_min = ds;
//       idx_min = i;
//     }
//   }

//   return (idx_min >= 0) ? std::make_pair(true, idx_min) : std::make_pair(false, idx_min);
// }

bool LqrLateralController::calcIsSteerConverged(const AckermannLateralCommand & cmd)
{
  return std::abs(cmd.steering_tire_angle - current_steering_.steering_tire_angle) < static_cast<float>(param_.converged_steer_rad_);
>>>>>>> Stashed changes
}

// std::vector<geometry_msgs::msg::Pose> LqrLateralController::extractPoses(
//   const autoware_auto_planning_msgs::msg::Trajectory & trajectory)
// {
//   std::vector<geometry_msgs::msg::Pose> poses;

//   for (const auto & p : trajectory.points) {
//     poses.push_back(p.pose);
//   }

//   return poses;
// }

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


<<<<<<< Updated upstream
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
=======
  auto output_tp_array_ = motion_utils::convertToTrajectoryPointArray(input_data.current_trajectory);
  const auto closest_idx_result = motion_utils::findNearestIndex(output_tp_array_, current_pose_, 0.3,M_PI_4);
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


  //  const auto closest_idx_result = findClosestIdxWithDistAngThr(extractPoses(input_data.current_trajectory), current_pose_, 3.0, M_PI_4);

  // if (!closest_idx_result.first) {
  //   RCLCPP_ERROR(get_logger(), "cannot find closest waypoint");
  //   return {};
  // }

  // auto trajectory_=points.at(closest_idx_result.second);


  auto phi = tf2::getYaw(current_odometry_.pose.pose.orientation);
  // phi = atan2(sin(phi),cos(phi));
  auto phi_des = tf2::getYaw(trajectory_.pose.orientation);

  // phi_des = atan2(sin(phi_des),cos(phi_des));
  // if (phi_des>1.57){
  //   phi_des=1.57-phi_des;
  // }
  // else if(phi_des<-1.57){
  //   phi_des=-1.57-phi_des;
  // }

  // if (phi>1.57){
  //   phi=1.57-phi;
  // }
  // else if(phi<-1.57){
  //   phi=-1.57-phi;
  // }
  
  RCLCPP_ERROR(logger_,
  "\n x %f \n"
  "- y %f \n"
  "- z %f \n"
  "- w %f \n",
  trajectory_.pose.orientation.x,trajectory_.pose.orientation.y,trajectory_.pose.orientation.z,trajectory_.pose.orientation.w
  );




  // auto r= sqrt(pow(current_odometry_.pose.pose.position.x,2)+pow(current_odometry_.pose.pose.position.y,2)); 


  // Eigen::Quaterniond tf2_quat(trajectory_.pose.orientation.w,trajectory_.pose.orientation.x,trajectory_.pose.orientation.y,trajectory_.pose.orientation.z);
  // Eigen::Matrix3d rotation_matrix=tf2_quat.normalized().toRotationMatrix();

  // Eigen::Vector3d euler = tf2_quat.toRotationMatrix().eulerAngles(0,1,2);



  Eigen::Vector3d angular_rates(current_vel_.twist.angular.x,current_vel_.twist.angular.y,current_vel_.twist.angular.z);

  // auto local_angles = rotation_matrix.inverse()*euler; 

  auto curr_angular_vel = angular_rates;

  // auto local_angle = local_angle_prev + angular_rates(2);

  auto time = (double)(input_data.current_trajectory.header.stamp.sec + input_data.current_trajectory.header.stamp.nanosec*1e-9);

  // auto phi_des_rps =(tan(phi_des-phi)*trajectory_.longitudinal_velocity_mps)/0.548  ;
  double dt=0.01;
  auto phi_des_rps = (phi_des-last_phi_)/dt;
  last_phi_ = phi_des;



  // // auto phi_des_rps = (phi_des - prev_phi_des_) / 0.01;
  // prev_phi_des_ = phi_des;

  Eigen::Vector4d state = Eigen::Vector4d(
    trajectory_.pose.position.y - current_pose_.position.y,
    trajectory_.lateral_velocity_mps - current_vel_.twist.linear.y, 
    phi_des - phi,
    phi_des_rps - curr_angular_vel(2));

  double u = lqr_->calculate_control_signal(current_vel_.twist.linear.x,phi_des_rps,state);
  
  
  RCLCPP_ERROR(
    logger_,
    "\n --- Run --- \n"
    "- control signal u: %f \n"
    "- phi: %f \n"
    "- phi_des: %f \n"
    "- heading_rate_rps: %f \n"
    "- innex : %ld \n"
    "- v_x: %f \n"
    "- Z_angular_vel: %f \n"
    " - Heading_trajectory %f \n"
    "tan %f \n"
    " trajectory_.longitudinal_velocity_mps %f \n"
    " timestamp %f \n"
    "--- --- ---",
    u, phi, phi_des, phi_des_rps, *closest_idx_result,current_vel_.twist.linear.x , curr_angular_vel(2),trajectory_.heading_rate_rps, tan(phi_des),trajectory_.longitudinal_velocity_mps,time); // TODO: Change from ERROR to INFO/DEBUG
>>>>>>> Stashed changes

  std::cout<<"control signal"<<u<<std::endl;
  const auto cmd_msg = generateOutputControlCmd(u);

  // if (!file_.is_open()) {
  //       std::cerr << "Error opening file: " << std::endl;
  //   }

  // else{
  //   file_<<input_data.header.stamp.seq;
  //   file_<<",";
  //   file_<<u;
  //   file_<<","
  //   file_<<phi;
  //   file_<<",";
  //   file_<<phi_des;
  //   file_<<",";
  //   file_<<phi_des_rps;
  //   file_<<",";
  //   file_<<*closest_idx_result;
  //   file_<<",";
  //   file_<<current_vel_.twist.linear.x ;
  //    file_<<",";
  //   file_<<current_vel_.twist.angular.z;
  //   file_<<",";
  //   file_<<trajectory_.longitudinal_velocity_mps;
  //   file_<<"\n";


  // }


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
