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

#ifndef LQR_LATERAL_CONTROLLER__LQR_LATERAL_CONTROLLER_HPP_
#define LQR_LATERAL_CONTROLLER__LQR_LATERAL_CONTROLLER_HPP_

#include <cstdint>
#include <memory>
#include "tf2/utils.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "trajectory_follower_base/lateral_controller_base.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
<<<<<<< Updated upstream
// #include "geometry_msgs/msg/twist_with_covariance.hpp"
// #include <geometry_msgs/msg/quaternion.hpp>
// #include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
=======
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
>>>>>>> Stashed changes

#include "lqr_lateral_controller/lqr.hpp"
#include "lqr_lateral_controller/visibility_control.hpp"
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>

// #include <fstream>
// #include <vector>
// #include <sstream>

using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralControllerBase;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware_auto_control_msgs::msg::AckermannLateralCommand;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

namespace lqr_lateral_controller
{



struct Param
{
  // Global Parameters
  double wheel_base;
  double max_steering_angle;  // [rad]

  // Algorithm Parameters
  double ld_velocity_ratio;  // TDOO: Declare more parameters
  // double ld_lateral_error_ratio;
  // double ld_curvature_ratio;
  // double min_lookahead_distance;
  // double max_lookahead_distance;
  // double reverse_min_lookahead_distance;  // min_lookahead_distance in reverse gear
  // double converged_steer_rad_;
  // double prediction_ds;
  // double prediction_distance_length;  // Total distance of prediction trajectory
  // double resampling_ds;
  // double curvature_calculation_distance;
  // double long_ld_lateral_error_threshold;
  // bool enable_path_smoothing;
  // int path_filter_moving_ave_num;
};

class LQR_LATERAL_CONTROLLER_PUBLIC LqrLateralController : public LateralControllerBase
{
public:
  /// \param node Reference to the node used only for the component and parameter initialization.
  explicit LqrLateralController(rclcpp::Node & node);
  ~LqrLateralController() = default;

  void testObject() const;

private:
  bool isReady([[maybe_unused]]const InputData & input_data) override;  // From base class
  LateralOutput run(InputData const & input_data) override;  // From base class
  // std::pair<bool, int32_t> findClosestIdxWithDistAngThr(const std::vector<geometry_msgs::msg::Pose> & poses,
  // const geometry_msgs::msg::Pose & current_pose, double th_dist, double th_yaw);
  double normalizeEulerAngle(const double euler);

  // std::vector<geometry_msgs::msg::Pose> extractPoses(
  // const autoware_auto_planning_msgs::msg::Trajectory & trajectory);

  // Logger and clock
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  // string filename_="debug.txt";
  // std::ofstream file_;
  
  geometry_msgs::msg::Pose current_pose_;
  autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_;
  nav_msgs::msg::Odometry current_odometry_;
  autoware_auto_vehicle_msgs::msg::SteeringReport current_steering_;
  geometry_msgs::msg::TwistWithCovariance current_vel_;
  double last_phi_;
  double last_time_;
  // uint8_t i_{};
  // std::ofstream csv_file_;



  autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_ref_;

  AckermannLateralCommand generateOutputControlCmd(const double& target_curvature);

  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr  subscription_;
  void get_ref_trajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr  msg );

  

  // Parameters
  Param param_{};
<<<<<<< Updated upstream
=======
  // double prev_phi_des_{};
  size_t last_nearest_index_;
  // Algorithm
  std::shared_ptr<lqr_lateral_controller::LQR> lqr_;
>>>>>>> Stashed changes


  // Algorithm
  std::shared_ptr<lqr_lateral_controller::LQR> lqr = std::make_shared<lqr_lateral_controller::LQR>();
//;
};

}  // namespace lqr_lateral_controller

#endif  // LQR_LATERAL_CONTROLLER__LQR_LATERAL_CONTROLLER_HPP_
