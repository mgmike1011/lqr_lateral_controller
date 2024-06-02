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

#include "rclcpp/rclcpp.hpp"
#include "trajectory_follower_base/lateral_controller_base.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include "lqr_lateral_controller/lqr.hpp"
#include "lqr_lateral_controller/visibility_control.hpp"
#include <cmath>

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
  double wheel_base;
  double max_steering_angle;  // [rad]
  double converged_steer_rad_;

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

  // Logger and clock
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  
  // Algorithm input data
  geometry_msgs::msg::Pose current_pose_;
  autoware_auto_planning_msgs::msg::TrajectoryPoint trajectory_;
  nav_msgs::msg::Odometry current_odometry_;
  autoware_auto_vehicle_msgs::msg::SteeringReport current_steering_;
  geometry_msgs::msg::TwistWithCovariance current_vel_;

  // Parameters
  Param param_{};
  double prev_phi_des_;
  double y_des_prev_;
  double y_prev_;
  double u_prev_;
  double time_prev_;
  size_t last_nearest_index_;
  double curvature_;
  double e_yLeI_;

  double y_dot_des;
  double y_dot;
  // Algorithm
  std::shared_ptr<lqr_lateral_controller::LQR> lqr_;

  // Methods
  AckermannLateralCommand generateOutputControlCmd(double& target_curvature);
  bool calcIsSteerConverged(const AckermannLateralCommand & cmd);
};

}  // namespace lqr_lateral_controller

#endif  // LQR_LATERAL_CONTROLLER__LQR_LATERAL_CONTROLLER_HPP_
