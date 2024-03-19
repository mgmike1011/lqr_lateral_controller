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

#include <iostream>

namespace lqr_lateral_controller
{

LqrLateralController::LqrLateralController(rclcpp::Node & node)
: clock_(node.get_clock())
{
  RCLCPP_INFO(m_logger, "LQR Lateral controller initialized.");
}
// LqrLateralController::LqrLateralController()
// {

// }

AckermannLateralCommand LqrLateralController::generateOutputControlCmd()
{
  AckermannLateralCommand output_cmd;
  output_cmd.stamp = clock_->now();
  output_cmd.steering_tire_angle = 1.0;

  return output_cmd;
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

  // setResampledTrajectory();
  // if (param_.enable_path_smoothing) {
  //   averageFilterTrajectory(*trajectory_resampled_);
  // }

  const auto cmd_msg = generateOutputControlCmd();

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

int64_t LqrLateralController::foo(int64_t bar) const
{
  std::cout << "Hello World, " << bar << std::endl;
  return bar;
}

}  // namespace lqr_lateral_controller
