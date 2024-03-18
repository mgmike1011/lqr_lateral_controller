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

#include "lqr_lateral_controller/visibility_control.hpp"

using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralControllerBase;
using autoware::motion::control::trajectory_follower::LateralOutput;

namespace lqr_lateral_controller
{

class LQR_LATERAL_CONTROLLER_PUBLIC LqrLateralController : public LateralControllerBase
{
public:
  LqrLateralController();
  ~LqrLateralController() = default;

  // rclcpp::Logger m_logger = rclcpp::get_logger("lqr_lateral_controller_logger");  // ROS logger used for debug logging.
  
  int64_t foo(int64_t bar) const;
private:
  bool isReady([[maybe_unused]]const InputData & input_data) override;
  LateralOutput run([[maybe_unused]] InputData const & input_data) override;
};

}  // namespace lqr_lateral_controller

#endif  // LQR_LATERAL_CONTROLLER__LQR_LATERAL_CONTROLLER_HPP_
