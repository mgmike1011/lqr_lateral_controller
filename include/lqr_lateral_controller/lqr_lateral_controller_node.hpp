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

#ifndef LQR_LATERAL_CONTROLLER__LQR_LATERAL_CONTROLLER_NODE_HPP_
#define LQR_LATERAL_CONTROLLER__LQR_LATERAL_CONTROLLER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "lqr_lateral_controller/lqr_lateral_controller.hpp"

namespace lqr_lateral_controller
{
using LqrLateralControllerPtr = std::unique_ptr<lqr_lateral_controller::LqrLateralController>;

class LQR_LATERAL_CONTROLLER_PUBLIC LqrLateralControllerNode : public rclcpp::Node
{
public:
  explicit LqrLateralControllerNode(const rclcpp::NodeOptions & options);

private:
  //
  //  NOT USED - launch from autoware
  //
};
}  // namespace lqr_lateral_controller

#endif  // LQR_LATERAL_CONTROLLER__LQR_LATERAL_CONTROLLER_NODE_HPP_
