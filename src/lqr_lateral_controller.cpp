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

LqrLateralController::LqrLateralController()
{
  // RCLCPP_INFO(m_logger, "LQR Lateral controller initialized.");
}

bool LqrLateralController::isReady([[maybe_unused]] const InputData & input_data)
{
  return true;
}

LateralOutput LqrLateralController::run([[maybe_unused]] const InputData & input_data)
{
  LateralOutput output;
  return output;
}

int64_t LqrLateralController::foo(int64_t bar) const
{
  std::cout << "Hello World, " << bar << std::endl;
  return bar;
}

}  // namespace lqr_lateral_controller
