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

#include "lqr_lateral_controller/visibility_control.hpp"


namespace lqr_lateral_controller
{

class LQR_LATERAL_CONTROLLER_PUBLIC LqrLateralController
{
public:
  LqrLateralController();
  int64_t foo(int64_t bar) const;
};

}  // namespace lqr_lateral_controller

#endif  // LQR_LATERAL_CONTROLLER__LQR_LATERAL_CONTROLLER_HPP_
