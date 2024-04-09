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

#include "lqr_lateral_controller/lqr_lateral_controller_node.hpp"

namespace lqr_lateral_controller
{

LqrLateralControllerNode::LqrLateralControllerNode(const rclcpp::NodeOptions & options)
:  Node("lqr_lateral_controller", options)
{
  //
  //  NOT USED - launch from autoware
  //

  // Eigen::Vector4d state = Eigen::Vector4d(0.1-1.0,0.05-0.5,1-20,0.05-0.5);
  // std::cout<<"test111"<<std::endl;
  // // std::cout<<lqr_->x_state_(0)<<std::endl;
  // // auto u = lqr_->calculate_control_signal(5.0,0.1,state); 
  // std::cout<<u<<std::endl;
  

  std::cout<<"test22"<<std::endl;
}

}  // namespace lqr_lateral_controller

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(lqr_lateral_controller::LqrLateralControllerNode)
