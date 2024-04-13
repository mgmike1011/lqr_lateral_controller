#ifndef LQR_HPP_
#define LQR_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <math.h>

// implemenataion based on: https://ieeexplore.ieee.org/document/9051390
// LQR version with finite horizon

namespace lqr_lateral_controller
{
class LQR
{
public:
  LQR();
  ~LQR() = default;
  rclcpp::Logger logger_ = rclcpp::get_logger("lqr");
  int64_t testObject(int64_t bar) const;

  void set_Q(const Eigen::Vector4d& gains);
  void set_R(const double& r);
  double calculate_control_signal(const double& v_x, const double& yaw_des_dot, const Eigen::Vector4d& xstate);

private:
  const double Calfa_ = 150;  // [-] wheel tire leteral stiffness; for simplicity assumed to be the
                              // same for front and rear wheels
  // data taken from unity model, scripts and configs
  const double m_ = 5;            // [kg] mass
  const double lf_ = 0.114;       // [m] front overhang length
  const double lr_ = 0.11;        // [m] rear overhang length
  const double Iz_ = 0.02623506;  // [kg*m^2]
  const int8_t N_ = 20;           // horizon for riccati recursion
  Eigen::Vector4d x_state_;
  double R_;
  Eigen::Matrix4d Q_;
  Eigen::Matrix4d get_A(const double& v_x);
  Eigen::Vector4d get_B();
  Eigen::Vector4d get_C(const double& v_x, const double& yaw_des_dot);
  Eigen::RowVector4d get_K(const double& v_x, const double& yaw_des_dot);
  void set_Q_(const Eigen::Matrix4d& C);


};
}  // namespace lqr_lateral_controller
#endif  // LQR_HPP_
