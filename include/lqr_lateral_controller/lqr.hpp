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
private:
  double Calfa_;  // [-] wheel tire leteral stiffness; for simplicity assumed to be the
                              // same for front and rear wheels
  // Data taken from unity model, scripts and configs
  double m_ ;            // [kg] mass
  double lf_;       // [m] front overhang length
  double lr_;        // [m] rear overhang length
  double Iz_;  // [kg*m^2]
  int8_t N_ ;            // horizon for riccati recursion
  double tc_;
  double R_;
  double L_; // look-ahead distance
  Eigen::Vector4d x_state_;
  Eigen::Matrix4d Q_ ;

  Eigen::Matrix4d get_A(const double & v_x);
  Eigen::Vector4d get_B();
  Eigen::Vector4d get_C(const double & v_x, const double & yaw_des_dot);
  Eigen::RowVector<double,4> get_K(double v_x, double yaw_des_dot,const double & tp,const size_t & index);
  void set_Q_(const Eigen::Vector4d & C);

public:
  LQR();
  ~LQR() = default;

  Eigen::RowVector<double,5> K_ = {};

  rclcpp::Logger logger_ = rclcpp::get_logger("lqr");
  int64_t testObject(int64_t bar) const;

  void set_Q(const Eigen::Vector4d & gains);
  void set_R(const double & r);
  double calculate_control_signal(const double & v_x, const double & yaw_des_dot,const double & tp,const Eigen::Vector4d & xstate, const double & R,const size_t & index);
};
}  // namespace lqr_lateral_controller
#endif  // LQR_HPP_
