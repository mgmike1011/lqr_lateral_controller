#include "lqr_lateral_controller/lqr.hpp"

namespace lqr_lateral_controller
{

LQR::LQR()
{
  RCLCPP_INFO(logger_, "LQR controller initialization.");

  Calfa_ = 1.0;
  m_ = 1.0;          // [kg] mass
  lf_ = 0.114;       // [m] front overhang length
  lr_ = 0.11;        // [m] rear overhang length
  Iz_ = 0.02623506;  // [kg*m^2]
  N_ = 5;
  tc_ = 0.1;
  L_ = 5.0;
}

int64_t LQR::testObject(int64_t bar) const
{
  std::cout << "Hello from LQR controller: " << bar << std::endl;
  return bar;
}

Eigen::Matrix4d LQR::get_A(const double & v_x)
{
  Eigen::Matrix4d A = {};

  const double A_11 = -(2 * Calfa_ + 2 * Calfa_) / (m_ * v_x);
  const double A_12 = (2 * Calfa_ + 2 * Calfa_) / m_;
  const double A_13 = (-2 * Calfa_ * lf_ + 2 * Calfa_ * lr_) / (m_ * v_x);
  const double A_31 = -(2 * Calfa_ * lf_ - 2 * Calfa_ * lr_) / (Iz_ * v_x);
  const double A_32 = (2 * Calfa_ * lf_ - 2 * Calfa_ * lr_) / Iz_;
  const double A_33 = -(2 * Calfa_ * lf_ * lf_ + 2 * Calfa_ * lr_ * lr_) / (Iz_ * v_x);

  A << 0, 1, 0, 0, 0, A_11, A_12, A_13, 0, 0, 0, 1, 0, A_31, A_32, A_33;

  return A;
}

Eigen::Vector4d LQR::get_B()
{
  Eigen::Vector4d B = {};
  const double B_10 = 2 * Calfa_ / m_;
  const double B_30 = 2 * Calfa_ * lf_ / Iz_;

  B << 0, B_10, 0, B_30;

  return B;
}

Eigen::Vector4d LQR::get_C(const double & v_x, const double & yaw_des_dot)
{
  Eigen::Vector4d C = {};

  const double C_10 = -(2 * Calfa_ * lf_ - 2 * Calfa_ * lr_) / (m_ * v_x) - v_x;
  const double C_30 = -(2 * Calfa_ * lf_ * lf_ + 2 * Calfa_ * lr_ * lr_) / (Iz_ * v_x);

  C << 0, C_10, 0, C_30;

  return C * yaw_des_dot;
}
void LQR::set_Q_(const Eigen::Vector4d & C)
{
  Q_ = C * C.transpose()*tc_;
}

void LQR::set_Q(const Eigen::Vector4d & gains)
{
  Q_ = gains.asDiagonal();
}

void LQR::set_R(const double & r)
{
  R_ = r;
}

Eigen::RowVector<double, 4> LQR::get_K(
  double v_x, double yaw_des_dot, const size_t & index)
{
  const Eigen::Matrix4d A = get_A(v_x);
  const Eigen::Vector4d B = get_B();


  const Eigen::Vector4d C = get_C(v_x, yaw_des_dot);
  const Eigen::Matrix4d Phi = Eigen::Matrix4d::Identity() + A * tc_;
  const Eigen::Vector4d Gamma = B * tc_;

  set_Q_(C);
  
  Q_(0, 0) = 1e-2;

  if (index > 105) {
    Q_(0, 0) = 5e-3;
       
  }
  if (index > 140) {
    Q_(0, 0) =1e-2;
  }

  set_R(1e-4);

  Eigen::Matrix4d Pt = Q_;

  for (int i = 0; i < N_; i++) {
    const double res = (R_ + Gamma.transpose() * Pt * Gamma);

    const auto Pt_1 = Q_ + Phi.transpose() * Pt * Phi -
                Phi.transpose() * Pt * Gamma * (1 / res) * Gamma.transpose() * Pt * Phi;
    Pt = Pt_1;
  }

  const double res = (R_ + Gamma.transpose() * Pt * Gamma);

  Eigen::RowVector<double, 4> K = -1 / res * Gamma.transpose() * Pt * Phi;


  return K;
}

double LQR::calculate_control_signal(
  const double & v_x, const double & yaw_des_dot, const Eigen::Vector4d & xstate,
  const double & R, const size_t & index)
{
  x_state_ = xstate;

  Eigen::RowVector<double, 4> K = get_K(v_x, yaw_des_dot, index);
  
  const double ay = v_x * v_x / R;
  const double L = lf_ + lr_;

  const double mr = m_ * lf_ / L;
  const double mf = m_ * lr_ / L;

  const double Kv = (mf / (2 * Calfa_)) - (mr / (2 * Calfa_));
  const double e2_ss = (-lr_ / R) + ((lf_ * m_ * v_x * v_x) / (2 * Calfa_ * L * R));

  double delta_ff = L / R + Kv * ay + K(2) * e2_ss;

  const double u = (K * x_state_) + delta_ff;

  if (isnan(u)) {
    return 0.0;
  }
  return u;
}

}  // namespace lqr_lateral_controller
