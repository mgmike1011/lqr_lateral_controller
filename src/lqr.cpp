#include "lqr_lateral_controller/lqr.hpp"

namespace lqr_lateral_controller
{

LQR::LQR()
{
  RCLCPP_INFO(logger_, "LQR controller initialization.");

  Calfa_=1.0;
  m_ = 1.0;            // [kg] mass
  lf_ = 0.114;       // [m] front overhang length
  lr_ = 0.11;        // [m] rear overhang length
  Iz_ = 0.02623506;  // [kg*m^2]
  N_ = 5;   
  tc_=0.1;
  L_=5.0;
  
}

int64_t LQR::testObject(int64_t bar) const
{
  std::cout << "Hello from LQR controller: " << bar << std::endl;
  return bar;
}

Eigen::Matrix4d LQR::get_A(const double & v_x)
{
  Eigen::Matrix4d A = {};

  double A_11 = -(2*Calfa_+2*Calfa_)/(m_*v_x);
  double A_12 = (2*Calfa_ + 2*Calfa_)/m_;
  double A_13 = (-2*Calfa_*lf_ + 2*Calfa_*lr_)/(m_*v_x);
  double A_31 = -(2*Calfa_*lf_ - 2*Calfa_*lr_)/(Iz_*v_x);
  double A_32 = (2*Calfa_*lf_ - 2*Calfa_*lr_)/ Iz_;
  double A_33 = -(2*Calfa_*lf_*lf_ + 2*Calfa_*lr_*lr_)/(Iz_*v_x);

  A <<0,1,0,0,
  0,A_11,A_12,A_13,
  0,0,0,1,
  0,A_31,A_32,A_33;


  RCLCPP_INFO(logger_,"AAAAAAAAA %f, %f, %f, %f, %f, %f, %f",A(5),m_,Calfa_,Iz_,tc_,lf_,lr_);

  return A;
}

 Eigen::Vector4d LQR::get_B()
{
   Eigen::Vector4d B = {};
   double B_10 = 2*Calfa_/m_;
   double B_30 = 2*Calfa_*lf_/Iz_;

  B << 0,B_10,0,B_30;

  return B;
}

 Eigen::Vector4d LQR::get_C(const double & v_x, const double & yaw_des_dot)
{

  Eigen::Vector4d C = {};


  double C_10 = -(2*Calfa_*lf_ - 2*Calfa_*lr_)/(m_*v_x) - v_x;
  double C_30 = -(2*Calfa_*lf_*lf_ + 2*Calfa_*lr_*lr_)/(Iz_*v_x);

  C<< 0,C_10,0,C_30;
  
  return C*yaw_des_dot;
}
void LQR::set_Q_(const Eigen::Vector4d & C)
{
  Q_ = C * C.transpose();
}

void LQR::set_Q(const  Eigen::Vector4d& gains)
{
  Q_ = gains.asDiagonal();
}

void LQR::set_R(const double & r)
{
  R_ = r;
}

Eigen::RowVector<double,4> LQR::get_K(double v_x, double yaw_des_dot,const double & tp)
{
  Eigen::Matrix4d  A = get_A(v_x);
  Eigen::Vector4d B = get_B();

  
  Eigen::Vector4d C = get_C(v_x,yaw_des_dot);
  // Eigen::Matrix4d  Q_f = Eigen::Matrix4d ::Zero();  // C*C.transpose();
  Eigen::Matrix4d  Phi =  Eigen::Matrix4d::Identity() + A*tp;
  Eigen::Vector4d Gamma = B*tp;
 

    RCLCPP_INFO(logger_, "B %f %f %f %f \n ", 
    B(0),B(1),B(2),B(3)
    );

  RCLCPP_INFO(logger_, "C %f %f %f %f \n ", 
    C(0),C(1),C(2),C(3)
    );



   RCLCPP_INFO(logger_, "A  %f %f %f %f \n %f %f %f %f  \n %f %f %f %f  \n %f %f %f %f  \n   ", 
    A(0,0),A(0,1),A(0,2),A(0,3),
    A(1,0),A(1,1),A(1,2),A(1,3),
    A(2,0),A(2,1),A(2,2),A(2,3),
    A(3,0),A(3,1),A(3,2),A(3,3)  
    );

  

   RCLCPP_INFO(logger_, "Phi  %f %f %f %f  \n %f %f %f %f \n %f %f %f %f  \n %f %f %f %f  \n   ", 
    Phi(0,0),Phi(0,1),Phi(0,2),Phi(0,3),
    Phi(1,0),Phi(1,1),Phi(1,2),Phi(1,3),
    Phi(2,0),Phi(2,1),Phi(2,2),Phi(2,3),
    Phi(3,0),Phi(3,1),Phi(3,2),Phi(3,3)
    );

  RCLCPP_INFO(logger_, "Gamma %f %f %f %f\n ", 
    Gamma(0),Gamma(1),Gamma(2),Gamma(3)
    );
  
  // RCLCPP_INFO(logger_, "Gamma phi %f %f %f %f %f \n ", 
  //   Gamma_phi(0),Gamma_phi(1),Gamma_phi(2),Gamma_phi(3),Gamma_phi(4)
  //   );

  

  // Eigen::Matrix4d  Q_d;
  // Q_d=C*C.transpose();

  // RCLCPP_INFO(logger_, "C %f, %f, %f, %f ", C(0),C(1),C(2),C(3));


  Eigen::Matrix4d  Q_f;
  // Q_f.diagonal() << 15, 10, 10, 10;
  Q_f=C*C.transpose()*tp;
  Q_f(0,0) =5e-2;

  Q_ = Q_f;
  R_=1e-4;





  Eigen::Matrix4d  Pt = Q_;

    RCLCPP_INFO(logger_, "P_t before %f %f %f %f  \n %f %f %f %f \n %f %f %f %f  \n %f %f %f %f  \n ", 
    Pt(0,0),Pt(0,1),Pt(0,2),Pt(0,3),
    Pt(1,0),Pt(1,1),Pt(1,2),Pt(1,3),
    Pt(2,0),Pt(2,1),Pt(2,2),Pt(2,3),
    Pt(3,0),Pt(3,1),Pt(3,2),Pt(3,3)
    );

 for (int i = 0; i < N_; i++) 
  {
    const double res = (R_ + Gamma.transpose() * Pt * Gamma);
    double res_loop =  Gamma.transpose() * Pt * Gamma;
    RCLCPP_INFO(logger_, "Res loop %f ", res_loop);


    auto Pt_1 = Q_ + Phi.transpose() * Pt * Phi - Phi.transpose() * Pt * Gamma * (1 / res) * Gamma.transpose() * Pt * Phi;
    Pt=Pt_1;
    
    }

  
  RCLCPP_INFO(logger_, "P_t after %f %f %f %f  \n %f %f %f %f \n %f %f %f %f  \n %f %f %f %f \n  ", 
    Pt(0,0),Pt(0,1),Pt(0,2),Pt(0,3),
    Pt(1,0),Pt(1,1),Pt(1,2),Pt(1,3),
    Pt(2,0),Pt(2,1),Pt(2,2),Pt(2,3),
    Pt(3,0),Pt(3,1),Pt(3,2),Pt(3,3)    
  );


 const double res = (R_ + Gamma.transpose() * Pt * Gamma);
 RCLCPP_INFO(logger_, "Res %f ", res);

 const Eigen::RowVector<double,4> K = -1/res * Gamma.transpose() * Pt * Phi;

  return K;
}

double LQR::calculate_control_signal(const double & v_x, const double & yaw_des_dot,const double & tp,const  Eigen::Vector4d & xstate, const double & R)
{
  RCLCPP_INFO(logger_, "Calculate control signal.");

  x_state_ = xstate;
  
  Eigen::RowVector<double,4> K = get_K(v_x,yaw_des_dot,tp);
  RCLCPP_INFO(logger_, "K size %ld, %ld", K.rows(), K.cols());

  RCLCPP_INFO(logger_, "x_state_ %f %f %f %f \n ", 
    x_state_(0),x_state_(1),x_state_(2),x_state_(3)
    );

   
    RCLCPP_INFO(logger_, "K %f %f %f %f \n ", 
    K(0),K(1),K(2),K(3)
    );




  double ay = v_x*v_x/R;
  double L = lf_+lr_;

  double mr = m_*lf_/L;
  double mf = m_*lr_/L;

  double Kv = (mf/(2*Calfa_)) - (mr/(2*Calfa_));
  double e2_ss = (-lr_/R) + ((lf_*m_*v_x*v_x)/(2*Calfa_*L*R));

  double delta_ff = L/R + Kv*ay + K(2)*e2_ss;

  const double u = (K * x_state_) + delta_ff;

  RCLCPP_INFO(logger_, "delta_ff %f ", delta_ff);

  if (isnan(u)) 
  {
    return 0.0;
  }
  return u;
}

}  // namespace lqr_lateral_controller
