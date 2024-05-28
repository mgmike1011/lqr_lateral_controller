#include "lqr_lateral_controller/lqr.hpp"

namespace lqr_lateral_controller
{

LQR::LQR()
{
<<<<<<< Updated upstream
    std::cout<<"jestem w lqr"<<std::endl;
    // x_state_=Eigen::Vector4d(0,0,0,0);
    RCLCPP_INFO(logger_, "LQR controller initialization.");
=======
  RCLCPP_INFO(logger_, "LQR controller initialization.");
  tc_=0.01;

  Calfa_ = 250;  // [-] wheel tire leteral stiffness; for simplicity assumed to be the
                              // same for front and rear wheels
  // Data taken from unity model, scripts and configs
  m_ = 5;            // [kg] mass
  lf_ = 0.114;       // [m] front overhang length
  lr_ = 0.11;        // [m] rear overhang length
  Iz_ = 0.02623506;  // [kg*m^2]
  N_ = 10; 
>>>>>>> Stashed changes
}

int64_t LQR::testObject(int64_t bar) const
{
    std::cout << "Hello from LQR controller" << bar << std::endl;
    return bar;
}

<<<<<<< Updated upstream
Eigen::Matrix4d LQR::get_A(double v_x){

    Eigen::Matrix4d A;

    A << 0, 1, 0, 0,
        0, -(2*Calfa_ + 2*Calfa_)/(m_ * v_x), (2*Calfa_+2*Calfa_)/m_, (-2*Calfa_*lf_ + 2*Calfa_*lr_)/(m_*v_x),
        0, 0, 0, 1,
        0, -(2*Calfa_*lf_ - 2*Calfa_*lr_)/(Iz_*v_x), (2*Calfa_*lf_ - 2*Calfa_*lr_)/Iz_, -(2*Calfa_*pow(lf_,2)+2*Calfa_*pow(lr_,2))/(Iz_*v_x);

    return A;    
=======
Eigen::Matrix4d LQR::get_A(const double & v_x){

 RCLCPP_INFO(logger_, "AAAAAAAAAAA, %f, %f",Calfa_,Iz_);
  Eigen::Matrix4d A = {};

  A << 0, 1, 0, 0,
   0, -(2 * Calfa_ + 2 * Calfa_) / (m_ * v_x), (2 * Calfa_ + 2 * Calfa_) / m_, (-2 * Calfa_ * lf_ + 2 * Calfa_ * lr_) / (m_ * v_x),
   0, 0, 0, 1,
   0,-(2 * Calfa_ * lf_ - 2 * Calfa_ * lr_) / (Iz_ * v_x),
   (2 * Calfa_ * lf_ - 2 * Calfa_ * lr_) / Iz_,
    -(2 * Calfa_ * pow(lf_, 2) + 2 * Calfa_ * pow(lr_, 2)) / (Iz_ * v_x);

    RCLCPP_INFO(logger_, "AAAAAAAAAAA, %f,",A(5));

  return A;
>>>>>>> Stashed changes
}

Eigen::Vector4d LQR::get_B(){
    Eigen::Vector4d B;

    B<< 0, 2*Calfa_/m_, 0, 2*lf_*Calfa_/Iz_;

<<<<<<< Updated upstream
    return B; 
=======
  B << 0, 2 * Calfa_  / m_, 0, 2 * lf_ * Calfa_ / Iz_;
>>>>>>> Stashed changes

}

Eigen::Vector4d LQR::get_C(double v_x, double yaw_des_dot){

    Eigen::Vector4d C;

<<<<<<< Updated upstream
    C<< 0, (-(2*Calfa_*lf_-2*Calfa_*lr_)/(m_*v_x) - v_x)*yaw_des_dot, 0, -(2*Calfa_*pow(lf_,2) + 2*Calfa_*pow(lr_,2))/(Iz_*v_x) * yaw_des_dot;
=======
  C << 0, 
       std::abs((-(2 * Calfa_ * lf_ - 2 * Calfa_ * lr_) / (m_ * v_x) - v_x) * yaw_des_dot), 
       0,
       std::abs(-(2 * Calfa_ * pow(lf_, 2) + 2 * Calfa_ * pow(lr_, 2)) / (Iz_ * v_x) * yaw_des_dot);
>>>>>>> Stashed changes

    return C;
}
void LQR::set_Q_(Eigen::Matrix4d C){

    Q_=C*C.transpose();
}

void LQR::set_Q(Eigen::Vector4d gains){

    Q_ = gains.asDiagonal();
}

void LQR::set_R(double r){

    R_=r;
}

<<<<<<< Updated upstream
Eigen::RowVector4d LQR::get_K(double v_x){

    Eigen::Matrix4d A =get_A(v_x);
    Eigen::Vector4d B =get_B();
    // Eigen::Vector4d C =get_C(v_x,yaw_des_dot);
    Eigen::Matrix4d Q_f= Eigen::Matrix4d::Zero();//C*C.transpose();
    Q_f.diagonal()<< 0.5,0.5,0.5,0.5;

    Q_=Q_f;
    // set_Q_(C);
    set_R(0.5);

    Eigen::Matrix4d Pt = Q_;
=======
Eigen::RowVector4d LQR::get_K(const double & v_x, const double & yaw_des_dot)
{
  Eigen::Matrix4d A = get_A(v_x);
  Eigen::Vector4d B = get_B();
  // Eigen::Matrix4d Q_f = Eigen::Matrix4d::Zero();  // C*C.transpose();
  // Eigen::Vector4d C =get_C(v_x,yaw_des_dot);

  Eigen::Vector4d C(0,std::abs((-(2 * Calfa_ * lf_ - 2 * Calfa_ * lr_) / (m_ * v_x) - v_x) * yaw_des_dot),  0,std::abs(-(2 * Calfa_ * pow(lf_, 2) + 2 * Calfa_ * pow(lr_, 2)) / (Iz_ * v_x) * yaw_des_dot));

  RCLCPP_INFO(logger_, "C %f, %f, %f, %f ", C(0),C(1),C(2),C(3));

  
  Eigen::Matrix4d Phi =  Eigen::Matrix4d::Identity() + A*tc_;
  Eigen::Vector4d Gamma = Phi*B*tc_;
  // Eigen::Vector4d Gamma_phi = Phi*C*tc_;




  
  Eigen::Matrix4d Q_d;
  Q_d=C*C.transpose();
  // Q_=Q_f;

  Eigen::Matrix4d Q_f;
  Q_f.diagonal() << 1000, 1000, 1000, 1000;

  Q_ = Q_f;
  R_=0.001;
  // set_R(0.00001);

  for(int i=0;i<16;i++){
   
    RCLCPP_INFO(logger_, "Q_d %f ", Q_d(i));
  
  }
>>>>>>> Stashed changes

    for(int i =0;i<N_;i++){

<<<<<<< Updated upstream
        double res=(R_+B.transpose()*Pt*B);

        auto Pt_1 = Q_ + A.transpose()*Pt*A - A.transpose()*Pt*B*(1/res)*B.transpose()*Pt*A;
=======
  for (int i = 0; i < N_; i++) 
  {
    const double res = (R_ + Gamma.transpose() * Pt * Gamma);

    auto Pt_1 =
      Q_ + Phi.transpose() * Pt * Phi - Phi.transpose() * Pt * Gamma * (1 / res) * Gamma.transpose() * Pt * Phi;
>>>>>>> Stashed changes

        Pt=Pt_1;
    }

<<<<<<< Updated upstream
    double res = (R_ + B.transpose()*Pt*B);

    Eigen::RowVector4d K = -(1/res)*B.transpose()*Pt*A;
=======
  const double res = (R_ + Gamma.transpose() * Pt * Gamma);

  const Eigen::RowVector4d K = -1/res * Gamma.transpose() * Pt * Phi;
>>>>>>> Stashed changes

    return K;
}

<<<<<<< Updated upstream
double LQR::calculate_control_signal(double v_x, Eigen::Vector4d xstate){

    std::cout<<"test calculate control signal"<<std::endl;
    std::cout<<xstate(0)<<std::endl;
    std::cout<<Iz_<<std::endl;



  
    x_state_=xstate;

    // RCLCPP_ERROR(logger_lqr,"LQRRR x_state_ %.4f",x_state_(0));

    
    std::cout<<"test za xstate"<<std::endl;
     std::cout<<x_state_(0)<<std::endl;

    Eigen::RowVector4d K=get_K(v_x);

    K_=K;
=======
double LQR::calculate_control_signal(const double & v_x, const double & yaw_des_dot,const Eigen::Vector4d & xstate)
{
  RCLCPP_INFO(logger_, "Calculate control signal.");
  
  x_state_ = xstate;
  
  const Eigen::RowVector4d K = get_K(v_x,yaw_des_dot);

  RCLCPP_INFO(logger_, "K %f, %f, %f, %f ", K(0),K(1),K(2),K(3));
  

  double u = K * x_state_;

  prev_u_=u;
>>>>>>> Stashed changes

    double u = K*x_state_; 
    // RCLCPP_ERROR(logger_lqr,"LQRR x_state_ %.4f",u);
    if(std::isnan(u)){
        return 0.0;
    }else{

        return u;
    }
}






} // namespace lqr_lateral_controller

