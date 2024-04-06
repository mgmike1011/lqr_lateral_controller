#include "lqr_lateral_controller/lqr.hpp"

namespace lqr_
{

LQR::LQR()
{
    RCLCPP_ERROR(logger_, "LQR controller initialization.");
}

int64_t LQR::testObject(int64_t bar) const
{
    std::cout << "Hello from LQR controller" << bar << std::endl;
    return bar;
}

Eigen::Matrix4d LQR::get_A(double v_x){

    Eigen::Matrix4d A;

    A << 0, 1, 0, 0,
        0, -(2*Calfa_ + 2*Calfa_)/(m_ * v_x), (2*Calfa_+2*Calfa_)/m_, (-2*Calfa_*lf_ + 2*Calfa_*lr_)/(m_*v_x),
        0, 0, 0, 1,
        0, -(2*Calfa_*lf_ - 2*Calfa_*lr_)/(Iz_*v_x), (2*Calfa_*lf_ - 2*Calfa_*lr_)/Iz_, -(2*Calfa_*pow(lf_,2)+2*Calfa_*pow(lr_,2))/(Iz_*v_x);

    return A;    
}

Eigen::Vector4d LQR::get_B(){
    Eigen::Vector4d B;

    B<< 0, 2*Calfa_/m_, 0, 2*lf_*Calfa_/Iz_;

    return B; 

}

Eigen::Vector4d LQR::get_C(double v_x, double yaw_des_dot){

    Eigen::Vector4d C;

    C<< 0, (-(2*Calfa_*lf_-2*Calfa_*lr_)/(m_*v_x) - v_x)*yaw_des_dot, 0, -(2*Calfa_*pow(lf_,2) + 2*Calfa_*pow(lr_,2))/(Iz_*v_x) * yaw_des_dot;

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

Eigen::RowVector4d LQR::get_K(double v_x, double yaw_des_dot){

    Eigen::Matrix4d A =get_A(v_x);
    Eigen::Vector4d B =get_B();
    Eigen::Vector4d C =get_C(v_x,yaw_des_dot);
    Eigen::Matrix4d Q_f=C*C.transpose();
    Q_=Q_f;
    // set_Q_(C);
    set_R(0.2);

    Eigen::Matrix4d Pt = Q_;

    for(int i =0;i<N_;i++){

        double res=(R_+B.transpose()*Pt*B);

        auto Pt_1 = Q_ + A.transpose()*Pt*A - A.transpose()*Pt*B*(1/res)*B.transpose()*Pt*A;

        Pt=Pt_1;
    }

    double res = (R_ + B.transpose()*Pt*B);

    Eigen::RowVector4d K = -(1/res)*B.transpose()*Pt*A;

    return K;
}

double LQR::calculate_control_signal(double v_x, double yaw_des_dot,Eigen::Vector4d xstate){

    std::cout<<"test"<<std::endl;

    x_state_=xstate;

    Eigen::RowVector4d K=get_K(v_x,yaw_des_dot);

    double u = K*x_state_; 

    return u;
}






} // namespace lqr_lateral_controller

