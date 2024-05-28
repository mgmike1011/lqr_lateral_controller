#ifndef LQR_HPP_
#define LQR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include "rclcpp/rclcpp.hpp"


// implemenataion based on: https://ieeexplore.ieee.org/document/9051390
// LQR version with finite horizon

namespace lqr_lateral_controller
{
class LQR
{
private:
<<<<<<< Updated upstream

    const double Calfa_ = 150;  // [-] wheel tire leteral stiffness; for simplicity assumed to be the same for front and rear wheels
    // data taken from unity model, scripts and configs
    const double  m_ = 5; // [kg] mass
    const double lf_ = 0.114  ; // [m] front overhang length
    const double lr_ =  0.11 ; // [m] rear overhang length
    const double Iz_ = 0.02623506; // [kg*m^2]
    const int8_t N_ = 2; //horizon for riccati recursion 

    double R_ {};
    Eigen::Matrix4d  Q_ {};
    // std::shared_ptr<Eigen::Vector4d>  x_state_=std::make_shared<Eigen::Vector4d>(0,0,0,0);
    
    Eigen::Matrix4d get_A(double v_x);
    Eigen::Vector4d get_B();
    Eigen::Vector4d get_C(double v_x,double yaw_des_dot);
    Eigen::RowVector4d get_K(double v_x);
    void set_Q_(Eigen::Matrix4d C);
    // rclcpp::Logger logger_lqr;

    
=======
  double Calfa_ {};  // [-] wheel tire leteral stiffness; for simplicity assumed to be the
                              // same for front and rear wheels
  // Data taken from unity model, scripts and configs
  double m_ {};            // [kg] mass
  double lf_ {};       // [m] front overhang length
  double lr_ {};        // [m] rear overhang length
  double Iz_ {};  // [kg*m^2]
  int16_t N_ {};            // horizon for riccati recursion
  double R_{};
  double prev_u_{};
  double tc_{}; 

  Eigen::Vector4d x_state_;
  Eigen::Matrix4d Q_ = {};

  Eigen::Matrix4d get_A(const double & v_x);
  Eigen::Vector4d get_B();
  Eigen::Vector4d get_C(const double & v_x, const double & yaw_des_dot);
  Eigen::RowVector4d get_K(const double & v_x,const double & yaw_des_dot);
  void set_Q_(const Eigen::Matrix4d & C);
>>>>>>> Stashed changes

public:
    LQR();
    ~LQR() = default;

    Eigen::Vector4d x_state_ {} ;
    Eigen::RowVector4d K_{};



    

    rclcpp::Logger logger_ = rclcpp::get_logger("lqr");
    int64_t testObject(int64_t bar) const;

<<<<<<< Updated upstream
    void set_Q(Eigen::Vector4d gains);
    void set_R(double r);
    double calculate_control_signal(double v_x,Eigen::Vector4d xstate);
  
=======
  void set_Q(const Eigen::Vector4d & gains);
  void set_R(const double & r);
  double calculate_control_signal(const double & v_x, const double & yaw_des_dot,const Eigen::Vector4d & xstate);
>>>>>>> Stashed changes
};
}
#endif // LQR_HPP_
