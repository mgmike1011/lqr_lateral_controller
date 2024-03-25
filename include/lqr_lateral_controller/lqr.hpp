#ifndef LQR_HPP_
#define LQR_HPP_

#include <rclcpp/rclcpp.hpp>

namespace lqr_lateral_controller
{
class LQR
{
public:
    LQR();
    ~LQR() = default;

    rclcpp::Logger logger_ = rclcpp::get_logger("lqr");
    int64_t testObject(int64_t bar) const;
};
}
#endif // LQR_HPP_
