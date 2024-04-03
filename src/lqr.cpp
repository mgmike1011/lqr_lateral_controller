#include "lqr_lateral_controller/lqr.hpp"

namespace lqr_lateral_controller
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
} // namespace lqr_lateral_controller
